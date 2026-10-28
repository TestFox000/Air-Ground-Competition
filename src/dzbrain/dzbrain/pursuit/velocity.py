"""Arc-length speed planning and a stateful, jerk-limited command ramp.

The forward/backward pass is the maximum reachable *second-order* profile on
the supplied grid. It is not a jerk-constrained time-optimal solver. The final
profile uses monotone cubic velocity on each interval, zero acceleration at
the joins, and an analytically computed time dilation per stop-to-stop leg. Consequently its
acceleration is continuous and its (piecewise) jerk is bounded everywhere.
``second_order_duration_s`` is a comparison bound, not the final run time.

``speed_caps`` and curvature describe a sampled envelope. Between two nodes
the smaller endpoint cap applies. They must be positive; a zero-speed event
belongs in ``stop_indices``. A continuous geometry compiler must separately
bound curvature *between* its samples. This module cannot certify unseen
curvature peaks or unexpected, moving obstacles.

Stops have zero speed and zero acceleration. Dwell and reverse direction are
execution-layer events: this module always returns nonnegative speed. A
caller must advance the time parameter to depart a stop; sampling v(s) at a
stationary start is, correctly, zero and is not a start command.

An abrupt external hard limit can be incompatible with an existing velocity
and acceleration. ``MotionState.step`` makes that conflict explicit, gives
the hard limit priority, and marks ``constraint_override``. Such a safety
override must never be counted as a successful jerk-constrained motion.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, replace
from typing import Sequence


def _finite(value: float, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or value < 0.0 or (positive and value <= 0.0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return value


@dataclass(frozen=True)
class MotionSample:
    speed_mps: float
    acceleration_mps2: float
    jerk_mps3: float
    reason: str = "profile"
    constraint_override: bool = False
    s: float = 0.0


@dataclass(frozen=True)
class _Interval:
    s0: float
    s1: float
    v0: float
    v1: float
    duration: float
    cap: float
    envelope: tuple[tuple[float, float, float], ...] = ()

    def at(self, u: float) -> MotionSample:
        u = min(1.0, max(0.0, u))
        dv = self.v1 - self.v0
        # v(u) = v0 + dv smoothstep(u); exact integral, not an Euler estimate.
        speed = self.v0 + dv * (3.0 * u * u - 2.0 * u * u * u)
        s = self.s0 + self.duration * (
            self.v0 * u + dv * (u**3 - 0.5 * u**4))
        return MotionSample(speed, 6.0 * dv * u * (1.0 - u) / self.duration,
                            6.0 * dv * (1.0 - 2.0 * u) / self.duration**2,
                            s=s)

    def u_at(self, s: float) -> float:
        if s <= self.s0:
            return 0.0
        if s >= self.s1:
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(48):
            mid = (lo + hi) * 0.5
            arc = self.s0 + self.duration * (self.v0 * mid
                    + (self.v1-self.v0) * (mid**3 - .5*mid**4))
            if arc < s:
                lo = mid
            else:
                hi = mid
        return (lo + hi) * 0.5

    def envelope_excess(self) -> float:
        """Exact monotone-velocity maximum on each original cap cell."""
        if not self.envelope:
            return max(self.v0,self.v1)-self.cap
        return max(self.at(self.u_at(hi if self.v1 >= self.v0 else lo)).speed_mps-cap
                   for lo,hi,cap in self.envelope)


@dataclass(frozen=True)
class VelocityProfile:
    """Immutable, executable time parameterization of a sampled path."""

    s: tuple[float, ...]
    speeds_mps: tuple[float, ...]
    second_order_speeds_mps: tuple[float, ...]
    times_s: tuple[float, ...]
    stop_indices: tuple[int, ...]
    speed_scale: float
    second_order_duration_s: float
    _intervals: tuple[_Interval, ...]

    @property
    def duration_s(self) -> float:
        return self.times_s[-1]

    def sample_time(self, t: float) -> MotionSample:
        """Time is relative to path start; outside the interval it is clamped.

        This returns the planned sample only. Execution must pause at each
        required stop until the task authorizes departure.
        """
        if not math.isfinite(t):
            raise ValueError("time must be finite")
        i = min(len(self._intervals) - 1,
                max(0, bisect.bisect_right(self.times_s, t) - 1))
        interval = self._intervals[i]
        return interval.at((t - self.times_s[i]) / interval.duration)

    def time_at(self, s: float) -> float:
        if not math.isfinite(s):
            raise ValueError("arc length must be finite")
        i = min(len(self._intervals) - 1,
                max(0, bisect.bisect_right(self.s, s) - 1))
        interval = self._intervals[i]
        return self.times_s[i] + interval.u_at(s) * interval.duration

    def speed_at(self, s: float) -> float:
        return self.sample_time(self.time_at(s)).speed_mps

    def audit(self) -> dict[str, float]:
        """Analytic interval extrema, including endpoints and every stop."""
        return {
            "duration_s": self.duration_s,
            "second_order_duration_s": self.second_order_duration_s,
            "speed_scale": self.speed_scale,
            "max_accel_mps2": max(max(0.0, 1.5 * (p.v1 - p.v0) / p.duration)
                                    for p in self._intervals),
            "max_decel_mps2": max(max(0.0, 1.5 * (p.v0 - p.v1) / p.duration)
                                    for p in self._intervals),
            "max_abs_jerk_mps3": max(6.0 * abs(p.v1 - p.v0) / p.duration**2
                                       for p in self._intervals),
            "max_envelope_excess_mps": max(0.0, max(p.envelope_excess()
                                                      for p in self._intervals)),
            "max_stop_speed_mps": max((self.speeds_mps[i] for i in self.stop_indices),
                                        default=0.0),
        }


def _build_velocity_profile(
    s: Sequence[float], curvature: Sequence[float], *,
    speed_caps: Sequence[float] | None = None, cruise_mps: float,
    lat_acc_max_mps2: float, accel_up_mps2: float,
    accel_down_mps2: float, jerk_max_mps3: float,
    stop_indices: Sequence[int] = (), start_speed_mps: float = 0.0,
    end_speed_mps: float = 0.0,
) -> VelocityProfile:
    """Build a feasible profile; reject inconsistent or unreachable inputs.

    All s values must be strictly increasing and finite. Zero caps do not
    silently become a crawl: they are rejected, with zero-speed waypoints
    represented explicitly by stop_indices instead. If an interval connects
    two stops, an internal midpoint is inserted to represent acceleration
    followed by braking (a two-node zero/zero profile cannot traverse it).

    Global dilation preserves zero boundary speeds. When nonzero boundary
    speeds are requested and dilation is necessary, this implementation
    rejects the request rather than changing those fixed boundary conditions.
    """
    cruise = _finite(cruise_mps, "cruise_mps", positive=True)
    lat = _finite(lat_acc_max_mps2, "lat_acc_max_mps2", positive=True)
    up = _finite(accel_up_mps2, "accel_up_mps2", positive=True)
    down = _finite(accel_down_mps2, "accel_down_mps2", positive=True)
    jerk = _finite(jerk_max_mps3, "jerk_max_mps3", positive=True)
    start = _finite(start_speed_mps, "start_speed_mps")
    end = _finite(end_speed_mps, "end_speed_mps")
    ss = tuple(float(x) for x in s)
    kk = tuple(float(x) for x in curvature)
    if len(ss) < 2 or len(ss) != len(kk):
        raise ValueError("s and curvature need the same length, at least two")
    if any(not math.isfinite(x) for x in ss + kk):
        raise ValueError("s and curvature must be finite")
    if any(b <= a for a, b in zip(ss, ss[1:])):
        raise ValueError("s must be strictly increasing")
    if speed_caps is None:
        caps = [cruise] * len(ss)
    else:
        if len(speed_caps) != len(ss):
            raise ValueError("speed_caps must have one value per s node")
        caps = [_finite(x, "speed_cap (use stop_indices for stops)", positive=True)
                for x in speed_caps]
    caps = [min(c, cruise, math.sqrt(lat / abs(k)) if k else math.inf)
            for c, k in zip(caps, kk)]
    stops: set[int] = set()
    for i in stop_indices:
        if isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < len(ss):
            raise ValueError("stop index outside the sampled path")
        stops.add(i)
    if (0 in stops and start != 0.0) or (len(ss) - 1 in stops and end != 0.0):
        raise ValueError("a mandatory stop conflicts with nonzero boundary speed")
    if start == 0.0:
        stops.add(0)
    if end == 0.0:
        stops.add(len(ss) - 1)

    # Refine the speed grid independently of geometry sampling. A 4 m straight
    # with only two geometric endpoints still needs acceleration, cruise and
    # braking regions; inserting only one midpoint would incorrectly produce
    # a whole-route triangular speed curve. The local acceleration distance
    # sets the resolution; input nodes and exact stop locations are preserved.
    nodes, node_caps, forced = [], [], []
    for i, x in enumerate(ss):
        nodes.append(x)
        node_caps.append(caps[i])
        forced.append(i in stops)
        if i + 1 < len(ss):
            local_cap = min(caps[i], caps[i + 1])
            spacing = max(1e-4, min(0.05, local_cap**2 / (8.0 * max(up, down))))
            count = max(1, math.ceil((ss[i + 1] - x) / spacing))
            if i in stops and i + 1 in stops:
                count = max(2, count)
            if len(nodes) + count > 200000:
                raise ValueError("speed grid exceeds 200000 nodes; split the route")
            for j in range(1, count):
                nodes.append(x + (ss[i + 1] - x) * j / count)
                node_caps.append(local_cap)
                forced.append(False)
    interval_caps = [min(a, b) for a, b in zip(node_caps, node_caps[1:])]
    # Both velocities bounding each time interval must obey its whole cap;
    # monotone smoothstep then guarantees the complete interval obeys it.
    bound = [min(node_caps[i], interval_caps[max(0, i - 1)],
                 interval_caps[min(i, len(interval_caps) - 1)])
             for i in range(len(nodes))]
    if start > bound[0] or end > bound[-1]:
        raise ValueError("boundary speed exceeds the envelope")
    def reachable(a_up: float, a_down: float) -> list[float]:
        result = [0.0 if forced[i] else bound[i] for i in range(len(nodes))]
        result[0], result[-1] = start, end
        for i in range(1, len(nodes)):
            ds = nodes[i] - nodes[i - 1]
            result[i] = min(result[i], math.sqrt(result[i - 1]**2 + 2.0 * a_up * ds))
        for i in range(len(nodes) - 2, -1, -1):
            ds = nodes[i + 1] - nodes[i]
            result[i] = min(result[i], math.sqrt(result[i + 1]**2 + 2.0 * a_down * ds))
        if abs(result[0] - start) > 1e-12 or abs(result[-1] - end) > 1e-12:
            raise ValueError("boundary speeds are not reachable under acceleration limits")
        return result

    speeds = reachable(up, down)
    second_order = list(speeds)
    raw_times = []
    for i, (v0, v1) in enumerate(zip(speeds, speeds[1:])):
        if v0 + v1 <= 0.0:
            raise ValueError("positive-distance interval has no reachable positive speed")
        raw_times.append(2.0 * (nodes[i + 1] - nodes[i]) / (v0 + v1))
    second_order_duration = sum(raw_times)

    # Reserve the extra distance needed by a zero-acceleration S transition
    # before applying the general dilation proof. For a 0 -> V transition,
    # smoothstep has peak a = 1.5*a_mean and peak j = 6*a_mean**2/V.
    # Reserving this margin keeps an ordinary long straight at its full
    # cruise speed instead of shrinking the whole plateau to fix its ends.
    reserve_a = math.sqrt(jerk * min(bound) / 6.0)
    speeds = reachable(min(up / 1.5, reserve_a), min(down / 1.5, reserve_a))

    # Collapse monotone transitions with a common feasible envelope before
    # smoothing. Resetting acceleration at every 5 mm geometry sample would
    # make the jerk limit depend on sampling density and needlessly slow the
    # entire route. A merged interval is admitted only if its *whole* speed
    # range satisfies every crossed cell cap; mandatory stops remain nodes.
    def trend(dv: float) -> int:
        return 1 if dv > 1e-12 else (-1 if dv < -1e-12 else 0)

    kept = [0]
    merged_caps = []
    merged_envelopes = []
    i = 0
    while i + 1 < len(nodes):
        j = i + 1
        sign = trend(speeds[j] - speeds[i])
        cap = interval_caps[i]
        while j + 1 < len(nodes) and not forced[j]:
            if trend(speeds[j + 1] - speeds[j]) != sign:
                break
            next_cap = min(cap, interval_caps[j])
            if max(speeds[i], speeds[j + 1]) > next_cap + 1e-12:
                # A varying envelope need not hold its lowest speed over the
                # entire transition. Certify the actual monotone v(s) at each
                # cell's worst endpoint instead of comparing unrelated maxima.
                duration = 2.0*(nodes[j+1]-nodes[i])/(speeds[i]+speeds[j+1])
                candidate = _Interval(nodes[i],nodes[j+1],speeds[i],speeds[j+1],duration,next_cap,
                                      tuple((nodes[k],nodes[k+1],interval_caps[k]) for k in range(i,j+1)))
                if candidate.envelope_excess() > 1e-12:
                    break
            cap = next_cap
            j += 1
        kept.append(j)
        merged_caps.append(cap)
        merged_envelopes.append(tuple((nodes[k],nodes[k+1],interval_caps[k]) for k in range(i,j)))
        i = j
    nodes = [nodes[i] for i in kept]
    speeds = [speeds[i] for i in kept]
    second_order = [second_order[i] for i in kept]
    forced = [forced[i] for i in kept]
    interval_caps = merged_caps

    times = []
    scale = 1.0
    for i, (v0, v1) in enumerate(zip(speeds, speeds[1:])):
        if v0 + v1 <= 0.0:
            raise ValueError("positive-distance interval has no reachable positive speed")
        duration = 2.0 * (nodes[i + 1] - nodes[i]) / (v0 + v1)
        times.append(duration)
        dv = v1 - v0
        peak_a = 1.5 * abs(dv) / duration
        peak_j = 6.0 * abs(dv) / duration**2
        if peak_a:
            scale = min(scale, math.sqrt((up if dv >= 0.0 else down) / peak_a))
        if peak_j:
            scale = min(scale, (jerk / peak_j)**(1.0 / 3.0))
    if scale < 1.0 - 1e-12 and (start != 0.0 or end != 0.0):
        raise ValueError("jerk smoothing needs time dilation incompatible with fixed nonzero boundary speeds")
    if scale < 1.0:
        scale *= 1.0 - 1e-12
    final = tuple(v * scale for v in speeds)
    intervals = tuple(_Interval(nodes[i], nodes[i + 1], final[i], final[i + 1],
                                times[i] / scale, interval_caps[i], merged_envelopes[i])
                      for i in range(len(times)))
    prefix = [0.0]
    for interval in intervals:
        prefix.append(prefix[-1] + interval.duration)
    return VelocityProfile(tuple(nodes), final, tuple(second_order), tuple(prefix),
                           tuple(i for i, f in enumerate(forced) if f), scale,
                           second_order_duration, intervals)


def _select_velocity_profile(
    s: Sequence[float], curvature: Sequence[float], *,
    speed_caps: Sequence[float] | None = None, cruise_mps: float,
    lat_acc_max_mps2: float, accel_up_mps2: float,
    accel_down_mps2: float, jerk_max_mps3: float,
    stop_indices: Sequence[int] = (), start_speed_mps: float = 0.0,
    end_speed_mps: float = 0.0,
) -> VelocityProfile:
    """Select the faster of two certified, conservative jerk-feasible profiles.

    The first follows the varying cap. The second uses the smallest cap across
    the route; avoiding numerous small speed changes can outweigh its reduced
    cruise speed when the first needs substantial global time dilation. Both
    obey every original cell cap and stop. This bounded candidate comparison
    is not a proof of global time optimality with jerk constraints.
    """
    options = dict(cruise_mps=cruise_mps, lat_acc_max_mps2=lat_acc_max_mps2,
                   accel_up_mps2=accel_up_mps2, accel_down_mps2=accel_down_mps2,
                   jerk_max_mps3=jerk_max_mps3, stop_indices=stop_indices,
                   start_speed_mps=start_speed_mps, end_speed_mps=end_speed_mps)
    profile = _build_velocity_profile(s, curvature, speed_caps=speed_caps, **options)
    caps = [min(cruise_mps, speed_caps[i] if speed_caps is not None else cruise_mps,
                math.sqrt(lat_acc_max_mps2 / abs(k)) if k else math.inf)
            for i, k in enumerate(curvature)]
    if max(caps) - min(caps) <= 1e-12 or min(caps) < max(start_speed_mps, end_speed_mps):
        return profile
    uniform = _build_velocity_profile(s, curvature, speed_caps=[min(caps)] * len(s), **options)
    if uniform.duration_s < profile.duration_s:
        return replace(uniform, second_order_duration_s=profile.second_order_duration_s)
    return profile


def build_velocity_profile(
    s: Sequence[float], curvature: Sequence[float], *,
    speed_caps: Sequence[float] | None = None, cruise_mps: float,
    lat_acc_max_mps2: float, accel_up_mps2: float,
    accel_down_mps2: float, jerk_max_mps3: float,
    stop_indices: Sequence[int] = (), start_speed_mps: float = 0.0,
    end_speed_mps: float = 0.0,
) -> VelocityProfile:
    """Build independently between existing zero-speed, zero-acceleration stops.

    These boundaries separate feasibility exactly: concatenating feasible
    legs preserves speed/acceleration continuity and bounded one-sided jerk.
    A local low cap or dilation therefore need not slow unrelated legs. No
    additional stops are introduced. Keep the whole-route candidate as a
    fallback so this refinement never increases its travel time. This is a
    certified conservative construction, not a globally time-optimal solver.
    """
    options = dict(cruise_mps=cruise_mps, lat_acc_max_mps2=lat_acc_max_mps2,
                   accel_up_mps2=accel_up_mps2, accel_down_mps2=accel_down_mps2,
                   jerk_max_mps3=jerk_max_mps3)
    whole = _select_velocity_profile(
        s, curvature, speed_caps=speed_caps, stop_indices=stop_indices,
        start_speed_mps=start_speed_mps, end_speed_mps=end_speed_mps, **options)
    # Whole-route construction above validates indices and all boundary
    # conditions before any slicing, including invalid bool/duplicate s input.
    cuts = sorted({0, len(s)-1, *stop_indices})
    if len(cuts) <= 2:
        return whole
    legs = []
    for lo, hi in zip(cuts, cuts[1:]):
        legs.append(_select_velocity_profile(
            s[lo:hi+1], curvature[lo:hi+1],
            speed_caps=None if speed_caps is None else speed_caps[lo:hi+1],
            stop_indices=tuple(i-lo for i in stop_indices if lo <= i <= hi),
            start_speed_mps=start_speed_mps if lo == 0 else 0.0,
            end_speed_mps=end_speed_mps if hi == len(s)-1 else 0.0, **options))
    if sum(p.duration_s for p in legs) >= whole.duration_s:
        return whole
    nodes, speeds, second, times, stops, intervals = [], [], [], [], set(), []
    time_offset = 0.0
    for leg in legs:
        index_offset = len(nodes)-1 if nodes else 0
        start = 1 if nodes else 0
        nodes.extend(leg.s[start:])
        speeds.extend(leg.speeds_mps[start:])
        second.extend(leg.second_order_speeds_mps[start:])
        times.extend(time_offset+t for t in leg.times_s[start:])
        stops.update(index_offset+i for i in leg.stop_indices)
        intervals.extend(leg._intervals)
        time_offset += leg.duration_s
    return VelocityProfile(tuple(nodes), tuple(speeds), tuple(second), tuple(times),
                           tuple(sorted(stops)), min(p.speed_scale for p in legs),
                           sum(p.second_order_duration_s for p in legs), tuple(intervals))


class SpeedEnvelope:
    """Sampled geometric cap with a discrete jerk-braking viability filter.

    The same smaller-endpoint rule as the offline profile applies to every
    spatial cell. A trial first acceleration is followed by maximum feasible
    jerk-limited braking to rest. Because those trajectories are ordered by
    first acceleration, bisection finds its safe upper bound. This is a
    receding-horizon safety filter, not a second independent speed planner.
    """

    def __init__(self, s: Sequence[float], caps: Sequence[float]):
        self.s, self.caps = tuple(map(float,s)), tuple(map(float,caps))
        if len(self.s)<2 or len(self.s)!=len(self.caps):
            raise ValueError("speed envelope needs matching node and cap arrays")
        if any(not math.isfinite(x) for x in self.s+self.caps) or any(c<=0 for c in self.caps):
            raise ValueError("speed envelope values must be finite and caps positive")
        if any(b<=a for a,b in zip(self.s,self.s[1:])):
            raise ValueError("speed envelope nodes must strictly increase")
        self.cells = tuple(min(a,b) for a,b in zip(self.caps,self.caps[1:]))

    def minimum(self, start_s: float, end_s: float) -> float:
        lo = min(len(self.cells)-1,max(0,bisect.bisect_right(self.s,start_s)-1))
        hi = min(len(self.cells)-1,max(lo,bisect.bisect_left(self.s,end_s)-1))
        return min(self.cells[lo:hi+1])

    def acceleration_ceiling(self, s: float, speed: float, acceleration: float, dt: float, *,
                             accel_up_mps2: float, accel_down_mps2: float,
                             jerk_max_mps3: float, speed_scale: float = 1.0,
                             end_s: float | None = None, delay_s: float = 0.0) -> tuple[float,bool]:
        """Return (largest viable next acceleration, feasible from this state).

        Delay reserves the distance an already-issued command can travel at
        the configured maximum acceleration; it does not alter steering or
        speed parameters. A false result means even immediate maximum legal
        braking cannot certify the sampled envelope: it must be reported.
        """
        up = _finite(accel_up_mps2,"accel_up_mps2",positive=True)
        down = _finite(accel_down_mps2,"accel_down_mps2",positive=True)
        jerk = _finite(jerk_max_mps3,"jerk_max_mps3",positive=True)
        dt = _finite(dt,"dt",positive=True)
        speed = _finite(speed,"speed")
        speed_scale = _finite(speed_scale,"speed_scale",positive=True)
        delay_s = _finite(delay_s,"delay_s")
        if not math.isfinite(s) or not math.isfinite(acceleration) or acceleration < -down-1e-12 or acceleration > up+1e-12:
            raise ValueError("invalid position or acceleration state for speed-envelope filter")
        acceleration = min(up,max(-down,acceleration))
        da = jerk*dt
        lower,upper = max(-down,acceleration-da),min(up,acceleration+da)
        unwind_ticks = max(1,int(math.floor((math.sqrt(1.+8.*max(speed,0.)/(jerk*dt*dt))-1.)*.5))+1)
        unwind_limit = (max(speed,0.)/dt+da*unwind_ticks*(unwind_ticks-1)*.5)/unwind_ticks
        lower = max(lower,-unwind_limit)
        if lower > upper+1e-12:
            return max(-down,acceleration-da),False
        end = self.s[-1] if end_s is None else min(self.s[-1],end_s)
        position = min(end,s+speed*delay_s+.5*up*delay_s**2)
        # There is no need to simulate braking on a uniform/high-cap region
        # when even the transient speed gained while unwinding a>0 is safe.
        peak_bound = speed+max(0.,upper)*dt+max(0.,upper)**2/(2.*jerk)+max(0.,upper)*dt
        if peak_bound <= self.minimum(s,end)*speed_scale+1e-12:
            return upper,True
        max_ticks = max(4,math.ceil((max(speed,0.)/down+(up+2.*down)/jerk+2.*dt)/dt))

        def viable(first_a):
            v,a,x = speed,acceleration,position
            for tick in range(max_ticks):
                if tick==0:
                    next_a = first_a
                else:
                    # Exact discrete acceleration that can unwind to zero
                    # without crossing zero speed: invert the triangular sum.
                    n = max(1,int(math.floor((math.sqrt(1.+8.*max(v,0.)/(jerk*dt*dt))-1.)*.5))+1)
                    unwind = min(down,(max(v,0.)/dt+da*n*(n-1)*.5)/n)
                    next_a = min(up,max(-down,min(a+da,max(a-da,-unwind))))
                next_v = v+next_a*dt
                if next_v < -1e-10:
                    return False
                next_v = max(0.,next_v)
                next_x = min(end,x+next_v*dt)
                cap = self.minimum(s if tick==0 else x,next_x)*speed_scale
                if next_v > cap+1e-12:
                    return False
                v,a,x = next_v,next_a,next_x
                if (v<=1e-12 and abs(a)<=da+1e-12) or x>=end:
                    return True
            return False

        if viable(upper):
            return upper,True
        if not viable(lower):
            return lower,False
        lo,hi=lower,upper
        for _ in range(24):
            mid=(lo+hi)*.5
            if viable(mid):lo=mid
            else:hi=mid
        return lo,True


@dataclass
class MotionState:
    """Per-executor state; never share one instance between vehicles/routes."""

    speed_mps: float = 0.0
    acceleration_mps2: float = 0.0

    def reset(self, speed_mps: float = 0.0, acceleration_mps2: float = 0.0) -> None:
        self.speed_mps = _finite(speed_mps, "speed_mps")
        acceleration_mps2 = float(acceleration_mps2)
        if not math.isfinite(acceleration_mps2):
            raise ValueError("acceleration must be finite")
        self.acceleration_mps2 = acceleration_mps2

    def step(self, target_mps: float, dt: float, *, accel_up_mps2: float,
             accel_down_mps2: float, jerk_max_mps3: float,
             hard_limit_mps: float | None = None,
             acceleration_ff_mps2: float | None = None,
             velocity_gain_s: float = 1.0,
             acceleration_ceiling_mps2: float | None = None) -> MotionSample:
        """Bound successive command acceleration and jerk, with safety priority.

        The desired speed is a soft target. A sudden reduction can require
        continued positive acceleration while jerk unwinds. Pass an external
        safety request as hard_limit_mps to forbid that, with an explicit
        infeasibility flag. The optional hard limit is applied last.
        """
        target = _finite(target_mps, "target_mps")
        dt = _finite(dt, "dt", positive=True)
        up = _finite(accel_up_mps2, "accel_up_mps2", positive=True)
        down = _finite(accel_down_mps2, "accel_down_mps2", positive=True)
        jerk = _finite(jerk_max_mps3, "jerk_max_mps3", positive=True)
        limit = None if hard_limit_mps is None else _finite(hard_limit_mps, "hard_limit_mps")
        old_v = _finite(self.speed_mps, "state speed")
        old_a = float(self.acceleration_mps2)
        tolerance = 1e-12 * max(1.0, up, down)
        if not math.isfinite(old_a) or old_a < -down - tolerance or old_a > up + tolerance:
            raise ValueError("state acceleration outside the configured limits; reset explicitly")
        old_a = min(up, max(-down, old_a))
        if limit is not None:
            target = min(target, limit)
        error = target - old_v
        direction = 1.0 if error >= 0.0 else -1.0
        max_a = up if direction > 0.0 else down
        step_a = jerk * dt

        # Select acceleration that leaves enough velocity error to unwind it
        # in discrete future steps. This avoids target overshoot in a feasible
        # monotone ramp without the jerk-breaking "clip speed to target" trick.
        lo, hi = 0.0, max_a
        for _ in range(48):
            a = (lo + hi) * 0.5
            n = max(1, int(math.ceil(a / step_a)))
            remaining_dv = dt * (n * a - step_a * n * (n - 1) * 0.5)
            if remaining_dv <= abs(error):
                lo = a
            else:
                hi = a
        wanted_a = direction * lo
        if acceleration_ff_mps2 is not None:
            ff = float(acceleration_ff_mps2)
            gain = _finite(velocity_gain_s, "velocity_gain_s", positive=True)
            if not math.isfinite(ff):
                raise ValueError("acceleration feedforward must be finite")
            wanted_a = ff + gain * error
            # The feedforward branch tracks a moving reference, but it must
            # retain enough positive speed to unwind any negative acceleration
            # before reaching zero. This is the same discrete viability bound.
            lo_stop, hi_stop = 0.0, down
            for _ in range(48):
                a = (lo_stop + hi_stop) * .5
                n = max(1, int(math.ceil(a / step_a)))
                loss = dt * (n * a - step_a * n * (n - 1) * .5)
                if loss <= old_v:
                    lo_stop = a
                else:
                    hi_stop = a
            wanted_a = max(wanted_a, -lo_stop)
            if limit is not None and limit >= old_v:
                lo_cap, hi_cap = 0.0, up
                for _ in range(48):
                    a = (lo_cap + hi_cap) * .5
                    n = max(1, int(math.ceil(a / step_a)))
                    gain_v = dt * (n * a - step_a * n * (n - 1) * .5)
                    if gain_v <= limit - old_v:
                        lo_cap = a
                    else:
                        hi_cap = a
                wanted_a = min(wanted_a, lo_cap)
        if acceleration_ceiling_mps2 is not None:
            ceiling = float(acceleration_ceiling_mps2)
            if not math.isfinite(ceiling) or ceiling < -down-tolerance or ceiling > up+tolerance:
                raise ValueError("acceleration ceiling outside the configured bounds")
            wanted_a = min(wanted_a,ceiling)
        new_a = min(up, max(-down, min(old_a + step_a, max(old_a - step_a, wanted_a))))
        new_v = old_v + new_a * dt
        override = new_v < -1e-12 or (limit is not None and new_v > limit + 1e-12)
        new_v = max(0.0, new_v)
        if limit is not None:
            new_v = min(new_v, limit)
        if new_v < 1e-14:
            new_v = 0.0       # roundoff only; removes a nonzero rest fixed point
        actual_a = (new_v - old_v) / dt
        actual_j = (actual_a - old_a) / dt
        # Following an emergency clip, reseed comfort acceleration at rest or
        # within limits. The returned actual values still expose the violation.
        self.speed_mps = new_v
        self.acceleration_mps2 = 0.0 if override else min(up, max(-down, actual_a))
        return MotionSample(new_v, actual_a, actual_j,
                            "hard_limit_override" if override else "jerk_ramp",
                            override)
