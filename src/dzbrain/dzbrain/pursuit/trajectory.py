# -*- coding: utf-8 -*-
"""Curvature-continuous local compilation of an existing ordered Route.

This is a trajectory connector, not a route-order planner. Mandatory task points
and bridge approach/departure poses stay fixed; ordinary grid centres are guides.
Each moving piece is a clothoid (linear curvature in arc length) or a quintic
Bezier connection whose curvature bound is certified by Bernstein subdivision.
Reversals are zero-curvature, zero-speed cusps, never a yaw-only manoeuvre.

A search exhaustion reports *this compiler did not find a connection*. It is not
proof that the map is impossible. A caller must supply ``pose_valid`` for obstacle
aware search and independently validate the complete swept trajectory before use.
The legacy Path is used only as the spatial projection index; returned geometry,
heading and curvature are always evaluated on the continuous curve.
"""
from __future__ import annotations

import bisect
import heapq
import itertools
import math
from dataclasses import dataclass, replace
from functools import lru_cache
from typing import Callable

from ..core import geometry as geo
from ..route.model import Route, Waypoint
from .path import Path, Projection

__all__ = ["Trajectory", "TrajectorySample", "TrajectoryInfeasible"]


class TrajectoryInfeasible(ValueError):
    """No certified connector was found within the finite search budget."""

    def __init__(self, message: str, *, waypoint: int | None = None):
        self.waypoint = waypoint
        self.errors = (message,)
        super().__init__(message)


@dataclass(frozen=True)
class TrajectorySample:
    x: float
    y: float
    yaw: float                 # direction of travel, not body heading
    curvature: float           # d(travel yaw)/ds
    v_max: float
    reverse: bool
    s: float
    seg_index: int
    curvature_rate: float = 0.0  # d(curvature)/ds, for steering-rate feasibility

    @property
    def body_yaw(self) -> float:
        return geo.normalize_angle(self.yaw + (math.pi if self.reverse else 0.0))


# Eight-point Gauss-Legendre quadrature, paired about the interval midpoint.
_GL = ((0.1834346424956498, 0.3626837833783620),
       (0.5255324099163290, 0.3137066458778873),
       (0.7966664774136267, 0.2223810344533745),
       (0.9602898564975363, 0.1012285362903763))


def _integrate(f, lo, hi):
    mid, half = (lo + hi) / 2.0, (hi - lo) / 2.0
    return half * sum(w * (f(mid - half * z) + f(mid + half * z)) for z, w in _GL)


def _bezier(points, t):
    work = list(points)
    while len(work) > 1:
        work = [((1.0 - t) * a[0] + t * b[0],
                 (1.0 - t) * a[1] + t * b[1]) for a, b in zip(work, work[1:])]
    return work[0]


def _derivative(points):
    n = len(points) - 1
    return tuple((n * (b[0] - a[0]), n * (b[1] - a[1]))
                 for a, b in zip(points, points[1:]))


def _split(points):
    row = list(points)
    left, right = [row[0]], [row[-1]]
    while len(row) > 1:
        row = [((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
               for a, b in zip(row, row[1:])]
        left.append(row[0]); right.append(row[-1])
    return tuple(left), tuple(reversed(right))


def _curvature_bound(points, limit, depth=0):
    """Conservative proof of |cross(p',p'')| / |p'|^3 <= limit on [0,1].

    Bernstein's convex-hull property bounds the numerator polynomial and the
    derivative box. Subdivision tightens both; failure is never treated as safe.
    """
    d = _derivative(points); dd = _derivative(d)
    mins = [min(p[k] for p in d) for k in (0, 1)]
    maxs = [max(p[k] for p in d) for k in (0, 1)]
    lower = [max(mins[k], -maxs[k], 0.0) for k in (0, 1)]
    speed2 = lower[0] ** 2 + lower[1] ** 2
    cross = []
    for k in range(8):
        v = 0.0
        for i in range(max(0, k - 3), min(4, k) + 1):
            j = k - i
            v += (math.comb(4, i) * math.comb(3, j) / math.comb(7, k)
                  * (d[i][0] * dd[j][1] - d[i][1] * dd[j][0]))
        cross.append(v)
    bound = max(abs(v) for v in cross) / speed2 ** 1.5 if speed2 > 1e-20 else math.inf
    if bound <= limit:
        return True
    # Reject a witnessed violation before spending time subdividing.
    for t in (0.0, 0.5, 1.0):
        dx, dy = _bezier(d, t); ax, ay = _bezier(dd, t)
        q = dx * dx + dy * dy
        if q < 1e-16 or abs(dx * ay - dy * ax) > limit * q ** 1.5 * (1.0 + 1e-12):
            return False
    if depth >= 14:
        return False
    a, b = _split(points)
    return _curvature_bound(a, limit, depth + 1) and _curvature_bound(b, limit, depth + 1)


class _Clothoid:
    def __init__(self, x, y, yaw, k0, k1, length, reverse=False):
        self.x, self.y, self.yaw = x, y, yaw
        self.k0, self.k1, self.length = k0, k1, length
        self.reverse = bool(reverse)

    def at(self, s):
        s = min(self.length, max(0.0, s))
        rate = (self.k1 - self.k0) / self.length
        angle = lambda u: self.yaw + self.k0 * u + 0.5 * rate * u * u
        if abs(self.k0) + abs(rate) < 1e-14:
            x, y = self.x + s * math.cos(self.yaw), self.y + s * math.sin(self.yaw)
        else:
            x = self.x + _integrate(lambda u: math.cos(angle(u)), 0.0, s)
            y = self.y + _integrate(lambda u: math.sin(angle(u)), 0.0, s)
        return x, y, geo.normalize_angle(angle(s)), self.k0 + rate * s

    def _state_at(self, s):
        return (*self.at(s), (self.k1-self.k0)/self.length)


class _Quintic:
    def __init__(self, points, reverse=False):
        self.points = tuple(points)
        self.d = _derivative(points); self.dd = _derivative(self.d)
        self.ddd = _derivative(self.dd)
        self.reverse = bool(reverse)
        # Arc length integration is of the analytic derivative, not chord sums.
        span = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        count = max(16, math.ceil(span / 0.01))
        self._ts = tuple(i / count for i in range(count + 1))
        arc = [0.0]
        for lo, hi in zip(self._ts, self._ts[1:]):
            arc.append(arc[-1] + _integrate(self._speed, lo, hi))
        self._arc = tuple(arc); self.length = arc[-1]

    def _speed(self, t):
        dx, dy = _bezier(self.d, t)
        return math.hypot(dx, dy)

    def _state_at(self, s):
        s = min(self.length, max(0.0, s))
        i = min(len(self._arc) - 2, max(0, bisect.bisect_right(self._arc, s) - 1))
        lo, hi = self._ts[i], self._ts[i + 1]
        span = self._arc[i + 1] - self._arc[i]
        t = lo + (hi - lo) * (s - self._arc[i]) / span if span > 1e-15 else lo
        # Two local Newton steps keep s geometric even on variable parameter speed.
        for _ in range(2):
            residual = self._arc[i] + _integrate(self._speed, lo, t) - s
            t = min(hi, max(lo, t - residual / max(self._speed(t), 1e-12)))
        x, y = _bezier(self.points, t); dx, dy = _bezier(self.d, t)
        ax, ay = _bezier(self.dd, t)
        jx, jy = _bezier(self.ddd, t)
        q = dx*dx+dy*dy; cross = dx*ay-dy*ax
        k = cross/q**1.5
        k_s = ((dx*jy-dy*jx)*q-3*cross*(dx*ax+dy*ay))/q**3
        return x, y, math.atan2(dy, dx), k, k_s

    def at(self, s):
        return self._state_at(s)[:4]


def _quintic_controls(start, goal, a, b):
    x0, y0, h0, k0 = start; x1, y1, h1, k1 = goal
    u0 = math.cos(h0), math.sin(h0); u1 = math.cos(h1), math.sin(h1)
    n0 = -u0[1], u0[0]; n1 = -u1[1], u1[0]
    return ((x0, y0), (x0 + a * u0[0] / 5, y0 + a * u0[1] / 5),
            (x0 + 2 * a * u0[0] / 5 + k0 * a * a * n0[0] / 20,
             y0 + 2 * a * u0[1] / 5 + k0 * a * a * n0[1] / 20),
            (x1 - 2 * b * u1[0] / 5 + k1 * b * b * n1[0] / 20,
             y1 - 2 * b * u1[1] / 5 + k1 * b * b * n1[1] / 20),
            (x1 - b * u1[0] / 5, y1 - b * u1[1] / 5), (x1, y1))


def _valid_piece(piece, pose_valid):
    if pose_valid is None:
        return True
    count = max(1, math.ceil(piece.length / 0.01))
    for i in range(count + 1):
        x, y, yaw, _ = piece.at(piece.length * i / count)
        if not pose_valid(x, y, geo.normalize_angle(yaw + (math.pi if piece.reverse else 0.0))):
            return False
    return True


def _direct(start, goal, reverse, kmax, pose_valid):
    """Shortest accepted member of a fixed, map-independent pose connector family."""
    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    if distance < 1e-8:
        return None
    # A near-straight connector is common and cheap; larger handles allow outward arcs.
    best = None
    for f0, f1 in ((1., 1.), (1.25, 1.25), (.75, .75), (1.5, 1.5),
                   (.5, 1.), (1., .5), (.75, 1.5), (1.5, .75), (2., 2.)):
        controls = _quintic_controls(start, goal, distance * f0, distance * f1)
        if not _curvature_bound(controls, kmax):
            continue
        candidate = _Quintic(controls, reverse)
        if (best is None or candidate.length < best.length) and _valid_piece(candidate, pose_valid):
            best = candidate
    return best


@dataclass
class _Node:
    x: float
    y: float
    yaw: float                 # body yaw
    k: float                   # steering curvature
    direction: int
    cost: float
    parent: int
    piece: object | None


def _connect(start, goal, direction, kmax, pose_valid, guide, max_nodes, goal_yaws=None,
             strict_reverse=False, goal_poses=None):
    """Hybrid local pose search using G2 pieces; task order is immutable."""
    sx, sy, sh = start; gx, gy, gh = goal
    targets = tuple(goal_poses or ((gx,gy,h) for h in (goal_yaws or (gh,))))
    allowed_directions = (-1,) if strict_reverse else (direction, -direction)
    travel_goals = {sign: tuple((x, y, geo.normalize_angle(h + (math.pi if sign < 0 else 0)), 0.0)
                               for x,y,h in targets) for sign in allowed_directions}
    direct = None
    for sign in allowed_directions:
        travel_start = (sx, sy, geo.normalize_angle(sh + (math.pi if sign < 0 else 0)), 0.0)
        for target in travel_goals[sign]:
            candidate = _direct(travel_start, target, sign < 0, kmax, pose_valid)
            if candidate is not None and (direct is None or candidate.length < direct.length-1e-9):
                direct = candidate
    if direct is not None: return [direct], 0
    # The local search may leave the polyline. Its finite box is an algorithmic budget,
    # not a physical infeasibility certificate.
    coords = [(sx, sy)] + [(x,y) for x,y,_ in targets] + list(guide)
    minx, maxx = min(p[0] for p in coords) - .8, max(p[0] for p in coords) + .8
    miny, maxy = min(p[1] for p in coords) - .8, max(p[1] for p in coords) + .8
    nodes = [_Node(sx, sy, sh, 0.0, direction, 0.0, -1, None)]
    counter = itertools.count()
    queue = [(math.hypot(gx-sx, gy-sy), next(counter), 0)]
    visited = {}
    length = .12
    def key(n):
        return (round(n.x/.04), round(n.y/.04), round(geo.normalize_angle(n.yaw)/(math.pi/18)),
                round(n.k/kmax), n.direction)
    def heuristic(n):
        return min(max(math.hypot(x-n.x,y-n.y),abs(geo.normalize_angle(h-n.yaw))/kmax)
                   for x,y,h in targets)
    expansions = 0
    while queue and expansions < max_nodes:
        _, _, idx = heapq.heappop(queue); node = nodes[idx]
        state = key(node)
        if visited.get(state, math.inf) <= node.cost - 1e-9:
            continue
        visited[state] = node.cost; expansions += 1
        distance = min(math.hypot(node.x-x,node.y-y) for x,y,_ in targets)
        if node.direction in allowed_directions and distance < .65 and expansions % 2 == 0:
            start_t = (node.x, node.y, geo.normalize_angle(node.yaw + (math.pi if node.direction < 0 else 0)),
                       node.direction * node.k)
            end_piece = None
            for target in travel_goals[node.direction]:
                candidate = _direct(start_t, target, node.direction < 0, kmax, pose_valid)
                if candidate is not None and (end_piece is None or candidate.length < end_piece.length):
                    end_piece = candidate
            if end_piece is not None:
                pieces = [end_piece]
                while idx > 0:
                    p = nodes[idx]
                    if p.piece is not None: pieces.append(p.piece)
                    idx = p.parent
                return list(reversed(pieces)), expansions
        directions = ((node.direction, -node.direction)
                      if abs(node.k) < 1e-12 and not strict_reverse else (node.direction,))
        for sign in directions:
            for k in (-kmax, 0.0, kmax):
                h = geo.normalize_angle(node.yaw + (math.pi if sign < 0 else 0.0))
                piece = _Clothoid(node.x, node.y, h, sign * node.k, sign * k, length, sign < 0)
                x, y, travel_yaw, _ = piece.at(length)
                body_yaw = geo.normalize_angle(travel_yaw + (math.pi if sign < 0 else 0))
                if not (minx <= x <= maxx and miny <= y <= maxy): continue
                candidate = _Node(x, y, body_yaw, k, sign,
                                  node.cost + length * (1.08 if sign < 0 else 1.)
                                  + (.24 if sign != node.direction else 0.)
                                  + .002 * abs(k-node.k)/kmax, idx, piece)
                if visited.get(key(candidate), math.inf) <= candidate.cost: continue
                if not _valid_piece(piece, pose_valid): continue
                nodes.append(candidate)
                heapq.heappush(queue, (candidate.cost + 1.12*heuristic(candidate), next(counter), len(nodes)-1))
    raise TrajectoryInfeasible(f"local connector search exhausted ({expansions} nodes) from {start} to {goal}; not a proof of map infeasibility")


class Trajectory(Path):
    """Ordered Route compiled into G2 moving intervals and stationary cusps.

    ``field`` may be a FieldSpec-like object or a map dict. ``pose_valid`` is the
    authoritative field footprint predicate, evaluated at <= 1 cm spacing during
    search. The caller still owns independent continuous/swept acceptance.
    """
    def __init__(self, route: Route, cfg, field=None, *, pose_valid: Callable | None = None,
                 max_nodes: int = 18000, start_pose: geo.Pose | None = None,
                 start_yaw: float | None = None):
        if len(route.waypoints) < 2:
            raise TrajectoryInfeasible("at least two waypoints are required")
        self._route = route
        original_points = tuple(route.waypoints)
        self._points = original_points
        self._cfg = cfg
        if start_pose is not None and start_yaw is not None:
            raise ValueError("pass start_pose or start_yaw, not both")
        if start_pose is not None and not all(math.isfinite(v) for v in
                (start_pose.x,start_pose.y,start_pose.yaw)):
            raise TrajectoryInfeasible("initial pose must be finite")
        if start_pose is not None and math.hypot(start_pose.x-original_points[0].x,
                                               start_pose.y-original_points[0].y)>1e-9:
            raise TrajectoryInfeasible("initial pose position must equal Route.start; arbitrary rejoining is unsupported")
        if start_yaw is not None and not math.isfinite(start_yaw):
            raise TrajectoryInfeasible("initial yaw must be finite")
        wheelbase = float(cfg.get("pursuit.wheelbase_m"))
        angle = float(cfg.get("pursuit.max_steer_deg"))
        if not (math.isfinite(wheelbase) and wheelbase > 0 and 0 < angle < 90):
            raise TrajectoryInfeasible("invalid wheelbase or steering bound")
        self.max_curvature = math.tan(math.radians(angle)) / wheelbase
        for w in self._points:
            if not all(math.isfinite(v) for v in (w.x, w.y, w.yaw, w.speed_scale)):
                raise TrajectoryInfeasible("nonfinite waypoint")
        field_data = field.get("map", field) if isinstance(field, dict) else field
        def entries(name):
            return tuple(field_data.get(name, ())) if isinstance(field_data, dict) else tuple(getattr(field_data, name, ()))
        mines, bridges = set(entries("mines")), set(entries("bridges"))
        # All curve positions are the bicycle rear-axle reference. A mine visit
        # constrains the physical body centre, whose position depends on yaw.
        ref_offset=float(cfg.get("vehicle.ref_offset_m",0.0))
        # An unmarked out-and-back guide excursion carries no task obligation.
        # Keeping its far vertex as a pose constraint would recreate PIVOT in the
        # compiler and can make a feasible mine visit look impossible (e.g. the
        # vehicle nose at G8 beside obstacle F8). Preserve explicit R commands and
        # all stops/mines/bridge gates; only eliminate the redundant guide detour.
        kept = list(range(len(original_points)))
        if field is not None:
            changed = True
            while changed:
                changed = False
                for j in range(1, len(kept)-1):
                    a,b,c = (original_points[kept[k]] for k in (j-1,j,j+1))
                    if (j+1 < len(kept)-1 and math.hypot(a.x-c.x,a.y-c.y)<1e-9
                            and not b.is_stop_point and b.block not in mines|bridges
                            and not c.is_stop_point and not b.reverse and not c.reverse):
                        del kept[j:j+2]; changed = True; break
        self._points = tuple(original_points[i] for i in kept)
        # The reference point is not the front bumper. A turn ending exactly one
        # cell before a door can still sweep the nose into its wall opening. Put
        # the zero-curvature entry/exit poses beyond the full body-corner reach;
        # the original neighbouring centres remain on the protected straight.
        protected_run = (geo.BLOCK_SIZE/2 + math.hypot(
            float(cfg.get("vehicle.length_m"))/2 + abs(float(cfg.get("vehicle.ref_offset_m",0.0))),
            float(cfg.get("vehicle.width_m"))/2) + 0.005)
        shifted = list(self._points)
        source_s_adjustments = {}
        for i,w in enumerate(self._points):
            if w.block not in bridges or i == 0 or i == len(self._points)-1:
                continue
            a,b=self._points[i-1],self._points[i+1]
            incoming=(w.x-a.x,w.y-a.y);outgoing=(b.x-w.x,b.y-w.y)
            before,after=math.hypot(*incoming),math.hypot(*outgoing)
            if before<1e-8 or after<1e-8:
                continue
            ux,uy=incoming[0]/before,incoming[1]/before
            if abs(ux*outgoing[1]-uy*outgoing[0])>1e-7 or ux*outgoing[0]+uy*outgoing[1]<=0:
                raise TrajectoryInfeasible("bridge approach/departure are not collinear",waypoint=kept[i])
            before_run=max(protected_run,before+abs(ref_offset)+.005 if a.block in mines else 0.)
            after_run=max(protected_run,after+abs(ref_offset)+.005 if b.block in mines else 0.)
            if before<before_run and i-1>0:
                delta=before_run-before
                shifted[i-1]=replace(a,x=a.x-ux*delta,y=a.y-uy*delta)
                source_s_adjustments[i-1]=delta
            if after<after_run and i+1<len(self._points)-1:
                delta=after_run-after
                shifted[i+1]=replace(b,x=b.x+ux*delta,y=b.y+uy*delta)
                source_s_adjustments[i+1]=-delta
        self._points=tuple(shifted)
        n = len(self._points)
        headings = [math.atan2(b.y-a.y, b.x-a.x) for a,b in zip(self._points,self._points[1:])]
        lengths = [math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(self._points,self._points[1:])]
        if any(v < 1e-8 for v in lengths):
            raise TrajectoryInfeasible("zero-length route segments must be merged by RouteBuilder")
        directions = [-1 if self._points[1].reverse else 1]
        inferred_reverse = False
        cusps = set()
        for i in range(1,n-1):
            explicit = bool(self._points[i+1].reverse)
            was_explicit = bool(self._points[i].reverse)
            if explicit != was_explicit:
                inferred_reverse = False
            turn = abs(geo.normalize_angle(headings[i]-headings[i-1]))
            if turn > math.pi-.02 and not explicit and not was_explicit:
                inferred_reverse = not inferred_reverse
            d = -1 if explicit or inferred_reverse else 1
            if d != directions[-1]: cusps.add(i)
            directions.append(d)
        mandatory = {0,n-1} | cusps | {i for i,w in enumerate(self._points) if w.is_stop_point or w.block in mines}
        bridge_straights = set()
        bridge_headings = {}
        for i,w in enumerate(self._points):
            if w.block in bridges:
                if i==0 or i==n-1:
                    raise TrajectoryInfeasible("bridge needs an approach and departure waypoint",waypoint=i)
                if abs(geo.normalize_angle(headings[i]-headings[i-1])) > 1e-6:
                    raise TrajectoryInfeasible("bridge approach/departure are not collinear",waypoint=i)
                if directions[i] != directions[i-1]:
                    raise TrajectoryInfeasible("direction change inside a bridge is forbidden",waypoint=i)
                mandatory.update((i-1,i,i+1));bridge_straights.update((i-1,i,i+1))
                for j in (i-1,i,i+1):
                    bridge_headings[j] = geo.normalize_angle(headings[i-1]+(math.pi if directions[i]<0 else 0))
        # Without map labels every input point is an obligation: dropping one silently
        # could miss a mine unknown to this compiler.
        if field is None:
            mandatory.update(range(n))
        anchors=sorted(mandatory)
        body_yaws=[]
        for i in range(n):
            if i==0: h=headings[0];d=directions[0]
            elif i==n-1:
                body_yaws.append(geo.normalize_angle(self._points[-1].yaw))
                continue
            elif i in cusps: h=headings[i-1];d=directions[i-1]
            elif i in bridge_straights:
                body_yaws.append(bridge_headings[i]);continue
            else:
                h=headings[i-1]+geo.normalize_angle(headings[i]-headings[i-1])/2
                d=directions[i]
            body_yaws.append(geo.normalize_angle(h+(math.pi if d<0 else 0)))
        self._pieces=[];self._piece_start=[];self._anchor_s={0:0.0};self.search_expansions=0
        total=0.0
        self.initial_pose = (start_pose if start_pose is not None else geo.Pose(
            self._points[0].x,self._points[0].y,
            body_yaws[0] if start_yaw is None else geo.normalize_angle(start_yaw)))
        current_x,current_y,current_yaw=self.initial_pose.x,self.initial_pose.y,self.initial_pose.yaw
        for a,b in zip(anchors,anchors[1:]):
            start=(current_x,current_y,current_yaw)
            goal=(self._points[b].x,self._points[b].y,body_yaws[b])
            d=directions[a]
            goal_yaws=[goal[2]]
            if b not in bridge_straights and b != n-1:
                # A mine or stop constrains position, not car heading. Searching
                # these shared, map-independent tangent candidates avoids inventing
                # a forbidden diagonal pose at an arena edge.
                for h in (headings[b-1],headings[min(b,n-2)]):
                    for sign in (d,-d):
                        candidate=geo.normalize_angle(h+(math.pi if sign<0 else 0))
                        if all(abs(geo.normalize_angle(candidate-v))>1e-8 for v in goal_yaws):
                            goal_yaws.append(candidate)
            mine_goal=self._points[b].block in mines and b not in bridge_straights
            goal_poses=[(goal[0]-ref_offset*math.cos(h) if mine_goal else goal[0],
                         goal[1]-ref_offset*math.sin(h) if mine_goal else goal[1],h) for h in goal_yaws]
            if pose_valid is not None:
                goal_poses=[p for p in goal_poses if pose_valid(*p)]
                if not pose_valid(*start) or not goal_poses:
                    raise TrajectoryInfeasible(f"mandatory endpoint footprint has no candidate tangent at waypoint {kept[a]} or {kept[b]}",waypoint=kept[b])
            try:
                pieces,count=_connect(start,goal,d,self.max_curvature,pose_valid,
                                      [(w.x,w.y) for w in self._points[a:b+1]],max_nodes,goal_yaws,
                                      bool(self._points[a+1].reverse),goal_poses)
            except TrajectoryInfeasible as e:
                raise TrajectoryInfeasible(f"waypoints {kept[a]}->{kept[b]}: {e}",waypoint=kept[b]) from e
            self.search_expansions+=count
            for piece in pieces:
                self._piece_start.append(total);self._pieces.append(piece);total+=piece.length
            current_x,current_y,end_yaw,_=pieces[-1].at(pieces[-1].length)
            current_yaw=geo.normalize_angle(end_yaw+(math.pi if pieces[-1].reverse else 0))
            self._anchor_s[b]=total
        self._total=total
        # Dense geometric index reuses the one authoritative projection implementation.
        dense=[];sample_s=[]
        for start,piece in zip(self._piece_start,self._pieces):
            count=max(1,math.ceil(piece.length/.008))
            for j in range(count):
                ss=start+piece.length*j/count
                x,y,yaw,_=piece.at(piece.length*j/count)
                dense.append(Waypoint(block="",x=x,y=y,yaw=yaw,reverse=piece.reverse));sample_s.append(ss)
        x,y,yaw,_=self._pieces[-1].at(self._pieces[-1].length)
        dense.append(Waypoint(block="",x=x,y=y,yaw=yaw,reverse=self._pieces[-1].reverse));sample_s.append(total)
        self._sampled_s=tuple(sample_s);self._index=Path(Route(tuple(dense),name=route.name+"/trajectory-index"))
        self._index_s=tuple(self._index.s_at_waypoint(i) for i in range(len(dense)))
        waypoint_s=[]
        for a,b in zip(anchors,anchors[1:]):
            lo,hi=self._anchor_s[a],self._anchor_s[b]
            for i in range(a,b):
                if i==a: waypoint_s.append(lo)
                else:
                    # Ordered nearest samples; ordinary centres are guide points, not
                    # geometrical obligations. Keep mapping monotone for mission indices.
                    first=bisect.bisect_left(sample_s,waypoint_s[-1]);last=bisect.bisect_right(sample_s,hi)
                    w=self._points[i]
                    best=min(range(first,last),key=lambda j:(dense[j].x-w.x)**2+(dense[j].y-w.y)**2)
                    waypoint_s.append(sample_s[best])
        waypoint_s.append(total)
        # Bridge anchors protect the whole body crossing. Mine events on that
        # straight occur when the body centre reaches the original grid centre,
        # so map their source s without moving the protected entry/exit poses.
        for i in bridge_straights:
            if self._points[i].block in mines:
                source_s_adjustments[i]=source_s_adjustments.get(i,0.)-ref_offset*directions[min(i,n-2)]
        # Restore the original source index namespace. Removed ordinary points
        # map monotonically onto the actual connecting curve, never to fake poses.
        cleaned_s=tuple(waypoint_s)
        remapped=[]
        for a,b in zip(range(len(kept)-1),range(1,len(kept))):
            ia,ib=kept[a],kept[b]
            lo=cleaned_s[a]+source_s_adjustments.get(a,0.0)
            hi=cleaned_s[b]+source_s_adjustments.get(b,0.0)
            for i in range(ia,ib):
                if i==ia:remapped.append(lo)
                else:
                    first=bisect.bisect_left(sample_s,remapped[-1]);last=bisect.bisect_right(sample_s,hi)
                    w=original_points[i]
                    best=min(range(first,last),key=lambda j:(dense[j].x-w.x)**2+(dense[j].y-w.y)**2)
                    remapped.append(sample_s[best])
        remapped.append(total)
        self._points=original_points
        n=len(original_points)
        self._waypoint_s=tuple(remapped)
        self._s_start=self._waypoint_s[:-1]
        self._segments=tuple((self.point_at(self._waypoint_s[i])[0],self.point_at(self._waypoint_s[i])[1],
                              self.point_at(self._waypoint_s[i+1])[0],self.point_at(self._waypoint_s[i+1])[1],
                              self._waypoint_s[i+1]-self._waypoint_s[i],self.heading_at(self._waypoint_s[i]))
                             for i in range(n-1))
        self._stop_indices=tuple(sorted({i for i,w in enumerate(self._points) if w.is_stop_point}|{n-1}))
        self.cusp_s=tuple(self._piece_start[i] for i in range(1,len(self._pieces))
                          if self._pieces[i].reverse != self._pieces[i-1].reverse)
        self.stop_s=tuple(sorted({self.s_at_waypoint(i) for i in self._stop_indices}|set(self.cusp_s)|{0.0}))
        self.mandatory_indices=tuple(kept[i] for i in anchors)
        self.removed_guide_indices=tuple(i for i in range(n) if i not in kept)

    def _piece_at(self,s):
        s=min(self._total,max(0.,float(s)))
        i=min(len(self._pieces)-1,max(0,bisect.bisect_right(self._piece_start,s)-1))
        return s,i,self._pieces[i],s-self._piece_start[i]

    def sample(self,s):
        s,i,piece,u=self._piece_at(s);x,y,yaw,k,k_s=piece._state_at(u)
        seg=min(len(self._points)-2,max(0,bisect.bisect_right(self._waypoint_s,s)-1))
        cruise=float(self._cfg.get("pursuit.cruise_speed_mps"))
        scale=min(self._points[seg].speed_scale,self._points[seg+1].speed_scale)
        cap=cruise*max(0.,scale)
        if abs(k)>1e-12:cap=min(cap,math.sqrt(float(self._cfg.get("pursuit.lat_acc_max_mps2"))/abs(k)))
        if abs(k_s)>1e-12:
            wheelbase=float(self._cfg.get("pursuit.wheelbase_m"))
            rate=math.radians(float(self._cfg.get("pursuit.max_steer_rate_deg_s")))
            cap=min(cap,rate*(1+(wheelbase*k)**2)/(wheelbase*abs(k_s)))
        return TrajectorySample(x,y,yaw,k,cap,piece.reverse,s,seg,k_s)

    def point_at(self,s):
        q=self.sample(s);return q.x,q.y,q.seg_index

    def heading_at(self,s):return self.sample(s).yaw
    def curvature_at(self,s):return self.sample(s).curvature
    def s_at_waypoint(self,index):return self._waypoint_s[min(len(self._waypoint_s)-1,max(0,index))]
    def segment_is_reverse(self,index):
        if not 0<=index<len(self._points)-1:return False
        return self.sample((self._waypoint_s[index]+self._waypoint_s[index+1])/2).reverse
    def segment_speed_scale(self,index):
        if not 0<=index<len(self._points)-1:return 1.
        return min(self._points[index].speed_scale,self._points[index+1].speed_scale)
    def corner_angle_deg(self,index):return 0.0
    def corner_turn_deg(self,index):return 0.0
    @property
    def sampled_s(self):return self._sampled_s
    @property
    def sampled_curvature(self):return tuple(self.sample(s).curvature for s in self._sampled_s)
    @property
    def speed_caps(self):return tuple(self.sample(s).v_max for s in self._sampled_s)
    def samples(self):return tuple(self.sample(s) for s in self._sampled_s)

    def _map_arc(self,s,source,dest):
        s=min(source[-1],max(0.,s));i=min(len(source)-2,max(0,bisect.bisect_right(source,s)-1))
        span=source[i+1]-source[i]
        return dest[i]+(dest[i+1]-dest[i])*(s-source[i])/span if span>1e-12 else dest[i]

    @lru_cache(maxsize=32)
    def _projection_index(self, upper):
        """Exclude future branches before nearest-segment search, including partial edges."""
        if upper >= self._total:
            return self._index,self._sampled_s,self._index_s
        count=bisect.bisect_right(self._sampled_s,upper)
        points=list(self._index.waypoints[:count]);source=list(self._sampled_s[:count])
        if source[-1] < upper:
            q=self.sample(upper)
            points.append(Waypoint(block="",x=q.x,y=q.y,yaw=q.yaw,reverse=q.reverse));source.append(upper)
        index=Path(Route(tuple(points),name=self.route.name+"/bounded-projection"))
        return index,tuple(source),tuple(index.s_at_waypoint(i) for i in range(len(points)))

    def project(self,x,y,*,s_hint=None,window_m=.60,allow_backward=True,motion_heading=None,
                back_m=None,s_min=None,s_max=None):
        upper=self._total if s_max is None else min(self._total,max(0.,float(s_max)))
        if s_min is not None and s_min > upper+1e-12:
            raise ValueError("projection s_min exceeds s_max")
        if upper <= 1e-12:
            q=self.sample(0.)
            return Projection(0,0.,q.x,q.y,0.,math.hypot(x-q.x,y-q.y),
                              self._waypoint_s[1],q.yaw,False)
        index,source,dest=self._projection_index(upper)
        hint=None if s_hint is None else self._map_arc(s_hint,source,dest)
        floor=None if s_min is None else self._map_arc(s_min,source,dest)
        p=index.project(x,y,s_hint=hint,window_m=window_m,allow_backward=allow_backward,
                              motion_heading=motion_heading,back_m=back_m,s_min=floor)
        s=self._map_arc(p.s,dest,source)
        if s_hint is not None and not allow_backward:s=max(s,min(s_hint,upper))
        if s_min is not None:s=max(s,min(s_min,upper))
        q=self.sample(s);lo=self._waypoint_s[q.seg_index];hi=self._waypoint_s[q.seg_index+1]
        length=hi-lo;t=(s-lo)/length if length>1e-12 else 0.
        return Projection(q.seg_index,t,q.x,q.y,s,math.hypot(x-q.x,y-q.y),length,q.yaw,
                          p.passed_end and upper >= self._total)
