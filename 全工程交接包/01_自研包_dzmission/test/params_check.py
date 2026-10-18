#!/usr/bin/env python3
"""极简 YAML 结构校验 + 参数键名核对

不依赖 PyYAML，只支持本项目参数文件用到的子集：
  - 注释 (#)
  - "key:" 开头的分组（缩进决定层级）
  - "key: value" 标量
  - "key: [a, b, c]" 行内列表
"""

import re
import sys

# C++ 中 declare_parameter / get_parameter 实际使用的完整键名
EXPECTED = set("""
mission.enable_autonomy
mission.require_motion_enable
mission.arrive_dist
mission.arrive_yaw_tol_deg
mission.goal_timeout_s
mission.goal_retry_period_s
mission.low_battery_pct
mission.link_lost_return_delay_s
mission.auto_return_on_link_loss
mission.engage_requires_localization
mission.engage_approach_dist
mission.gimbal_lock_chassis
mission.clear_recovery_delay_s
mission.dztrack_active_timeout_s
mission.use_dztrack_when_active
mission.default_ammo_when_link_lost
cmd.transport
cmd.serial_port
cmd.serial_baud
cmd.udp_bind_host
cmd.udp_bind_port
cmd.udp_peer_host
cmd.udp_peer_port
cmd.loopback_hz
cmd.link_timeout_ms
loc.imu_topic
loc.odom_topic
loc.amcl_topic
loc.anchor_topic
loc.imu_timeout_ms
loc.odom_timeout_ms
loc.zupt_linear_thresh
loc.zupt_gyro_thresh
loc.zupt_min_duration_ms
loc.wheel_sigma_v
loc.gyro_sigma_z
loc.enable_absolute_fix
loc.fix_sigma_pos
loc.fix_sigma_yaw
loc.fix_min_interval_ms
loc.fix_max_jump_m
loc.publish_tf
loc.publish_map_to_odom
loc.odom_frame
loc.base_frame
loc.map_frame
loc.dr_min_step_m
loc.dr_max_step_m
loc.initial_map_yaw_deg
loc.initial_col
loc.initial_row
loc.anchor_at_startup
loc.block_size_m
loc.anchor_prefix
arbiter.max_v
arbiter.max_servo_deg
arbiter.max_steer_deg
arbiter.max_w
arbiter.max_acc_v
arbiter.max_acc_servo
arbiter.max_acc_w
arbiter.source_ttl
arbiter.publish_cmd_vel
arbiter.publish_pursuit
arbiter.brake_publish_both
pivot.lock_deg
pivot.wait_servo_s
pivot.creep_v
pivot.tol_deg
pivot.lead_deg
pivot.creep_max_cm
pivot.safety_scale
pivot.total_timeout_s
pivot.nominal_radius_cm
pivot.steer_trim_deg
safety.odom_topic
safety.imu_topic
safety.vision_topic
safety.cmd_topic
safety.stall_cmd_v
safety.stall_actual_v
safety.odom_speed_scale
safety.stall_hold_ms
safety.stall_grace_ms
safety.recover_after_s
safety.recover_reverse_v
safety.recover_steer_deg
safety.tilt_pitch_warn_deg
safety.tilt_pitch_stop_deg
safety.tilt_roll_warn_deg
safety.tilt_roll_stop_deg
safety.tilt_hold_ms
safety.vision_near_box_ratio
safety.vision_timeout_ms
safety.vision_crawl_speed
safety.vision_frame_height
safety.odom_timeout_ms
safety.no_odom_crawl_speed
safety.no_odom_stop_ms
safety.map_crawl_speed
safety.map_stale_blocks
safety.eval_hz
safety.crawl_speed
safety.clear_hold_ms
safety.stuck_timeout_s
safety.publish_markers
aim.own_gimbal
aim.vision_topic
aim.gimbal_feedback_topic
aim.gimbal_cmd_topic
aim.laser_topic
aim.gimbal_center_tick
aim.gimbal_tick_min
aim.gimbal_tick_max
aim.gimbal_max_speed
aim.yaw_ticks_per_deg
aim.pitch_ticks_per_deg
aim.yaw_deg_sign
aim.pitch_deg_sign
aim.scan_yaw_dirs
aim.scan_yaw_min_deg
aim.scan_yaw_max_deg
aim.scan_pitch_levels_deg
aim.scan_dwell_ms
aim.scan_arrive_tol
aim.scan_speed
aim.pitch_scan_speed
aim.scan_mode
aim.scan_sweep_rate_dps
aim.scan_hold_ms
aim.scan_sweep_multi_pitch
aim.lost_grace_ms
aim.fire_hold_px
aim.require_blue_face
aim.face_unknown_policy
aim.face_debounce_frames
aim.require_laser_dot
aim.laser_dot_required_after_first
aim.laser_dot_debounce_frames
aim.target_hp_destroyed
aim.ammo_link_lost_policy
aim.ammo_link_lost_allowance
aim.max_local_shots
aim.require_turn_around
aim.turn_around_v_thresh_px_s
aim.turn_around_dwell_ms
aim.turn_around_velocity_alpha
aim.turn_around_v_soft_ratio
aim.turn_around_window_ms
aim.turn_around_vel_history
aim.lead_enable
aim.lead_exposure_ms
aim.lead_servo_ms
aim.lead_fire_ms
aim.lead_pipeline_frames
aim.detect_rate_hz
aim.lead_min_confidence
aim.lead_max_misses
aim.lead_aim_tolerance_px
aim.lead_trust_center_px
aim.engage_standoff_cells
aim.enable_auto_scan
aim.vision_smooth_alpha
aim.fast_alpha
aim.fast_error_threshold_px
aim.vision_step_gain
aim.fast_step_gain
aim.max_step_per_frame
aim.fast_max_step
aim.vision_deadband_px
aim.pitch_deadband_px
aim.pitch_step_gain
aim.pitch_max_step_per_frame
aim.vision_timeout_ms
aim.lost_frames_to_rescan
aim.motor_kp
aim.fast_motor_kp
aim.pitch_motor_kp
aim.pitch_max_speed
aim.position_deadband_ticks
aim.pitch_position_deadband_ticks
aim.fire_tolerance_px
aim.fire_debounce_frames
aim.fire_cooldown_ms
aim.fire_duration_ms
aim.require_stationary
aim.min_ammo_to_fire
aim.max_engage_range_m
referee.direct_serial
referee.serial_port
referee.serial_baud
referee.link_timeout_ms
referee.hit_hold_ms
referee.publish_laser_command
""".split())


def parse(path):
    """返回 [(dotted_key, raw_value, lineno)]"""
    out = []
    stack = []  # [(indent, key)]
    with open(path, encoding='utf-8') as f:
        for ln, raw in enumerate(f, 1):
            line = raw.rstrip('\n')
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            indent = len(line) - len(line.lstrip(' '))
            if '\t' in line[:indent + 1]:
                raise ValueError('line %d: TAB indentation' % ln)
            if indent % 3 != 0:
                print('  warn line %d: 缩进 %d 不是 3 的倍数' % (ln, indent))
            if ':' not in stripped:
                raise ValueError('line %d: 缺少冒号: %r' % (ln, stripped))
            key, _, rest = stripped.partition(':')
            key = key.strip()
            rest = rest.strip()
            while stack and stack[-1][0] >= indent:
                stack.pop()
            # ros__parameters 与节点名通配符不参与键名
            prefix = '.'.join(k for _, k in stack
                              if k not in ('ros__parameters',) and not k.startswith('**')
                              and k != '/**')
            # 移除行内注释（值里不含 # 的场景）
            value = rest
            if value.startswith('['):
                value = value.split(']')[0] + ']'
            elif ' #' in value:
                value = value.split(' #')[0].rstrip()
            if value == '':
                stack.append((indent, key))
                continue
            dotted = (prefix + '.' + key) if prefix else key
            out.append((dotted, value, ln))
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'dzmission/params/dzmission.yaml'
    print('解析 %s' % path)
    try:
        entries = parse(path)
    except ValueError as e:
        print('解析失败: %s' % e)
        return 1

    found = {}
    for k, v, ln in entries:
        if k in found:
            print('  [FAIL] 重复键 %s (line %d 与 %d)' % (k, found[k][1], ln))
            return 1
        found[k] = (v, ln)

    # 只统计分组之下的叶参数
    keys = {k for k in found if '.' in k}
    keys = {k for k in keys if not k.split('.')[0].startswith('**')}

    print('YAML 中共 %d 个叶参数' % len(keys))

    missing = sorted(EXPECTED - keys)
    extra = sorted(keys - EXPECTED)

    ok = True
    if missing:
        ok = False
        print('\n[FAIL] C++ 声明了但 YAML 缺失的键（会退回默认值）：')
        for k in missing:
            print('   - %s' % k)
    if extra:
        print('\n[warn] YAML 里有但 C++ 未声明的键（会被忽略或报错）：')
        for k in extra:
            print('   - %s = %s' % (k, found[k][0]))

    print('\n类型抽查：')
    for k in ('aim.scan_yaw_dirs', 'aim.max_step_per_frame', 'mission.enable_autonomy',
              'arbiter.max_v', 'aim.scan_pitch_levels_deg', 'cmd.transport'):
        if k in found:
            print('  %-32s = %s' % (k, found[k][0]))

    print()
    print('结果: %s' % ('全部键名一致' if not missing else '缺 %d 个键' % len(missing)))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
