# External position reference sent to px4_control_node in PX4 NED.
# active=true: all position and feed-forward fields are required and finite.
# active=false: stop pursuit immediately and hold the current vehicle position;
#               all remaining fields are ignored by px4_control_node.
# Using the same topic for target and stop messages preserves their order.

bool active
float32 x
float32 y
float32 z
float32 vx
float32 vy
float32 vz
float32 ax
float32 ay
float32 az
float32 yaw
float32 yawspeed
