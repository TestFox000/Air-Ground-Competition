# Control command from teleop node to control node.
#
# cmd: one of the constants below.
# For CMD_MOVE, dx/dy/dz are position increments already in NED frame
# (i.e. "up 10cm" is passed as dz = -0.10).

uint8 CMD_NONE     = 0
uint8 CMD_OFFBOARD = 1   # key '0': switch to Offboard mode
uint8 CMD_ARM      = 2   # key '1': arm and takeoff to configured height
uint8 CMD_DISARM   = 3   # key '2': disarm (rejected if airborne)
uint8 CMD_LAND     = 4   # key '3': land (NAV_LAND if land_control=true, else self-managed descent)
uint8 CMD_MOVE     = 5   # keys WASDQE: incremental position setpoint
uint8 CMD_YAW      = 6   # keys L/R: incremental yaw (dx carries dyaw in radians, dy/dz ignored)
uint8 CMD_GOTO_ENABLE  = 7   # key '4': allow external Goto execution
uint8 CMD_GOTO_DISABLE = 8   # key '5': reject Goto and hold the current pose

uint8 cmd
float32 dx
float32 dy
float32 dz
