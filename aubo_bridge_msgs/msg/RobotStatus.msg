# Overall robot status
uint8 DISCONNECTED = 0
uint8 BOOTING = 1
uint8 IDLE = 2
uint8 RUNNING = 3
uint8 ERROR = 4
uint8 EMERGENCY_STOP = 5

uint8 robot_state
bool power_on
bool brakes_released
float64 tool_position_x
float64 tool_position_y
float64 tool_position_z
float64 tcp_load_mass  # kg
float64 tcp_load_cog_x  # center of gravity
