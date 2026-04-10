# Trajectory control command
uint8 EXECUTE_TRAJECTORY = 0
uint8 EXECUTE_JOINT_MOVE = 1
uint8 EXECUTE_LINE_MOVE = 2
uint8 STOP_TRAJECTORY = 3
uint8 PAUSE_TRAJECTORY = 4
uint8 RESUME_TRAJECTORY = 5

uint8 command
TrajectoryPoint[] trajectory
float64 max_joint_velocity    # rad/s per joint
float64 max_joint_acceleration  # rad/s^2 per joint
float64 max_tool_velocity    # m/s
bool blocking
