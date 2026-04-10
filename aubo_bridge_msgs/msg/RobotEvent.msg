# Robot event notifications
uint8 ARM_CANBUS_ERROR = 0
uint8 REMOTE_HALT = 1
uint8 REMOTE_EMERGENCY_STOP = 2
uint8 JOINT_ERROR = 3
uint8 COLLISION = 8
uint8 SOFT_EMERGENCY = 6
uint8 SINGULARITY_OVERSPEED = 16
uint8 CURRENT_ALARM = 17
uint8 ALGORITHM_EXCEPTION = 29
uint8 OVERSPEED = 28
uint8 SOCKET_DISCONNECTED = 27

uint8 event_type
uint8 severity  # 0=INFO, 1=WARN, 2=ERROR
string description
uint64 timestamp_nanosec
