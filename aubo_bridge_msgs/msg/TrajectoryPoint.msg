# Joint positions (radians) for 6 joints
float64[6] joint_positions
# Joint velocities (rad/s)
float64[6] joint_velocities
# Cartesian pose (x, y, z, qx, qy, qz, qw)
float64 pose_position_x
float64 pose_position_y
float64 pose_position_z
float64 pose_orientation_x
float64 pose_orientation_y
float64 pose_orientation_z
float64 pose_orientation_w
# Blend radius for smoothing (0 = sharp corner)
float64 blend_radius
# Waypoint label
string label
