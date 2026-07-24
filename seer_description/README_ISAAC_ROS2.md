# Isaac Sim + ROS 2 control

This package can launch `seer_aubo.usd`, connect its articulation to ROS 2,
and use MoveIt 2 trajectories to control the six AUBO joints.

## Data flow

`MoveIt 2` → `FollowJointTrajectory` → `action_bridge.py` →
`/isaac_joint_commands` → `IsaacArticulationController`

Isaac Sim publishes the simulated joint state on `/joint_states` and simulation
time on `/clock`. The differential-drive base accepts `/cmd_vel`, publishes
`/odom`, and broadcasts the `odom` → `base_footprint` transform.

## One-command startup

Build once from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to \
  seer_description seer_aubo_moveit_config seer_aubo_stick_moveit_config
```

Close any manually opened Isaac Sim window, then run:

```bash
source install/setup.bash
ros2 run seer_description start_isaac_ros2_stack.sh
```

The default is headless. To show Isaac Sim:

```bash
ros2 run seer_description start_isaac_ros2_stack.sh --gui
```

Useful options:

```text
--no-rviz
--domain-id ID
--usd /absolute/path/to/seer_aubo.usd
--isaac-sim /absolute/path/to/isaacsim
```

## Warehouse demonstration

The package includes `urdf/warehouse_demo.usda`, a composed warehouse scene
that keeps `seer_aubo.usd` as a separate sublayer. It adds a 24 m × 18 m
building, eight stocked racks, loading pallets, a workcell, a conveyor, a
forklift obstacle, a charging station, safety markings, lights, an overview
camera, and collision geometry.

Launch the complete warehouse with the same ROS 2 control interfaces:

```bash
ros2 run seer_description start_warehouse_demo.sh --no-rviz
```

Use `--gui` instead of `--no-rviz` when a graphical session is available.
Regenerate the environment after editing the layout constants with:

```bash
ros2 run seer_description generate_warehouse_scene.py \
  --output /path/to/warehouse_demo.usda
```

## Stick-gripper robot

`urdf/seer_aubo_stick.usda` is generated from the Gazebo robot description
`composite_robot_stick.urdf.xacro`. It retains the adapter, motor housing,
finger meshes, inertial properties, collision geometry, and both 0–40 mm
prismatic finger joints. `urdf/warehouse_stick_demo.usda` composes that robot
with the warehouse.

Launch the warehouse and the matching stick-gripper MoveIt configuration:

```bash
ros2 run seer_description start_warehouse_stick_demo.sh
```

Open or close the gripper through the Isaac trajectory bridge:

```bash
# Open
ros2 run seer_description mobile_manipulator_control.py \
  --action-name /aubo_arm_controller/follow_joint_trajectory \
  gripper --position 0.0

# Close
ros2 run seer_description mobile_manipulator_control.py \
  --action-name /aubo_arm_controller/follow_joint_trajectory \
  gripper --position 0.04
```

Regenerate the warehouse composition with another robot layer using
`generate_warehouse_scene.py --robot-layer FILE`.

## ROS 2 interfaces

- Command topic: `/isaac_joint_commands` (`sensor_msgs/msg/JointState`)
- Simulated state: `/joint_states` (`sensor_msgs/msg/JointState`)
- Mobile-base command: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Mobile-base odometry: `/odom` (`nav_msgs/msg/Odometry`)
- Simulation clock: `/clock`
- RGB image: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- Depth image: `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)
- Camera calibration: `/camera/color/camera_info`
- Front lidar scan: `/front_lidar/scan` (`sensor_msgs/msg/LaserScan`)
- Rear lidar scan: `/back_lidar/scan` (`sensor_msgs/msg/LaserScan`)
- Front lidar points: `/front_lidar/points` (`sensor_msgs/msg/PointCloud2`)
- Rear lidar points: `/back_lidar/points` (`sensor_msgs/msg/PointCloud2`)
- MoveIt controller action:
  `/aubo_arm_controller_wo_gripper/follow_joint_trajectory`

The runner creates Isaac Sim 6.x-native camera and RTX lidar sensors on the
existing robot mounting frames at runtime. Camera data is published at 640×480
with a nominal 20 Hz update rate; the two 2D lidars publish scans and point
clouds at a nominal 10 Hz. Wall-clock rates track the simulation's real-time
factor. The source USD remains unchanged.

## Base and arm motion commands

Drive the base forward for two seconds:

```bash
ros2 run seer_description mobile_manipulator_control.py \
  base --linear 0.2 --duration 2.0
```

Move the six AUBO joints with a smooth four-second trajectory:

```bash
ros2 run seer_description mobile_manipulator_control.py \
  arm --target 0.0 -0.35 0.6 0.0 0.35 0.0 --duration 4.0
```

Run a base-then-arm demonstration:

```bash
ros2 run seer_description mobile_manipulator_control.py sequence
```

The controller limits base commands to 0.5 m/s linear and 1.0 rad/s angular,
reads the current arm pose from `/joint_states`, interpolates each arm motion,
and always publishes zero base velocity when it finishes or is interrupted.

The command message should use these joint names:

```text
shoulder_joint
upperArm_joint
foreArm_joint
wrist1_joint
wrist2_joint
wrist3_joint
gripper1_joint
gripper2_joint
```

## Notes

- The automation targets the current Isaac Sim 6.x installation at
  `~/isaacsim` and ROS 2 Jazzy. Override it with `--isaac-sim` when needed.
- Do not start a second Isaac Sim instance while the GUI is already running.
- `seer_aubo.usd` contains two stale references to Isaac 5.1 SICK lidar assets.
  The runner removes those references from a temporary runtime copy so startup
  cannot block. It also replaces the USD's legacy ActionGraph with a clean
  Isaac Sim 6.x ROS 2 graph. These runtime changes never modify the source USD.
  Unresolved caster visual warnings may remain and should be repaired
  separately.
