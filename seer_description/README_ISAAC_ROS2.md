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

Make sure the ROS `xacro` command is installed:

```bash
sudo apt install ros-${ROS_DISTRO}-xacro
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

`urdf/seer_aubo_stick.usda` references the stable `seer_aubo.usd` chassis and
arm, then overlays the stick-gripper adapter, motor housing, and finger meshes.
This avoids the Isaac Sim 6.1 URDF importer issue that drops the
`upperArm_Link` visual mesh. `urdf/warehouse_stick_demo.usda` composes that
robot with the warehouse.

Launch the warehouse and the matching stick-gripper MoveIt configuration:

```bash
ros2 run seer_description start_warehouse_stick_demo.sh
```

The USD includes two physical prismatic finger joints (`gripper1_joint` and
`gripper2_joint`), each with 0–40 mm travel, position drives, and collision
geometry. Adapter, motor and fingers are sibling rigid bodies connected by
fixed/prismatic joints; their initial joint frames match the URDF. This avoids
nesting a dynamic rigid body inside another rigid body.

Both finger states are published on `/joint_states`. In RViz select the
`gripper` planning group and `gripper_open` or `gripper_closed`, then use
**Plan & Execute**. The existing eight-joint `aubo_arm_controller` covers the
six arm joints and two fingers. The action bridge waits for fresh, measured
joint positions before reporting success (default finger tolerance: 1 mm).

For a direct trajectory command, without MoveIt planning:

```bash
ros2 run seer_description mobile_manipulator_control.py gripper --position 0.04
ros2 run seer_description mobile_manipulator_control.py gripper --position 0.0
```

Open/close planning and execution have been checked on Ubuntu 26.04 / Lyrical
with Isaac Sim 6.0.1-rc.7 using the internal Jazzy bridge. This does not validate
contact grasping, payload retention, or force control. The `manipulator` SRDF
group still includes the branched gripper, so its KDL Cartesian IK solver
cannot initialize; named **joint-space** goals and the separate gripper group
remain usable.

### Wheel contact fix

The original imported wheel cylinders did not create effective ground support.
The level base was balanced only by its two centerline caster spheres, so the
small lateral moment from the gripper caused approximately 16.5 degrees of roll
and pushed one wheel about 85.7 mm below the floor.

The stick USD now adds a non-instanceable sphere collision directly under each
drive-wheel rigid body. Drive-wheel static/dynamic friction is 1.0 and caster
friction is 0.0, matching the Gazebo URDF settings. In the regression probe:

- all four support points stayed at Z=0 within 0.2 micrometers for 300 steps;
- the chassis orientation remained the identity quaternion;
- a wheel-drive test moved the base approximately 1.17 m without tipping.

Regenerate the warehouse composition with another robot layer using
`generate_warehouse_scene.py --robot-layer FILE`.

## ROS distribution and Python compatibility

Keep each host's ROS and Python environment. For example, the Ubuntu 26.04
host uses Lyrical and a workspace `.venv` with Python 3.14, whereas Isaac Sim
6.0.1 uses its bundled Python 3.12.

`start_isaac_ros2_stack.sh` defaults to `--ros-bridge-mode auto`: on Lyrical it
isolates **only the Isaac process**, using Isaac's internal Jazzy backend and
`RaytracedLighting`. ROS nodes, MoveIt and the workspace continue to use
Lyrical. System `PYTHONPATH` and ROS shared libraries are removed from the
Isaac child process; the parent environment is preserved. On other ROS
releases, auto keeps the original system-library path and renderer.

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
source .venv/bin/activate
ros2 run seer_description start_warehouse_stick_demo.sh --gui
```

Explicit host-specific overrides are available:

```text
--ros-bridge-mode system|internal|auto
--bridge-distro jazzy|humble
--renderer RaytracedLighting|RealTimePathTracing
--isaac-sim /path/to/isaacsim
```

Internal mode requires the selected backend's libraries in the Isaac
installation; it does not install or change the host ROS distro. Cross-distro
communication was checked for this robot's standard messages and actions,
not for arbitrary custom messages. System mode can still use `.venv-isaac`
pure-Python dependencies when needed. `xacro` must be installed for the host
ROS distribution.

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
  `/aubo_arm_controller/follow_joint_trajectory`

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

Run the complete example. It records the initial arm pose, drives the base,
moves the arm to the example target, and returns the arm to its recorded pose:

```bash
ros2 run seer_description mobile_manipulator_control.py demo
```

The controller limits base commands to 0.5 m/s linear and 1.0 rad/s angular,
reads the current arm pose from `/joint_states`, interpolates each arm motion,
and always publishes zero base velocity when it finishes or is interrupted.
Use `sequence` instead of `demo` when the arm should remain at the target pose.

The Isaac/with-gripper controller action is the default. For the older
without-gripper Gazebo controller, place this global option before the command:

```bash
ros2 run seer_description mobile_manipulator_control.py \
  --action-name /aubo_arm_controller_wo_gripper/follow_joint_trajectory \
  demo
```

The command message should use these joint names:

```text
shoulder_joint
upperArm_joint
foreArm_joint
wrist1_joint
wrist2_joint
wrist3_joint
```

## Notes

- The automation targets the current Isaac Sim 6.x installation at
  `~/isaacsim`. It uses `$ROS_DISTRO` when sourced, or detects the installed
  distribution under `/opt/ros`. Override Isaac with `--isaac-sim` when needed.
- Do not start a second Isaac Sim instance while the GUI is already running.
- The host currently reports duplicate NVIDIA Vulkan ICDs. Isaac warns that
  this can cause instability; clean the duplicate driver installation
  separately from the robot-physics fix.
- `seer_aubo.usd` contains two stale references to Isaac 5.1 SICK lidar assets.
  The runner removes those references from a temporary runtime copy so startup
  cannot block. It also replaces the USD's legacy ActionGraph with a clean
  Isaac Sim 6.x ROS 2 graph. These runtime changes never modify the source USD.
  Unresolved caster visual warnings may remain and should be repaired
  separately.
