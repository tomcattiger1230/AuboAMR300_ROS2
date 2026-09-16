# Isaac Sim + ROS 2 control

This package can launch `seer_aubo.usd`, connect its articulation to ROS 2,
and use MoveIt 2 trajectories to control the six AUBO joints.

## Rebar loading experiment (2026-09-16)

The corrected finger variant now supports a concave pickup station and four chassis-mounted saddles. The bar stays horizontal, turns 90 degrees, follows a staged Cartesian route around the chassis, and is released onto the robot. A live payload-position check cancels execution if the bar is lost. The four original student storage states have also passed collision checks, planning and Isaac execution with the corrected finger tool.

```bash
ros2 run seer_description start_warehouse_finger_rebar_loading_demo.sh --gui --domain-id 133
# Another sourced terminal:
export ROS_DOMAIN_ID=133
ros2 run seer_description test_rebar_grasp.py --onboard-slot 1 --output /tmp/loading.json
```

The rack has been raised by 45 mm: seated bar Z=0.715 m, release TCP Z=0.720 m. V-shaped contact faces retain the curved side walls. Two solid 364.9 × 100 × 20 mm mounting rails now connect the eight pedestals to the actual chassis visual deck (Z≈0.59865 m), eliminating the previous 3.35 mm gap; they are also included in MoveIt. Use `measure_rebar_settling.py` after retraction to distinguish motion inside the seats from chassis drift; earlier four-slot reports used Z=0.670 m. Loading slot 1 with slots 2/3/4 occupied passed 26 checks. After the placement transient settled, all four bars had less than 0.004 mm chassis-relative position span over 30 wall-clock seconds (10.3 simulation seconds). Whole-chassis drift remains separately documented.

See [rebar experiment, results and camera acquisition](README_REBAR_GRASP.md). This launch includes the source station and chassis rack in MoveIt; the full warehouse collision scene is still incomplete.

## Previous tested configuration (2026-09-10)

The current workflow uses Ubuntu 26.04 / ROS Lyrical with Isaac's isolated internal Jazzy bridge,
i16H joint limits, a stick gripper, and an MV-CH100-60UM monochrome camera with a 12 mm C-mount lens.
The generic and RGB-D launch commands below remain available as separate variants; they do not select the monochrome model.
For the current variant, build and launch from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to seer_description seer_aubo_stick_mono_moveit_config
source install/setup.bash
ros2 run seer_description start_warehouse_stick_mono_demo.sh --gui --domain-id 133
```

使用 `finger_centered.stl` 与 `motor_new.stl` 的独立夹爪版本见
[finger + motor_adapter 夹爪说明](README_FINGER_GRIPPER.md)，对应启动命令为
`ros2 run seer_description start_warehouse_finger_mono_demo.sh --gui --domain-id 133`。

The validated camera mount is fixed to `wrist3_Link`, at `(0, 0.1, 0)` m with a 180° Z rotation.
URDF/Xacro, SDF, USD and the USD generator have been restored to this mount.
The Mac GUI follows the running `/robot_description` for camera placement and includes a camera close-up view.
See [monochrome camera and video](README_MONO_CAMERA.md) and [Mac GUI](../aubo_control_gui/README.md).
Real robot deployment remains pending; these checks concern simulation.

## Data flow

`MoveIt 2` → `FollowJointTrajectory` → `action_bridge.py` →
`/isaac_joint_commands` → `IsaacArticulationController`

Isaac Sim publishes the simulated joint state on `/joint_states` and simulation
time on `/clock`. The differential-drive base accepts `/cmd_vel`, publishes
`/odom`, and broadcasts the `odom` → `base_footprint` transform.

## One-command startup

Build once from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash # Use the ROS distribution installed on your host.
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
group retains the branched gripper for joint-space goals. Use the separate
`arm` chain for KDL IK and Cartesian wrist trajectories. See
[SLAM, navigation and wrist video](ISAAC_NAVIGATION_CAMERA.md) for the tested workflow.

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

### Stationary base drift and RViz jitter

The persistent jitter was corrected on 2026-09-14. With no incoming
`/cmd_vel`, the old watchdog still sent 399 all-zero `/isaac_cmd_vel` messages
in 20 seconds. The warmed-up robot moved 6.64 mm in X while the left and right
wheel joints moved 0.0669 and 0.0657 rad. A fresh-start sample was worse:
45.3 mm in X and 0.00814 rad (0.466 degrees) in yaw. The six arm joints and both
gripper positions remained stable, confirming that the visible RViz motion came
from base/wheel physics rather than Fast DDS or commanded arm motion.

Three conditions combined to produce the drift. Each drive wheel had two active,
coincident colliders: the imported cylinder and the added support sphere. The
competing contacts were the remaining physical source of wheel motion. The
watchdog also continuously repeated an unchanged zero target and the base
controller executed on every simulation frame, keeping the PhysX articulation
awake. Finally, the wheel velocity drives used damping 100000 without an explicit
force limit, so small contact errors could generate very large opposing torques.

The imported wheel cylinders are now disabled, leaving one active support-sphere
collider per wheel. The watchdog sends one zero command after its 0.5-second
timeout and then stays quiet until new input arrives. Base control executes only
when the ROS velocity subscriber produces a message. Wheel drive damping is 1000
and maximum force is 50. With both articulation controllers message-driven, the
20-second sample after a turn and forward command reported zero odometry and joint
variation at the published precision.

The arm/gripper position controller must execute each physics frame for its drives
to reach MoveIt's goal tolerance reliably. With base control still message-driven,
the complete arm test succeeds; the following 20-second sample showed 1.91 mm of
X drift and 0.000261 rad (0.015 degrees) of yaw variation. Arm and gripper joints
remained exactly stable. This small residual does not invalidate plans in the Mac
GUI while MoveIt's world contains no external collision geometry, because those
plans are relative to the mobile base. If world obstacles are later imported into
MoveIt, stationary base handling should be revisited before relying on millimetre
clearances.

Both lidars and the monochrome camera continued publishing during the test at
approximately 4.7, 4.7, and 3.0 Hz. Arm joint, Cartesian, and return plans all
executed successfully with maximum joint errors of 0.00150, 0.00166, and
0.00138 rad. See
[the 2026-09-14 regression record](test/results/isaac_stationary_drift_20260914.json).
The Mac GUI still ignores `odom` drift when MoveIt has no external collision
geometry, which remains correct because arm plans are expressed relative to the
mobile base.

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
- RGB-D variant RGB image: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- RGB-D variant depth image: `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)
- RGB-D variant camera calibration: `/camera/color/camera_info`
- Monochrome variant image: `/camera/image_raw` (`sensor_msgs/msg/Image`, `mono8`)
- Monochrome variant calibration: `/camera/camera_info`; no depth topic
- Front lidar scan: `/front_lidar/scan` (`sensor_msgs/msg/LaserScan`)
- Rear lidar scan: `/back_lidar/scan` (`sensor_msgs/msg/LaserScan`)
- Front lidar points: `/front_lidar/points` (`sensor_msgs/msg/PointCloud2`)
- Rear lidar points: `/back_lidar/points` (`sensor_msgs/msg/PointCloud2`)
- MoveIt controller action:
  `/aubo_arm_controller/follow_joint_trajectory`

The runner creates Isaac Sim 6.x-native camera and RTX lidar sensors on the
existing robot mounting frames at runtime. The RGB-D variant publishes 640×480 camera data
with a nominal 20 Hz update rate. The monochrome variant defaults to 1024×615 preview,
with 4096×2460 available via `--camera-resolution full`; see its dedicated README for rate and model limits.
The two 2D lidars publish scans and point
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
- Do not start a second Isaac Sim instance while an Isaac simulation is already running. Starting the Mac control GUI does not start Isaac.
- The host currently reports duplicate NVIDIA Vulkan ICDs. Isaac warns that
  this can cause instability; clean the duplicate driver installation
  separately from the robot-physics fix.
- `seer_aubo.usd` contains two stale references to Isaac 5.1 SICK lidar assets.
  The runner removes those references from a temporary runtime copy so startup
  cannot block. It also replaces the USD's legacy ActionGraph with a clean
  Isaac Sim 6.x ROS 2 graph. These runtime changes never modify the source USD.
  Unresolved caster visual warnings may remain and should be repaired
  separately.
