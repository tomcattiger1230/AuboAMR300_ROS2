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
  seer_description seer_aubo_moveit_config
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

## ROS 2 interfaces

- Command topic: `/isaac_joint_commands` (`sensor_msgs/msg/JointState`)
- Simulated state: `/joint_states` (`sensor_msgs/msg/JointState`)
- Mobile-base command: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Mobile-base odometry: `/odom` (`nav_msgs/msg/Odometry`)
- Simulation clock: `/clock`
- MoveIt controller action:
  `/aubo_arm_controller_wo_gripper/follow_joint_trajectory`

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
  `~/isaacsim` and ROS 2 Jazzy. Override it with `--isaac-sim` when needed.
- Do not start a second Isaac Sim instance while the GUI is already running.
- `seer_aubo.usd` contains two stale references to Isaac 5.1 SICK lidar assets.
  The runner removes those references from a temporary runtime copy so startup
  cannot block. It also replaces the USD's legacy ActionGraph with a clean
  Isaac Sim 6.x ROS 2 graph. These runtime changes never modify the source USD.
  Unresolved caster visual warnings may remain and should be repaired
  separately.
