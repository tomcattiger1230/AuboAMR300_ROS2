# Isaac Sim stick model: known USD issue

Status: wheel penetration and chassis roll fixed on 2026-07-24. The remaining
stick work concerns gripper dynamics and ROS 2 distribution compatibility.

## Wheel penetration fix

The original analytic wheel cylinders did not generate effective ground
support in PhysX. The stable-looking base model was balanced only by the front
and rear caster spheres; adding the gripper created a small lateral moment and
the chassis rolled around the line between those casters.

Measured before the fix, the left wheel center fell from Z=0.1 m to
Z=0.01426 m, putting its nominal lower surface at Z=-0.08574 m. The chassis
settled at approximately 16.5 degrees of roll.

`seer_aubo_stick.usda` now adds a non-instanceable sphere collision directly
under each drive-wheel rigid body. It also binds:

- static/dynamic friction 1.0 to the drive-wheel support collisions;
- static/dynamic friction 0.0 to the front and rear casters, matching the
  Gazebo URDF settings.

After 300 PhysX steps, all four supports remain at Z=0 within 0.2 micrometers
and the chassis orientation remains the identity quaternion. A separate drive
test moved the robot approximately 1.17 m in 300 steps while it remained level.

## ROS 2 environment on the Lyrical host

The host at `192.168.3.23` has ROS 2 Lyrical with Python 3.14, while Isaac Sim
uses Python 3.12. A workspace-level uv environment at `.venv-isaac` supplies
`empy==3.3.4` and `lark` to Isaac Python. With that environment on
`PYTHONPATH`, the Isaac ROS 2 bridge and camera graph reach the ready state.

Isaac still warns that the system Lyrical `rclpy` extension was compiled for a
different Python ABI, but the bridge's internal C backend continues to start.
The remaining host dependency is the ROS `xacro` executable:
`ros-lyrical-xacro` is available from the configured apt repository but is not
installed.

## Confirmed observations

- The arm and gripper display correctly in RViz. This indicates that the URDF,
  meshes, robot state publisher, and the MoveIt-side link/joint chain are not
  the source of the missing-link rendering problem.
- Importing the complete stick URDF with the Isaac Sim 6.1 URDF importer drops
  the visual instance for the arm `upperArm_Link` (`link2.DAE`). Its collision
  mesh is present, so the middle arm section appears missing only in Isaac Sim.
- The newly imported robot settles with approximately 16.5 degrees of roll:
  `/odom` reports orientation `x ~= -0.14338`, `w ~= 0.98967`.
- The original `seer_aubo.usd` in the same warehouse scene remains level:
  `/odom` reports the identity orientation and near-zero position.
- Reusing the original stable chassis/arm USD and adding only the gripper
  rigid bodies still reproduces the same roll.
- Reducing the added gripper mass to 1 gram per link and removing all added
  gripper collision shapes does not change the roll. This rules out gripper
  weight and contact geometry as the cause.
- The ineffective wheel-cylinder contacts were the direct cause of the chassis
  roll and wheel penetration. Nested gripper constraints supplied the lateral
  moment that exposed the already-missing wheel support.

## Current experimental implementation

- `seer_description/urdf/seer_aubo_stick.usda` references the stable
  `seer_aubo.usd` and overlays the gripper geometry.
- The current gripper experiment keeps the adapter and motor fixed bodies while
  the finger geometry is visual-only.
- `start_warehouse_stick_demo.sh` targets the original articulation root:
  `/World/seer_aubo_composite/base_footprint`.

## Recommended next investigation

1. Reintroduce one gripper constraint at a time and inspect its two world-space joint
   frames before starting PhysX.
2. If Isaac Sim 6.1 continues to react to the nested constraints, keep the
   stable articulation and drive the two finger visual transforms
   kinematically from a ROS 2 gripper command topic.
3. Replace only the failed `link2.DAE` Isaac visual with the existing
   `collision/link2.STL`, without changing the RViz URDF.

## Reproduction

```bash
cd ~/Develop/ROS_ws/amr300_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run seer_description start_warehouse_stick_demo.sh --no-rviz
```

Compare `/odom` against the stable model:

```bash
ros2 topic echo /odom --once
ros2 run seer_description start_warehouse_demo.sh --no-rviz
```
