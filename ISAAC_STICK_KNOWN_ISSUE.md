# Isaac Sim stick model: known USD issue

Status: paused on 2026-07-24. The stick USD is experimental and is not yet
recommended as the default simulation model.

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
- The remaining likely cause is an Isaac Sim 6.1 PhysX constraint/frame issue
  when new nested rigid bodies and joints are added to the older flat
  articulation.

## Current experimental implementation

- `seer_description/urdf/seer_aubo_stick.usda` references the stable
  `seer_aubo.usd` and overlays the gripper geometry.
- The current paused experiment keeps the adapter and motor fixed bodies while
  the finger geometry is visual-only. It still requires a final dynamics test
  before use.
- `start_warehouse_stick_demo.sh` targets the original articulation root:
  `/World/seer_aubo_composite/base_footprint`.

## Recommended next investigation

1. Test a visual-only gripper overlay to confirm that removing all new rigid
   constraints preserves the level chassis.
2. Reintroduce one constraint at a time and inspect its two world-space joint
   frames before starting PhysX.
3. If Isaac Sim 6.1 continues to react to the nested constraints, keep the
   stable articulation and drive the two finger visual transforms
   kinematically from a ROS 2 gripper command topic.
4. Replace only the failed `link2.DAE` Isaac visual with the existing
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
