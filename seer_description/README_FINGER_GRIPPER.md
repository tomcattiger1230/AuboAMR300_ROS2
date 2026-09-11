# finger + motor_adapter 夹爪版本

本版本使用 Blender 调整原点后的 `finger_centered.stl` 和 `motor_new.stl` 新建夹爪模型，保留原始 `finger.STL`、`motor_adapter.STL`、stick 夹爪及其启动入口。新版本继续使用 AUBO i16H、双激光雷达和 MV-CH100-60UM 黑白相机；相机安装位保持为 `wrist3_Link` 下 `(0, 0.1, 0)` m、绕 Z 旋转 180°。

## 文件与接口

| 用途 | 文件 |
|---|---|
| 夹爪组件 Xacro | [gripper_finger_motor_adapter.urdf.xacro](urdf/gripper_finger_motor_adapter.urdf.xacro) |
| 完整机器人 Xacro | [composite_robot_finger_mono.urdf.xacro](urdf/composite_robot_finger_mono.urdf.xacro) |
| 展开后的 URDF / Gazebo SDF | [URDF](urdf/composite_robot_finger_mono.urdf) / [SDF](urdf/composite_robot_finger_mono.sdf) |
| Isaac 机器人 / 仓库 USD | [机器人](urdf/seer_aubo_finger_mono.usda) / [仓库](urdf/warehouse_finger_mono_demo.usda) |
| MoveIt 配置 | `seer_aubo_finger_mono_moveit_config` |
| 启动脚本 | `start_warehouse_finger_mono_demo.sh` |

上层接口仍是 `gripper1_joint` 和 `gripper2_joint`。`0` 表示打开；当前保守闭合位置为 `0.0285 m`，两个关节沿相反方向各移动 28.5 mm。macOS GUI 从远端 `/robot_description` 自动读取这个上限，旧 stick 版本仍使用 40 mm。工具栏显示“模型：远端 robot_description”表示远端模型已经生效。

## STL 尺寸及当前装配假设

两个源 STL 均以毫米建模，URDF 和 USD 显式转换为米。

| STL | SHA-256 | 原始边界（mm） |
|---|---|---|
| `finger_centered.stl` | `314d811a2f3fd666e75003867208321db06ff4f70e9f3cf6e84619c21330c7ac` | X -0.021399…0.027601，Y -0.048413…-0.000627，Z -0.074905…0.075095 m |
| `motor_new.stl` | `b2efdfa39eff33f23b1b3d365a1a1084673f06eaed6bb69aa9150b75475559d4` | X -0.077473…0.121027，Y -0.031393…0.031607，Z -0.135626…0.001374 m |

当前模型按用户在 Blender 中调整的原点与方向装配：电机绕 X 旋转 180°，夹指绕 X 旋转 -90°，左右关节位于 X=±56.4 mm、Z=135 mm。旋转后的电机碰撞盒为 198.5×63×137 mm；夹指碰撞盒为 49×150×47.786 mm。全开时两盒内侧间距约 57.6 mm；每侧移动 28.5 mm 后保留约 0.6 mm 间隙。两份 STL 都不是封闭实体，因此质量仍使用 1.2 kg 与 0.18 kg 的暂估值，质心采用包围盒中心，惯量按对应盒体重新计算。收到实际质量和 CAD 装配基准后仍需校正。

## 构建与启动

Ubuntu 工作区使用项目已有的 Python 环境：

```bash
cd ~/Develop/ROS_ws/hongshi_mm_ws
source .venv/bin/activate
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select \
  seer_description seer_aubo_finger_mono_moveit_config aubo_control_gui
source install/setup.bash
ros2 run seer_description start_warehouse_finger_mono_demo.sh \
  --gui --domain-id 133 --camera-resolution preview
```

运行新版本前先停止旧的 Isaac / MoveIt stack，避免两套节点发布相同的关节、TF 和 action 名称。macOS 端关闭旧 GUI 后重新运行 `./scripts/start_macos_gui.command --execute`；GUI 会按远端 `robot_description` 显示新夹爪。

## 重新生成与验证

```bash
MODEL_DIR="$PWD/src/AuboAMR300_ROS2/seer_description/urdf"
ros2 run seer_description generate_finger_robot_models.py "$MODEL_DIR"
~/isaacsim/.venv/bin/python \
  "$MODEL_DIR/../scripts/generate_finger_gripper_usd.py" \
  "$MODEL_DIR" "$MODEL_DIR/../meshes"
check_urdf "$MODEL_DIR/composite_robot_finger_mono.urdf"
gz sdf -k "$MODEL_DIR/composite_robot_finger_mono.sdf"
```

2026-09-11 已在 Ubuntu 26 / ROS Lyrical 的隔离工作树中构建 `seer_description` 与新 MoveIt 包，`check_urdf` 和 `gz sdf -k` 通过。MoveIt 在 `init_pose` 下验证 `q=0` 和 `q=0.0285` 均为 `valid=true`；旧闭合量 `q=0.04` 会检测到两夹指约 22.4 mm 穿透并返回 `valid=false`。USD/模型测试 11 项及 9 个子测试通过，macOS GUI 测试 35 项通过（另 1 项跳过）。此次检查不执行机器人运动。
