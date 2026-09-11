# finger + motor_adapter 夹爪版本

本版本使用 `finger.STL` 和 `motor_adapter.STL` 新建夹爪模型，保留原来的 stick 夹爪及其启动入口。新版本继续使用 AUBO i16H、双激光雷达和 MV-CH100-60UM 黑白相机；相机安装位保持为 `wrist3_Link` 下 `(0, 0.1, 0)` m、绕 Z 旋转 180°。

## 文件与接口

| 用途 | 文件 |
|---|---|
| 夹爪组件 Xacro | [gripper_finger_motor_adapter.urdf.xacro](urdf/gripper_finger_motor_adapter.urdf.xacro) |
| 完整机器人 Xacro | [composite_robot_finger_mono.urdf.xacro](urdf/composite_robot_finger_mono.urdf.xacro) |
| 展开后的 URDF / Gazebo SDF | [URDF](urdf/composite_robot_finger_mono.urdf) / [SDF](urdf/composite_robot_finger_mono.sdf) |
| Isaac 机器人 / 仓库 USD | [机器人](urdf/seer_aubo_finger_mono.usda) / [仓库](urdf/warehouse_finger_mono_demo.usda) |
| MoveIt 配置 | `seer_aubo_finger_mono_moveit_config` |
| 启动脚本 | `start_warehouse_finger_mono_demo.sh` |

上层接口仍是 `gripper1_joint` 和 `gripper2_joint`。`0` 表示打开，`0.04 m` 表示闭合；两个关节沿相反方向各移动 40 mm。GUI、MoveIt 和 Isaac 控制器因此可以继续使用同一套规划、预览、执行及反馈逻辑。macOS GUI 收到远端 `/robot_description` 后会同时切换夹爪和相机外观；工具栏显示“模型：远端 robot_description”表示远端模型已经生效。

## STL 尺寸及当前装配假设

两个源 STL 均以毫米建模，URDF 和 USD 显式转换为米。

| STL | SHA-256 | 原始边界（mm） |
|---|---|---|
| `finger.STL` | `ec65465e9988417de465cf1a15fe1c443b58acd251ca0b81c533a1b63c74a069` | X 0…49，Y 0…47.785927，Z 0…150 |
| `motor_adapter.STL` | `fd08025df2d9639fc9a2677e205003a1cebdb3f95ba4a6f08ee7da5783622060` | X 0.749998…199.25，Y 0.314999…63.314999，Z 0.629999…137.630005 |

当前模型根据网格边界把电机法兰底面放在腕部法兰，使用一份 finger 网格及其镜像组成左右夹指。推算的开口内间距约 102 mm，闭合内间距约 22 mm。质量、惯量及规划碰撞体目前采用保守估计：电机组件为一个 198.5×63×137 mm 盒体，夹指各为一个 49×47.786×150 mm 盒体。收到装配图、关节零位、实测行程与质量后，应再校正安装外参、TCP、惯量及碰撞包络。

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

2026-09-11 已在 Ubuntu 26 / ROS Lyrical 的隔离工作树中构建 `seer_description` 与新 MoveIt 包，`check_urdf` 和 `gz sdf -k` 通过。新 MoveIt 配置能够加载 KDL、OMPL、STOMP 与 Pilz；在 `init_pose` 下，夹爪 `q=0` 和 `q=0.04` 的 `/check_state_validity` 均返回 `valid=true`、无接触。USD 检查确认左右 prismatic joint 范围、镜像轴、新网格引用及 MV-CH100-60UM Camera 均存在；macOS GUI 回归测试 35 项通过（另 1 项跳过）。此次验证没有启动第二套 Isaac，也没有执行机器人运动。
