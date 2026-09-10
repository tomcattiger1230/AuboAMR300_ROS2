# 学生 GUI 接入 Isaac / MoveIt

本轮实现范围为 Isaac Sim。学生电脑端 GUI 已合入本仓库，保留原始代码目录不变。
真实机器人部署按用户要求留到下一轮；`backend:=real` 目前仅选择墙上时钟，不会启动或适配任何实机驱动。

## 接口与行为

GUI → `moveit_msgs/action/MoveGroup` (`/move_action`) → MoveIt →
`control_msgs/action/FollowJointTrajectory` → Isaac articulation。
机械臂使用 `arm` 组，夹爪使用 `gripper` 组，复用现有
`/aubo_arm_controller/follow_joint_trajectory`，不发送学生版 SDK 服务或字符串夹爪命令。

- 从 `/joint_states` 按关节名称读取，不再假定数组前六项就是机械臂。
- 用单调时钟判断反馈是否过期；超过 1.5 秒拒绝新运动，活动请求发起取消。
- 末端位姿通过 `/compute_fk` 获取，坐标系为 `base_footprint`，参考末端为 `wrist3_Link`。
  这是腕部坐标，不是夹爪尖端 TCP；实际工具标定留到实机阶段。
- 关节目标和末端 XYZ 目标都经过 MoveIt 规划。XYZ 保持当前腕部姿态，但运动路径不保证为直线。
- 速度/加速度为模型限值的比例 `(0, 1]`，不是输入 rad/s 或 m/s。
- 快捷位保存六个关节角，再次运行时使用保存后的值。去掉原先固定路径及失败后自动插值重试。
- 停止时取消当前 action；即使请求尚未被服务器接受，稍后接受也立即取消。
  收到终态之前拒绝新运动，关闭窗口也先请求取消并等待。
- Isaac action bridge 取消时用新鲜实测位置发布保持命令及零速度，禁止再发送后续轨迹点。
- 夹爪显示模型单指关节位移 mm；不能等同于真实夹爪串口数值或两指间距。
- GUI 的任务状态只描述本界面发起的请求，不代替控制柜运行/急停状态。

碰撞检查范围仍是当前 MoveIt planning scene。Isaac 仓库场景几何尚未完整导入该场景，
因此测试通过不代表任意仓库姿态或实机路径都已验证。

## 环境与启动

已验证：Ubuntu 26.04 / ROS Lyrical / 系统 Python 3.14 / Qt 6.10.2，
Isaac 使用自己独立的 Python 与 Jazzy bridge。GUI 在宿主 Lyrical 环境运行。
Humble / Jazzy 尚未跑同等验收，勿将本轮结果视为这些发行版已验证。

Ubuntu 26 的 PySide6 按模块拆包，只有基础包不足以显示模型。已在仿真工作站安装：

```bash
sudo apt-get install python3-pyside6.qtwidgets python3-pyside6.qtquickwidgets \
  python3-pyside6.qtquick3d python3-numpy \
  qml6-module-qtquick qml6-module-qtqml-workerscript qml6-module-qtquick3d \
  qml6-module-qtquick3d-assetutils qml6-module-qtquick3d-helpers \
  qt6-quick3d-assetimporters-plugin
```

最后一个插件负责导入 DAE；缺少时界面可能只显示地面。
不要在 Isaac 自带 Python 中运行 GUI；若自建 `.venv` 没有宿主 ROS/Qt 模块，使用系统 Python。

在工作空间根目录构建：

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select aubo_student_description aubo_control_gui
source install/setup.bash
export ROS_DOMAIN_ID=133
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

先启动已有黑白相机 + 夹爪仿真（已启动则不要重复）：

```bash
ros2 run seer_description start_warehouse_stick_mono_demo.sh \
  --gui --domain-id 133 --camera-resolution preview
```

仅规划预览（默认）：

```bash
ros2 launch aubo_control_gui aubo_i16_gui.launch.py backend:=isaac
```

规划并执行仿真运动：

```bash
ros2 launch aubo_control_gui aubo_i16_gui.launch.py backend:=isaac enable_motion:=true
```

GUI 只连接已有 MoveIt，不启动第二套控制器。右侧为 i16 机械臂和末端夹爪预览；
完整底盘与相机模型仍在 Isaac / RViz 中显示。
快捷位保存于 Qt AppConfigLocation 下的 `quick_positions.json`，不会自动导入学生现场坐标。

## 验证

14 项回归测试通过，另完成 GUI 窗口及模型渲染检查。

在 source 工作空间环境后运行：

```bash
ROS_DOMAIN_ID=134 /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  src/AuboAMR300_ROS2/aubo_control_gui/test \
  --ignore=src/AuboAMR300_ROS2/aubo_control_gui/test/test_sim_workflow.py

ROS_DOMAIN_ID=133 /usr/bin/python3 \
  src/AuboAMR300_ROS2/aubo_control_gui/test/test_sim_workflow.py \
  --execute --output /tmp/isaac_gui_result.json
```

第二条会使仿真机械臂及夹爪运动，并返回测试起始位置。
[实测结果](test/results/isaac_gui_20260910.json)包括关节规划、末端目标、夹爪开合、运动中取消及返回。
取消返回状态 5（CANCELED），之后 1 秒最大关节变化 0.0004 rad。

## 来源与后续实机接入

GUI 基于用户提供的 `aubo_develop_student/aubo_i16_pc_ws/src/aubo_control_gui`，
保留其 PySide6 控件和模型显示，替换 ROS 控制链路。新 `motion_client.py` 可脱离 Qt 验证。
`aubo_student_description` 是学生 `aubo_description` 的模型资源副本，重命名避免与官方驱动同名包冲突。
学生 Jetson 夹爪驱动、厂商 SDK 二进制、内核模块及现场 systemd 配置未导入本轮运行链路。

下一轮实机工作需实现/配置标准机械臂、夹爪控制器，并核对 SDK、真实关节状态、
夹爪行程映射、停止语义和工具坐标。完成后 GUI 继续使用相同 MoveIt 接口。

关键位置另见 [四个存储位、放置位及过渡点验证](KEY_POSITION_VALIDATION.md)：四个存储位在当前完整模型中存在夹爪与底盘碰撞，未启用为默认快捷位。

## macOS 本地 GUI（Fast DDS）

界面、机械臂预览、参数输入及快捷位文件在 Mac 本地；MoveIt、控制器和 Isaac 在 Ubuntu 仿真机。
通信使用原有 ROS 2 topic/service/action，经 `rmw_fastrtps_cpp` 直接传输，不使用 SSH 转发控制命令。
Mac 需要 ROS 客户端运行库；单独安装 PySide6 或 Fast DDS 不足以运行本 GUI。
Apple Silicon 环境使用 RoboStack Lyrical，避免 Mac 与 MoveIt 的消息发行版不同。

首次安装（已有 conda 或 [micromamba](https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html) 时）：

```bash
cd ~/Develop/github/AuboAMR300_ROS2
# micromamba 可以替换为 conda；在独立环境中安装，不要装进 base。
micromamba env create -p ~/.venvs/aubo-ros-lyrical \
  -f aubo_control_gui/environment-macos.yml
./scripts/build_macos_moveit_msgs.sh
```

远端当前 `moveit_msgs=2.7.2`，RoboStack 二进制包为 2.7.1，`MotionPlanRequest` 相差 `smoothness_level` 字段。
上面的构建脚本将官方 2.7.2 消息包编译到独立 overlay；启动器优先加载它，不替换远端软件。
后续远端消息版本升级时，需要重新核对并同步此 overlay。

启动仅规划模式：

```bash
cd ~/Develop/github/AuboAMR300_ROS2
./scripts/start_macos_gui.command
```

也可在 Finder 中双击 `scripts/start_macos_gui.command`。
指定仿真执行模式：

```bash
./scripts/start_macos_gui.command --execute
```

切换仿真机或 DDS domain：

```bash
./scripts/start_macos_gui.command --peer 192.168.3.133 --domain-id 133
```

启动器设置 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=133`、
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 和 `ROS_STATIC_PEERS=192.168.3.133`。
启动器使用仓库内 `config/fastdds_macos.xml`（UDPv4、自动端口），覆盖进程继承的 DDS profile 和 discovery server 设置。
这避免已有全局 profile 仅绑定 127.0.0.1 或固定端口而导致跨机失败；不会修改全局配置文件。
静态 peer 为指定主机建立发现连接，不代表仅能访问该主机，也不是访问控制机制。
远端 ROS 节点必须使用相同 domain、兼容的 DDS，发现范围不能为 OFF，且网络允许 DDS UDP 双向通信。
如果需要在远端显式添加 Mac（当前地址 `192.168.3.131`），在启动相关 ROS 进程前设置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=133
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=192.168.3.131
```

环境变量只影响之后启动的进程。Mac IP 变化时同步调整配置。
启动 GUI 不会启动远端仿真；Ubuntu 仿真与 MoveIt 须先按上文启动。
默认仅规划模式中，机械臂和夹爪按钮均只请求规划；`--execute` 才允许执行。
首次操作先使用“当前关节角 → 初始”，然后设置目标及速度/加速度比例。
快捷位保存于 Mac 的 Qt AppConfigLocation，不会自动同步 Ubuntu 的快捷位文件。
末端显示仍是 `base_footprint` 下的 `wrist3_Link`，不是夹爪尖端 TCP。

参考：[RoboStack 安装](https://robostack.github.io/GettingStarted.html)、
[ROS 2 静态 peer 与发现范围](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst)。

本次跨机通信验证（2026-09-10）：Mac arm64 / RoboStack Lyrical / Fast DDS → Ubuntu Lyrical，
关节反馈、FK 和 MoveGroup 仅规划请求通过（返回码 1），未执行运动。
[验证结果](test/results/macos_fastdds_20260910.json)。可自行复查：

```bash
./scripts/start_macos_gui.command --check
```

`--check` 不打开窗口，也不能与 `--execute` 同时使用。
启动器优先加载 overlay 的 Python 模块及动态库，避免 Python 使用新消息、DDS 序列化仍加载旧库。

Mac 本地窗口及机械臂预览已验证。PySide6 6.11.2 的 RuntimeLoader 仅列出 OBJ/glTF/GLB，
因此 Mac 使用由原始 DAE 转换的 GLB；Ubuntu 继续使用 DAE。
七个 GLB 由 Assimp 6.0 `assimp export linkN.DAE linkN.glb -fglb2` 生成，
转换前后面数及变换后的边界一致（GLB 对顶点进行了去重）。
这些文件仅用于 GUI 预览，不改变仿真 URDF、碰撞网格或目标参数。
[转换核验](test/results/macos_mesh_conversion_20260910.json)。

### GUI 夹爪预览

右侧模型包含连接板、电机和两片长夹指。安装位姿及夹指运动轴从当前
`seer_description/urdf/composite_robot_stick_mono.urdf` 读取，挂在 `wrist3_Link` 下。
`gripper1_joint` 和 `gripper2_joint` 的实际反馈分别驱动两片夹指，单位为米；
点击开合按钮不会直接伪造模型位置，仍需等待仿真反馈。默认仅规划模式不会使夹爪实际开合。

预览 GLB 来自本地及仿真机一致的 STL，来源哈希、面数和边界核验在
`meshes/gripper/provenance.json`。转换合并重复顶点并使用 URDF 的灰/黑/白材质；
仅增加显示资源，原始 STL、仿真模型及目标参数不变。
