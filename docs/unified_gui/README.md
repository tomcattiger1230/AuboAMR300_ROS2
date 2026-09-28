# SEER + AUBO 统一控制台

统一 GUI 的首版代码在 `aubo_control_gui/aubo_control_gui/unified/`。
它提供底盘地图与导航、机械臂与夹爪、连接配置、独立子系统状态和日志。
打开窗口只展示离线布局；点击“连接机器人”才启动通信。

## 运行

当前仓库的普通 Python 环境可直接预览，无需 ROS、Isaac 或设备：

```bash
.venv/bin/python scripts/start_unified_gui.py
.venv/bin/python scripts/start_unified_gui.py --mode real
.venv/bin/python scripts/start_unified_gui.py --page arm
```

macOS 推荐独立的统一 GUI ROS 环境；现有机械臂专用环境缺少导航消息和 TF。
依赖解析会升级 ROS 栈，因此将新环境与已有验证环境分开：

```bash
micromamba create -y -p "$HOME/.venvs/aubo-unified-ros-lyrical" \
  -f aubo_control_gui/environment-unified-macos.yml
export AUBO_GUI_ENV="$HOME/.venvs/aubo-unified-ros-lyrical"
export AUBO_GUI_OVERLAY="$HOME/.local/share/aubo-unified-gui-overlay"
./scripts/build_macos_moveit_msgs.sh
```

再构建实机接口并启动：

```bash
# 实机机械臂所需的自定义消息；只构建接口，不加载 SDK 或访问设备
./scripts/build_macos_aubo_interfaces.sh
./scripts/start_unified_gui.command --mode isaac --domain-id 133 --peer 192.168.0.103
./scripts/start_unified_gui.command --mode real --domain-id 20 --peer 192.168.0.103 --seer-ip 192.168.3.250
```

Ubuntu 的 ROS 工作空间构建并 source 后：

```bash
colcon build --symlink-install --packages-select aubo_bridge_msgs aubo_student_description aubo_control_gui
source install/setup.bash
ros2 run aubo_control_gui seer_aubo_gui --mode isaac --domain-id 133 --peer 192.168.0.103
```

`seer_description` 模型资源也需要在工作空间中构建或存在于本源码仓库。
已有独立机械臂 GUI 的启动入口继续可用。

## 模式和界面

| 项目 | Isaac 仿真 | 真实 SEER + AUBO |
|---|---|---|
| 默认 ROS Domain | 133 | 20，可修改以匹配设备 |
| 时间 | ROS 仿真时钟 | 系统时间 |
| 底盘驾驶 | ROS `/cmd_vel` | SEER TCP 19205 / 2010、停止 2000 |
| 导航 | Nav2 `/navigate_to_pose` | SEER TCP 19206 / 3051 站点导航 |
| 取消导航 | Nav2 action cancel | SEER 3003 |
| 暂停 / 继续 | 本首版未提供 | SEER 3001 / 3002 |
| 地图 | `/map`、map→base_footprint TF、`/plan` | 本地 `.smap`，车端地图坐标位置反馈 |
| 机械臂 | 已有 MoveIt 规划、三维拖动、轨迹预览和独立执行 | AUBO ROS 桥关节 / 末端请求、三维反馈 |
| 夹爪 | MoveIt gripper 组 | `/aubo/gripper_command`，需已有夹爪桥 |

连接期间锁定模式、ROS 域和 IP。断开并等待请求终态后才能切换。
两种地图和两种导航语义分开：仿真填 map 坐标，实车选择车端已有的 LM 站点。
载入本地 `.smap` 后，实机导航自动列出 LM 点位及坐标，按编号排序。
下拉选择、双击 SEER 地图站点和手动输入目标 ID 会同步地图高亮与下拉选择。
切换地图会清空上一次目标；地图点击和点位选择仅设置目标，发送按钮才启动导航。
“停止底盘与机械臂”同时请求底盘停止、导航取消和机械臂停止，属于软件请求。

## 同网段部署

界面中的“ROS 桥主机 IP”是运行 ROS / AUBO 桥的电脑地址，
不是 SDK 机械臂控制柜地址。“SEER IP”直接用于底盘 TCP 连接。
默认 IP 只是现有项目的预设，请按现场设备地址修改。

连接时使用 `rmw_fastrtps_cpp`、SUBNET 发现和静态 ROS peer。
本机与设备应互通，ROS 域和消息定义应一致；同网段并不自动保证 DDS 端口可达。
GUI 不负责启动 Isaac、MoveIt、Nav2、AUBO Bridge，也不自动上电或解锁机械臂。

仿真使用实时 SLAM 地图时，按 `seer_description/ISAAC_NAVIGATION_CAMERA.md` 启动 Isaac、建图与导航。
使用已保存地图时，按下面的静态地图定位流程启动。GUI 复用已有 MoveIt，不启动第二套控制器。

## 仿真地图与初始位置

1. 选择仿真模式，连接匹配的 ROS 域和主机。
2. 点击“载入仿真地图 · YAML”选择 YAML，配套 PGM/PNG 会一起读取。此时只显示本地预览。
3. 填写“远程地图路径”，该路径必须是 ROS 主机上已存在的 YAML 绝对路径。本机文件不会自动上传；钢筋实验室地图会预填本次已保存的远程路径。
4. 点击“应用到仿真”，通过 `/map_server/load_map` 请求远程加载，成功后切回远程 `/map`。
5. 勾选“在地图上选择初始位置与朝向”，从已知空闲位置向朝向方向拖动；也可填写初始 X、Y、朝向。
6. 点击“设置初始位置”，发布 `/initialpose`。位置必须对应机器人实际所在位置，该操作只修改定位估计，不移动机器人。
7. 等待“AMCL 已返回定位结果”。导航另外需要 Nav2 action 和实时 `map` TF 就绪。

“显示 ROS 实时地图”可退出本地预览。本地预览、地图请求期间或等待新定位反馈时不能发送导航目标。
地图和初始位置都需要显式按钮确认；选择文件、拖动地图不会自动请求设备。
本地预览支持标准 `trinary` YAML + PGM/PNG，支持地图原点旋转、反色和图片行方向转换。

ROS 主机先停止 SLAM 建图与其 scan_filter，再启动定位，避免多套节点发布 `/map` 和 `map → odom`：

```bash
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_mm_ws/install/setup.bash
export ROS_DOMAIN_ID=133
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# 填写运行 GUI 的电脑当前 IP：
export ROS_STATIC_PEERS=192.168.0.153
ros2 launch seer_description isaac_localization.launch.py \
  map:=/home/arnold/.local/share/aubo-mapping-runs/rebar_lab_20260928/export/rebar_lab.yaml
```

Isaac 与其机器人 ROS stack 也需要使用同一 ROS 域和局域网发现配置；改变发现配置后需要重启相关进程。
本次远程主机依赖安装在用户目录，需要先加载
`/home/arnold/.local/share/aubo-mapping-runs/rebar_lab_20260928/environment.lan.sh`。
该文件包含本次 Mac 的静态 peer，Mac IP 改变后也要更新。
定位启动器包含 map_server、AMCL、scan_filter 和自动生命周期管理，不自动启动导航控制器。
需要导航时另启 `isaac_navigation.launch.py`，并确保其 Nav2 依赖已经安装。

2026-09-28 已从 Mac GUI 实际验证本地预览、远程加载、`/initialpose` 发布、AMCL 回包及 `map → base_footprint` TF。
载入地图为 492×372、0.05 m/像素，初始估计为 (0,0,0)，AMCL 反馈约 (0.014,0.011,0.002 rad)。
验证中底盘线速度与角速度均为零，未发送导航或机械臂动作；截图为 `simulation_map_preview.png`。

## 实机 ROS 桥

实机先在 ROS 桥主机运行匹配版本的 AUBO Bridge，并配置真实 `robot_host`、
`robot_port`。如果使用独立 Jetson 夹爪桥，AUBO Bridge 的
`gripper_enabled` 应为 false，Jetson 与 GUI 的 ROS 域须一致。
GUI 端需编译相同 `aubo_bridge_msgs`；不需要安装厂商 AUBO SDK。
实机机械臂页明确使用 SDK 桥接口，首版不提供 MoveIt 碰撞规划或学生固定快捷路线。

## Isaac 实验实时观察

仿真模式增加“Isaac 实时场景”页，通过 ROS 主机的 8081 端口显示实际第三视角 JPEG，
并显示取筋、运送、竖直化、接管等实验阶段。仅点击连接或使用 `--connect` 才启动画面请求；
断开后停止请求，服务无新画面时会清空画面。

```bash
./scripts/start_unified_gui.command --mode isaac --domain-id 133 \
  --peer 192.168.0.103 --page scene --connect
```

远程 Isaac 启动前设置 `REBAR_VIDEO_DIR`，第三视角记录器会同时生成原始帧和原子更新的 `latest.jpg`。
对应画面服务为：

```bash
python3 seer_description/scripts/observer_video.py \
  --directory "$REBAR_VIDEO_DIR" --host 0.0.0.0 --port 8081
```

实验流程仍由独立的 `record_rebar_workflow.sh --compact` 执行；实时场景页用于观察。
“底盘与导航”页显示 ROS 地图位置，“机械臂与夹爪”页显示实时关节模型与夹爪反馈。
2026-09-28 已实际验证实时场景页，截图为 `isaac_live_experiment.png`。
本次运行目录为远程 `/home/arnold/.local/share/aubo-experiments/rebar_live_20260928_224230/`；
竖直化与预定位使用 1 倍参考速度，以便观察。
八个实验阶段均已完成；随后按用户要求停止远程仿真、实验流程、画面服务与定位节点。

## 验证边界

SEER 源码复用自本机 `hongshi_mobile_manipulator`，提交
`13ec3c09fee147819fa34f13aacc9916ba6d57e6`：

- `agv_core/client.py`、`protocol.py`、`smap.py`、`status_semantics.py`；
- `ui/map_view.py`、`styles.py`，调整地图模块的相对导入和未载入地图提示。

来源目录已保存状态查询、定位和 LM 站点导航的历史验证报告。
`docs/agv_api_verified.md` 对手动开环运动仍记录未验证项，较新 GUI 已实现该接口。
因此实机手动驾驶默认关闭，现场确认 2010 / 2000 的单位和停止行为后，
可在连接前勾选“实机开环接口已现场验证”。历史验证不等于本次完整组合验收。
底盘手动速度限制为 0.3 m/s、0.6 rad/s；按住持续发送，松开、切页和窗口失去活动状态请求停止。

真实机械臂请求需 IDLE、上电、释放刹车及新鲜关节 / 状态反馈；请求期间禁止再次发送。
SDK 桥的同步运动、反馈刷新、停止命令响应仍须在现场组合验收。
当前桥是单线程 ROS spin，同步 SDK 调用可能推迟 STOP topic 回调，
且 STOP 依赖 SDK 提供 `move_stop`。统一 GUI 等待请求和设备终态，不将本地“已发送”作为已停止。
本次修复了桥接 `/aubo/move_to_pose` 写入未定义响应字段的问题。

实机末端请求使用速度参数 0.02；现有桥把该参数传给关节速度设置，不能将其宣称为 TCP 线速度。

本次不联系实车、不发运动指令；离线 / localhost 测试不能替代 Isaac 和实机验收。

## 检查

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest \
  aubo_control_gui/test/test_unified_console.py \
  aubo_control_gui/test/test_view_math.py \
  aubo_control_gui/test/test_tool_preview.py \
  aubo_control_gui/test/test_presets.py -q
```

覆盖模式预览不建立连接、配置校验、速度限制、TCP 分片响应、
实机驾驶门控、松开停止顺序、过期 / 未知急停反馈、旋转地图坐标转换。
`test_unified_ros.py` 在具有生成消息的 ROS 环境中运行，固定 LOCALHOST 发现和
ROS Domain 221，检查真实类型、机械臂模块及共享 ROS context，不连接设备。

界面截图：`arm_preview.png`、`real_preview.png`，均为未连接设备的离线界面。

## 本次验证结果（2026-09-28）

31 项检查通过，其中 25 项离线检查和 6 项 localhost ROS 检查。
ROS 检查包含两种模式的底盘与机械臂同时载入、配置锁定和断开恢复。
独立 Mac ROS 环境、MoveIt 2.7.2 消息覆盖层和 AUBO 消息接口已在本机建立；
启动脚本已验证。原生 Qt 三维模型和两种模式的截图已检查。
未执行远端 Isaac / 实机运动；下一阶段应分别验收导航、机械臂执行和软件停止。
