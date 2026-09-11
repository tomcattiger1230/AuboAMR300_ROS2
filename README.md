# AuboAMR300_ROS2

SEER AMB-300 底盘、AUBO i16H 机械臂、末端夹爪和工业相机的 ROS 2 开发与 Isaac 仿真项目。
当前整合与验证面向 Isaac；实机部署留待后续。GUI 在 macOS 本地运行，通过 Fast DDS 连接 Ubuntu 上的 MoveIt 与仿真。

## 文档导航

| 内容 | 文档 |
|---|---|
| Mac GUI 安装、启动、拖动末端、规划/执行与排障 | [GUI README](aubo_control_gui/README.md) |
| Isaac 与 ROS 2 启动及发行版兼容性 | [Isaac ROS 2](seer_description/README_ISAAC_ROS2.md) |
| 双雷达、SLAM、导航和视频 | [导航与相机](seer_description/ISAAC_NAVIGATION_CAMERA.md) |
| 黑白相机参数、URDF/SDF/USD、视频入口 | [MV-CH100-60UM + 12 mm](seer_description/README_MONO_CAMERA.md) |
| 新 finger + motor_adapter 夹爪模型与启动 | [新夹爪版本](seer_description/README_FINGER_GRIPPER.md) |
| GUI 模型资源来源与 i16H 限位 | [模型资源 README](aubo_student_description/README.md) |
| 学生关键位置与碰撞验证 | [关键位置验证](aubo_control_gui/KEY_POSITION_VALIDATION.md) |

## Mac GUI 快速启动

完成 [GUI 环境安装](aubo_control_gui/README.md#macos-本地-guifast-dds)，并先在 Ubuntu 启动 Isaac / MoveIt。

```bash
cd ~/Develop/github/AuboAMR300_ROS2
./scripts/start_macos_gui.command --check
./scripts/start_macos_gui.command --execute
```

默认仿真机 `192.168.3.133`、ROS domain `133`、`rmw_fastrtps_cpp`。
检查成功应包含 `fresh: true`、`moveit: true`、`fk: true`、`plan_error_code: 1`。
`--check` 只规划、不执行；`--check-gui` 验证窗口反馈和远端相机安装描述后自动关闭。
`--execute` 允许单独点击执行，不会在启动时自动运动。

窗口右上方“相机特写”定位并放大末端相机，拖动空白处旋转、滚轮缩放；“整体视图”恢复机械臂全景。
修改代码后须关闭旧 GUI 并重新启动；源码启动优先读取当前仓库的模型与 QML。

## 2026-09-11 更新

- 使用 Blender 调整原点后的 `finger_centered.stl` 和 `motor_new.stl` 建立独立 URDF、SDF、USD 和 MoveIt 版本，原始网格与 stick 夹爪继续保留。
- 新旧夹爪沿用相同 ROS 关节接口；macOS GUI 现在从远端 `/robot_description` 同步整套夹爪与相机外观。
- 新版本构建、URDF/SDF 解析、USD 结构及 GUI 模型解析已通过离线检查；实际 Isaac 运动测试需在切换 stack 后执行。

## 2026-09-10 更新

- GUI 分离目标设置、规划预览与执行，支持鼠标拖动末端、关节目标及夹爪规划。
- i16H J3 限位统一为 ±161°；关键存储位的碰撞问题仍需结合模型与安装参数复核，未关闭碰撞检查。
- MV-CH100-60UM 黑白相机及 12 mm C 口镜头加入 GUI 几何和特写入口；GUI 安装位置优先跟随 Ubuntu 运行中的机器人描述。镜头外形与安装外参仍属近似模型。
- 增加 GUI 反馈自检、启动配置与缺失关节日志，以及普通 UDP 发送诊断。
- macOS 终端曾出现 UDP `Broken pipe`，随后用户终端的 UDP、DDS 反馈、FK 和仅规划检查全部通过。此类错误先检查终端网络访问，不应归因于输入法日志。

相机三维外观已包含在 GUI；视频使用独立网页入口（参见相机文档），尚未嵌入 GUI。
本次新增相机显示与视角检查未执行机器人运动；历史仿真运动测试记录见 GUI README。

### 2026-09-10：GUI 相机安装位置跟随运行中的 Ubuntu 模型

Ubuntu 当前运行的相机相对 `wrist3_Link` 为 `(0, 0.1, 0)` m、绕 Z 旋转 180°。
此前本地 URDF 已将父节点改为 `gripper_motor_link`，但 Ubuntu 运行中的模型未重载，导致两端显示不同。
GUI 订阅远端 `/robot_description`（transient-local），同步工具几何、夹指运动轴及相机安装变换。
工具栏显示“模型：远端 robot_description”后，右侧夹爪与相机以运行中的机器人描述为准；收到描述前使用本地 stick 模型。
按用户确认，磁盘中的相机 URDF/Xacro、SDF、USD 及 USD 生成脚本均保持 `wrist3_Link` 安装，与当前运行模型一致。
