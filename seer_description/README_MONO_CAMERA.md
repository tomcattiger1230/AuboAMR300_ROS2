# MV-CH100-60UM + 12 mm C 口镜头版本

本版本保留底盘、六轴机械臂、末端夹爪和双雷达，将末端的 Gemini RGB-D 模型换为黑白工业相机（挂接在 `wrist3_Link`）。原 Gemini 版本仍使用原来的文件和启动入口。

另有保留同一相机、改用 `finger.STL` 与 `motor_adapter.STL` 的
[新夹爪版本](README_FINGER_GRIPPER.md)。

## 独立模型

| 用途 | 文件 |
|---|---|
| 相机机身、镜头、光学帧 | [mv_ch100_60um_12mm.urdf.xacro](urdf/mv_ch100_60um_12mm.urdf.xacro) |
| 完整机器人 Xacro | [composite_robot_stick_mono.urdf.xacro](urdf/composite_robot_stick_mono.urdf.xacro) |
| 展开的 URDF | [composite_robot_stick_mono.urdf](urdf/composite_robot_stick_mono.urdf) |
| Gazebo SDF | [composite_robot_stick_mono.sdf](urdf/composite_robot_stick_mono.sdf) |
| 机器人 USD（ASCII） | [seer_aubo_stick_mono.usda](urdf/seer_aubo_stick_mono.usda) |
| 仓库场景 USD | [warehouse_stick_mono_demo.usda](urdf/warehouse_stick_mono_demo.usda) |
| 独立 MoveIt 配置包 | `seer_aubo_stick_mono_moveit_config` |

USD 通过独立覆盖层禁用旧相机及其关节，添加新的刚体、碰撞体、腕部固定关节和实际 USD Camera。不要只单独复制覆盖层：底盘、机械臂和仓库继续引用同目录的已有资产。

## 参数及模型边界

按用户选择使用 MV-CH100-60UM 黑白版。产品资料为 4096×2460、3.45 μm 方形像元、C 口、USB3.0、全局快门；机身 29×44×59 mm，约 113 g，标称全幅最高 36 fps。来源：[产品页](https://absolutegauge.com/product/mv-ch100-60u-10-mp-area-scan-camera/)、[厂商规格书](https://absolutegauge.com/download/ae903c8c8cbf69f3bdabfd729e093bd2/)。

12 mm 焦距来自用户要求。有效感光面由像元尺寸计算为 14.1312×8.487 mm，理想针孔视场为水平 60.98°、垂直 38.95°；原生图像 fx、fy 约 3478.26 px。预览模式对整幅图像降采样，保持相同视场，不是裁剪 ROI。

尚未提供镜头具体型号和安装 CAD，因此镜头外形暂用直径 35 mm、长度 40 mm 的圆柱，质量暂估 80 g，组合惯量也为估计值。机身前端面按末端安装结构暂放在 `(0, 0.1, 0)`，相对 `wrist3_Link`，绕其 Z 轴旋转 180°，镜头光轴沿腕部 +Z（与 `camera_joint` 关系见 xacro）。光学原点暂放在镜头前端 40 mm 处。这些安装外参、入口瞳位置和镜头包络需按实物标定，不能用于加工安装支架。相机机身尺寸采用规格书，不代表精确 CAD。

模型假定镜头像场覆盖整块感光面，未模拟暗角。图像模型为无畸变理想针孔；零畸变系数不是实际镜头标定结果。不模拟量子效率、曝光电路、触发时序、USB 带宽或 MVS SDK。Isaac 内部 RGB 渲染经灰度转换后发布 `mono8`；它不能代表实际传感器的光谱响应。这个单目版本不发布深度，也关闭了旧 MoveIt 深度相机 OctoMap 配置。

## 构建和启动

在自己的工作区加载主机 ROS 发行版后：

```bash
colcon build --packages-select seer_description seer_aubo_stick_mono_moveit_config
source install/setup.bash
export ROS_DOMAIN_ID=133
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run seer_description start_warehouse_stick_mono_demo.sh --gui --domain-id 133
```

默认 `--camera-resolution preview` 为 1024×615。要测试原生 10 MP，先停止当前 stack，再启动：

```bash
ros2 run seer_description start_warehouse_stick_mono_demo.sh \
  --gui --domain-id 133 --camera-resolution full
```

另一个同环境终端启动 SLAM 和黑白视频：

```bash
ros2 launch seer_description isaac_mapping.launch.py \
  video_host:=0.0.0.0 image_topic:=/camera/image_raw \
  rviz_config:=isaac_navigation_mono.rviz start_rviz:=true
```

远程主机浏览入口是 `http://192.168.3.133:8080/`。单独视频可运行：

```bash
ros2 run --prefix /usr/bin/python3 seer_description camera_video.py \
  --ros-args -p host:=0.0.0.0 -p image_topic:=/camera/image_raw
```

发布接口：`/camera/image_raw`（mono8）、`/camera/camera_info`，光学帧 `camera_optical_frame`。`/camera/render/image_raw` 是实现灰度转换的内部渲染话题。没有 `/camera/depth/image_raw` 发布器。机械臂继续使用 `arm` 规划组、`gripper` 夹爪组。

## 生成及验证

安装相机 Xacro 后，可重新生成 URDF/SDF；下面的 `$MODEL_DIR` 指源码中的 `seer_description/urdf`：

```bash
ros2 run --prefix /usr/bin/python3 seer_description generate_mono_robot_models.py "$MODEL_DIR"
~/isaacsim/.venv/bin/python "$MODEL_DIR/../scripts/generate_mono_camera_usd.py" "$MODEL_DIR"
```

USD 生成器需要带 `pxr` 的 Python，不能直接混用主机 ROS Python 与 Isaac Python。

保存的 URDF/SDF 为可移植的 ROS 模型，其中控制器参数路径保留 `$(find seer_description)`。Gazebo 使用 SDF 前先展开该路径：

```bash
xacro "$MODEL_DIR/composite_robot_stick_mono.sdf" > /tmp/composite_robot_stick_mono.sdf
gz sdf -k /tmp/composite_robot_stick_mono.sdf
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix seer_description)/share:${GZ_SIM_RESOURCE_PATH:-}"
```

SDF 已通过解析检查；本轮实际运行与运动测试在 Isaac 完成，未声称新 SDF 已在 Gazebo 完成全部控制测试。`gz_frame_id` 是 Gazebo ROS 扩展，SDFormat 校验可能输出保留该字段的提示。

在线图像检查（用主机系统 Python，避免 `.venv` 没有 OpenCV）：

```bash
ros2 run --prefix /usr/bin/python3 seer_description test_mono_camera.py --output /tmp/mono_preview.json
# full 模式：追加 --width 4096 --height 2460
ros2 run seer_description test_isaac_arm.py --execute --output /tmp/mono_arm.json
```

原生模式实测收到 4096×2460 mono8，K 中 fx=fy≈3478.26，深度发布器为 0；10 秒监测窗口收到 5 帧。厂商 36 fps 是硬件规格，不是当前 RTX 仓库场景的仿真帧率保证。预览模式实测为 1024×615，10 秒接收 19 帧；机械臂规划、IK、笛卡尔运动和回位通过，最大关节误差约 0.00121 rad。完整结果见 [测试记录](test/results/mono_camera_20260910.json)，[黑白画面示例](test/results/mv_ch100_60um_preview.jpg)。

## macOS GUI 中查看相机

按 [GUI README](../aubo_control_gui/README.md) 启动后，使用“相机特写”查看机身和镜头，再用“整体视图”返回全景。
GUI 启动时从本文件所列的组合 URDF 读取工具几何；连接后从远端运行中的 `/robot_description` 同步夹爪、相机几何及安装位置，不需要另行导入 USD。
这只显示三维外观；实时视频仍使用上面的网页服务。更新源码后需要重启 GUI，已打开的窗口不会热加载模型。

### 2026-09-10：GUI 相机安装位置跟随运行中的 Ubuntu 模型

Ubuntu 当前运行的相机相对 `wrist3_Link` 为 `(0, 0.1, 0)` m、绕 Z 旋转 180°。
此前本地 URDF 已将父节点改为 `gripper_motor_link`，但 Ubuntu 运行中的模型未重载，导致两端显示不同。
GUI 订阅远端 `/robot_description`（transient-local），同步夹爪、相机几何和腕部固定关节链。
工具栏显示“模型：远端 robot_description”后，右侧工具外观以运行中的机器人描述为准；收到描述前使用本地 stick 模型。
按用户确认，相机 URDF/Xacro、SDF、USD 及 USD 生成脚本均保持 `wrist3_Link` 安装，与当前运行模型一致。
