# Isaac 场景：SLAM、导航、机械臂和腕部视频

本流程用于 `warehouse_stick_demo.usda` 仿真。2026-09-10 实测环境为 Ubuntu 26.04.1、ROS 2 Lyrical、Isaac Sim 6.0.1-rc.7、RTX 4090。Isaac 子进程使用自带 Jazzy 桥接；SLAM、Nav2、MoveIt 使用主机 Lyrical。不要把主机 Python 3.14 的 ROS 库注入 Isaac Python 3.12。

## 启动顺序

各终端先进入自己的工作区并加载实际安装的 ROS 发行版。下面是本次远程主机示例：

```bash
cd ~/Develop/ROS_ws/hongshi_mm_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=133
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

1. 启动带夹爪的仓库场景和 MoveIt：

```bash
ros2 run seer_description start_warehouse_stick_demo.sh --gui --domain-id 133
```

2. 场景就绪后，启动建图、视频和导航 RViz：

```bash
ros2 launch seer_description isaac_mapping.launch.py \
  video_host:=0.0.0.0 start_rviz:=true
```

视频节点默认仅监听本机。上面显式绑定局域网后，浏览器访问 `http://192.168.3.133:8080/`。`/stream.mjpg` 是实时 MJPEG，`/snapshot.jpg` 是当前 JPEG。断流超过 3 秒不继续展示旧帧；新请求返回 HTTP 503。服务没有认证，仅用于受信任的仿真局域网。ROS 图仍限制在本机，不需要让浏览器加入 DDS 网络。

3. 在仓库空旷原点转一圈，补充扫描后启动导航：

```bash
ros2 run seer_description test_isaac_mapping.py --execute --output /tmp/mapping_turn.json
ros2 launch seer_description isaac_navigation.launch.py
```

等生命周期节点全部 active 后，可在导航 RViz 的 Nav2 Goal 工具中设置目标，或运行中心通道四目标测试：

```bash
ros2 run seer_description test_isaac_navigation.py --execute --output /tmp/navigation_result.json
```

路线为 `(1.5,0) → (1.5,1.5) → (0,1.5) → (0,0)`，约 6 m；从原点附近开始，速度 0.25 m/s。测试先调用规划服务，再执行 NavigateToPose，记录每段路径与 TF 实际到达位置，最后检查墙体目标被拒绝。省略 `--execute` 只检查规划。不要与键盘遥控同时使用。

4. 底盘停止后测试机械臂：

```bash
ros2 run seer_description test_isaac_arm.py --execute --output /tmp/arm_result.json
```

依次验证六轴关节规划、腕部 IK、5 mm 插补步长的笛卡尔路径、回到初始关节位置，以及不可达位姿拒绝。省略 `--execute` 只规划。RViz 选择 `arm` 进行六轴末端规划；原有 `manipulator` 保留八轴联合关节目标，`gripper` 保留开合目标。当前测试的末端是 `wrist3_Link`，不是经过标定的夹持中心 TCP。

## 实现与检查点

- 前后 RTX 雷达原始话题为 `/front_lidar/scan`、`/back_lidar/scan`。过滤话题追加 `_filtered`，保留原始时间戳和 TF；无效 `-1`、非有限值和越界距离改为 NaN，防止自遮挡被误当作无障碍空间。角度末端按实际数组长度校正。
- SLAM Toolbox 使用前雷达、0.05 m 栅格，发布 `map → odom`；Isaac 发布 `odom → base_footprint`。前后雷达都进入 Nav2 局部障碍层，远近距离限制显式配置。双雷达点云合并尚未实现。
- 全局静态层开启 `footprint_clearing_enabled`，清除车体当前占据区域中的扫描盲区；规划禁止穿过其余未知区域。机器人轮廓为 1.0 × 0.8 m，另加 5 cm padding；这是底盘轮廓，导航时应保持机械臂收拢，不能代表伸展机械臂的全部扫掠体积。
- 万向轮材质使用 `min` 摩擦组合规则，确保零摩擦不会与地面取平均后变成阻力；驱动轮使用 `max`。仅设置摩擦系数不足以保证底盘能移动。
- Nav2 显式使用 `Twist`，适配 Isaac。`cmd_vel_watchdog.py` 将 `/cmd_vel` 转发到 `/isaac_cmd_vel`，用单调时钟检测 0.5 秒超时并发零速度；仿真暂停也能检测失联。应始终通过正式 stack 启动器启动这两个节点。
- 机械臂增加 `aubo_base_link → wrist3_Link` 的独立 `arm` 链，只给该链加载 KDL，解决原八轴分支组无法初始化 IK 的问题。
- 相机由 `camera_joint` 固定在 `wrist3_Link`，模型使用 Gemini 33X 简化资产；这证明仿真模型有腕部相机，不代表已经检查实体机器人的相机硬件。RGB、对齐深度和 CameraInfo 分别为 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`。分辨率 640×480；深度编码 32FC1。视频编码使用 ROS 启动器对应的系统 Python，避免 `.venv` 中缺少 OpenCV 的问题。

## 本次实测结果

四段导航均成功，实际到达误差为 0.112 / 0.145 / 0.146 / 0.145 m；墙体目标被拒绝。六轴关节规划、腕部 IK、笛卡尔轨迹和返回均成功，笛卡尔完成率 100%，最大关节误差约 0.00115 rad；10 m 高的不可达位姿返回 -31。视频 10 秒记录 46 帧，实测约 4.81 帧/秒（墙钟），640×480，确认画面随机器人运动变化。

地图已保存到 `maps/warehouse_demo.yaml` / `.pgm`，栅格 494×377、分辨率 0.05 m。它是这次有限路线的建图样本，灰色区域仍未观测。完整指标见 [测试记录](test/results/isaac_20260910.json)，预览见 [地图](maps/warehouse_demo.png)。远程完整日志和 MP4 在工作区 `log/codex_navigation/`。

## 保存与复现

地图保存为 PGM/YAML，可供后续定位使用：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/warehouse_map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

回归测试在独立 DDS 域运行：

```bash
ROS_DOMAIN_ID=136 /usr/bin/python3 src/AuboAMR300_ROS2/seer_description/test/test_navigation_adapters.py
~/isaacsim/.venv/bin/python src/AuboAMR300_ROS2/seer_description/test/test_stick_usd.py
```

发行版与 Python 路径需按主机调整；Humble/Jazzy 主机保留原来的 system/internal 桥接选择，不能将这次 Lyrical 实测理解为所有发行版都经过验证。Nav2 依赖已写入 package.xml，安装依赖时使用主机对应发行版的 rosdep。

当前验证覆盖中心通道低速导航与机械臂短距离运动，不涵盖整仓自动探索、动态人群、长期运行、真实雷达标定、抓取保持力。机械臂测试使用 MoveIt 当前规划场景的碰撞检测，尚未把仓库 USD 的全部障碍物自动导入 MoveIt；不能据此认定机械臂已能绕开任意仓库物体。

参考：[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)、[Nav2 TwistStamped 迁移说明](https://discourse.openrobotics.org/t/notice-nav2-migrated-to-twiststamped-to-replace-twist-for-cmd-vel-topics/40944)。
