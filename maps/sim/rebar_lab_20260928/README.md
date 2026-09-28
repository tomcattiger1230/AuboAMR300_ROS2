# 钢筋实验室仿真地图 · 2026-09-28

远程主机 `arnold@192.168.0.103`，场景 `warehouse_finger_rebar_lab_demo.usda`。
使用 ROS 2 Lyrical、Isaac Sim、前雷达过滤数据及 SLAM Toolbox 实际采集，未发送机械臂、夹爪、测试机或 Nav2 导航动作。

## 地图文件

- `rebar_lab.yaml` + `rebar_lab.pgm`：供 Nav2 map_server / AMCL 使用的二维占据栅格。
- `rebar_lab.png`：PGM 的无损 PNG 预览，白色为空闲、黑色为障碍、灰色为未知。
- `rebar_lab.posegraph` + `rebar_lab.data`：SLAM Toolbox 序列化数据，可用于后续继续建图。
- `manifest.json`：地图统计、现场停车反馈与场景/建图配置摘要。
- `survey.json` / `survey.log`：底盘采集路线和关节反馈记录。
- `map_rebar_lab.py`：本次执行脚本的原样快照。
- `map_saver.log` / `map_load_validation.log`：保存与 Nav2 重新加载检查。
- `SHA256SUMS`：四个地图核心文件的远程 SHA256 校验值，本地复制后已逐一核对。

地图为 **492 × 372** 栅格，分辨率 **0.05 m/像素**，原点 **(-12.335, -9.245, 0)**。
已知栅格 **176151 / 183024（96.24%）**。未知区域需要保留为未知，不能当作可通行区域。
YAML 中的图片路径为 `rebar_lab.pgm`，可整体移动本目录到 ROS 主机。

## 采集结果与运行状态

底盘以最高 0.3 m/s 采集设备前方、东侧、后方、中部及西侧通道，完成 10 个采集位置。
最后返回精确原点时，在取料台附近触发雷达距离保护；实际终点约为 `(0.213, -0.005) m`。
因此 `survey.json` 中的路线完成标志为 `false`，记录了最后一个返回目标被阻止；地图已经成功保存和序列化。
后续可重复使用仓库 `scripts/map_rebar_lab.py`，默认返回点已调整为 `(0.4, 0)`，留出取料台前停车距离。

停车后连续 4 秒里程计的线速度、角速度和位置变化均为零。
采集期间机械臂关节有微小仿真反馈变化，最大跨度 0.002 rad；没有发送机械臂运动动作。
采集结束时远程 Isaac 场景与 SLAM 保持运行，采集程序已经退出；远程日志目录为
`/home/arnold/.local/share/aubo-mapping-runs/rebar_lab_20260928/`。
建图依赖解包在用户目录 `/home/arnold/.local/share/aubo-mapping-runtime/`，未修改系统 ROS 安装。

随后为 GUI 地图加载与初始定位验收，远程场景复位并启用局域网 ROS 发现，SLAM 已停止，
改为 map_server + AMCL 定位。启动环境为同一远程日志目录中的 `environment.lan.sh`。
当前使用本目录的已保存地图；GUI 流程见 `docs/unified_gui/README.md`。
随后完成 GUI 实时观察实验，按用户要求停止远程 Isaac、实验流程、画面服务及定位节点。
重新使用地图时需重新启动场景与定位服务。

已通过 Nav2 map_server 的实际重新加载检查及本地 PGM/PNG 图像读取检查。
本次没有执行导航测试，后续定位与导航应另外验收。

## 使用地图

将本目录复制到 ROS 主机后，设置 `yaml_filename` 为该主机上的 YAML 绝对路径，例如：

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=true \
  -p yaml_filename:=/your/ros/workspace/maps/sim/rebar_lab_20260928/rebar_lab.yaml
```

该命令仅启动生命周期地图服务，还需配置并激活该节点；定位、TF 和 Nav2 导航组件另外启动。
继续 SLAM 建图时使用序列化的 `.posegraph` / `.data`；该用途与加载静态栅格地图进行定位不同。
