# 本地精简车载取筋至竖直预定位路径（2026-09-28）

本次在 Mac 本地通过 `uv` 环境重新规划，未连接 Ubuntu。
结果是**本地运动学候选**，还未进行 MoveIt/FCL 或 Isaac 动态验证。

## 路径变化

范围是机器人在开阔点 `(6.08, 2.60)` m、朝向 `-90°` 停车后，
从车载第 1 槽抓取钢筋，到测试台前竖直预定位姿态。保留原有抓取接触、
脱离鞍座和抬到 `0.90 m` 的步骤。从这个抬升点开始：

| 现有流程 | 新候选 |
|---|---|
| 再抬到 `1.05 m` | 从 `0.90 m` 直接开始同步抬升、转向 |
| 移到 `(-0.35, 0.55, 1.20)` 侧向中间点 | 取消该中间点 |
| 中间点原地竖直化，末端姿态变化 `180°` | 六个关节按不同进度连续单向变化 |
| 再经关节规划到正面姿态，至少增加 `90°` | 一条曲线到 `(-0.80, -0.08, 1.50)` |

两条路径的最终 TCP 位姿相同：工具轴沿底盘 `-X`，夹爪开合轴沿
底盘 `-Y`，钢筋沿 `+Z`。新候选采用不同的肩部逆解分支，末端轴线和
抓取中心不变。该分支的 `wrist2_joint` 始终为正值，未跨过零角度。
后续底盘接近和末端微调仍需从这个新关节分支重新验证。

## 本地结果

| 指标 | 现有流程的几何下界 | 新候选 |
|---|---:|---:|
| TCP 路程（从 `0.90 m` 抬升点算起） | ≥1.839 m | 1.776 m |
| 末端累计姿态变化 | ≥270° | 174.0° |
| 总关节变化量 | 未记录旧轨迹，不能比较 | 7.235 rad，无折返 |

新候选的累计姿态变化比旧流程的必要转动量少约 **35.5%**；
TCP 路程比旧流程的几何下界少约 **3.5%**。旧 OMPL 实际轨迹未保存，
这里没有把旧的直线连点当成完整仿真轨迹，也没有给出执行时间或成功率。

进行了 440 次参数评估，并对最终曲线做 801 个状态的密集筛选。
相邻采样最大关节变化 `0.327°`；模型中最小保守间隙约 `10.1 mm`，
发生在钢筋与上臂之间。所有关节都在当前 URDF 限位内，末端到达目标位姿。

- [对比图](test/results/rebar_compact_local_20260928/comparison.png)
- [本地骨架动画，9 秒](test/results/rebar_compact_local_20260928/preview.mp4)
- [指标、起终点关节角和约束](test/results/rebar_compact_local_20260928/report.json)
- [路径 CSV](test/results/rebar_compact_local_20260928/trajectory.csv)
- [NumPy 路径数据](test/results/rebar_compact_local_20260928/trajectory.npz)

动画时长仅用于可视化，**不是机器人执行时长**。CSV 的 `progress` 是
归一化路径参数，不是时间；尚未做实际速度、加速度和抓取稳定性验证。

## 本地模型与复现

脚本展开仓库当前 `composite_robot_finger_mono.urdf.xacro`，读取实际
关节变换、限位及碰撞图元，并读取对应 SRDF 的碰撞豁免关系。
FK 与之前保存的 MoveIt 抬升位姿一致；对旧正面预定位关节角进行 FK，
TCP 差约 `0.16 mm`，与原报告一致。

碰撞筛选包含底盘、机械臂、夹爪、腕部相机、料架和测试机静态盒体。
钢筋按当前 MoveIt 的 `0.62 m` 长度、`20 mm` 半径包络处理。
圆柱使用覆盖球链，盒体使用定向盒分离轴检测；这是保守图元的采样筛选，
未替代连续碰撞检测。其他已装车槽位、其他仓库设施及运动抱爪不在此模型内。

起点关节分支由旧 GUI 报告的 `0.90 m` 抬升逆解重建，并非当前机器人反馈。
即使 TCP 相同，实际抓取后处于不同逆解分支时，也不能直接使用这组关节路径。

在仓库根目录执行：

```bash
uv sync
uv run python seer_description/scripts/rebar_pose_planner/optimize_rebar_vertical_path.py \
  --samples 70 --start-height 0.90 \
  --output-dir seer_description/test/results/rebar_compact_local_20260928

uv run python seer_description/scripts/rebar_pose_planner/render_offline_rebar_path.py \
  seer_description/test/results/rebar_compact_local_20260928/trajectory.npz \
  seer_description/test/results/rebar_compact_local_20260928/preview.mp4

uv run pytest -q seer_description/test/test_offline_rebar_model.py
```

规划器使用固定随机种子，搜索不同关节的单调进度参数，兼顾 TCP 路程与
姿态累计变化；密集采样不满足 `8 mm` 阈值的候选会被拒绝。
视频导出需要本地 `ffmpeg`。

## 下一次仿真验证

已有八段仿真节点仍使用原路径。待允许连接 Ubuntu 后，应读取真实抓取后
关节状态，重新匹配起点分支与实际钢筋抓取偏移，再由 MoveIt 检查完整场景、
重定时曲线。通过后才将旧“抬到 1.05 m → 侧向竖直化 → 正面预定位”
替换为这一段动作，并验证载筋驶近、微调和测试机接管。
