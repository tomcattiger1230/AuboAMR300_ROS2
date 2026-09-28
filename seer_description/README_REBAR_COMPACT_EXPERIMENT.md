# 精简机械臂轨迹与第三视角完整实验（2026-09-28）

本轮把车载取筋后的竖直化、测试台正面预定位合为一次连续关节运动。
完整流程仍按节点执行：取筋装车 → 载筋导航 → 车载取筋 → 同步竖直化与预定位
→ 预定位状态确认 → 竖直持筋接近 → 夹持线微调 → 测试机夹紧、机械臂松爪。
最后一阶段验证钢筋由测试机保持，不包含拉伸力或钢筋拉断仿真。

## 完整复跑结果

修正后从复位场景自动连续完成八个节点，无人工接续。共 `97/97` 项检查通过，
各阶段分别为 `25、11、16、12、6、7、12、8` 项。
新机械臂曲线的 `1001/1001` 个状态通过 MoveIt 碰撞筛查；计划执行 `44 s`。
根据实际关节反馈计算，TCP 路程约 `1.810 m`，累计姿态变化约 `180.26°`。
最终测试机双爪夹紧、机器人夹爪张开、钢筋仍由测试机保持，报告均通过。

- [第三视角完整视频，2 分 42 秒](test/results/rebar_compact_experiment_20260928/rebar_compact_automation.mp4)
- [检查和轨迹汇总](test/results/rebar_compact_experiment_20260928/summary.json)
- [计划与实测轨迹对比](test/results/rebar_compact_experiment_20260928/execution_comparison.png)
- [夹紧与松爪报告](test/results/rebar_compact_experiment_20260928/08.json)

录像为 `1280×720 / 24 fps` 输出，第 04 段接近实际动作速度，其余段加速。
最终视频来自独立复跑，未拼入前面调试时需要恢复的试验。

## 轨迹与数据位置

- 执行节点：`scripts/rebar_tester_mission/04_compact_vertical.py`。
- 曲线和速度参数：`scripts/rebar_tester_mission/compact_curve.py`。
- 本地重新优化：`scripts/rebar_pose_planner/optimize_rebar_vertical_path.py`。
- 实验录像、报告和遥测：`test/results/rebar_compact_experiment_20260928/`。
- 本地优化候选：该目录的 `planning/`；`planning/report.json` 的候选状态
  只描述本地优化阶段，远程实验结果由独立节点报告描述。
- 早期 GUI 起始分支的离线候选：
  [README_REBAR_COMPACT_LOCAL_PLAN.md](README_REBAR_COMPACT_LOCAL_PLAN.md)。

保留钢筋脱离鞍座及抬至 `0.90 m` 的必要动作，省去再次抬到 `1.05 m`、
绕到侧面中间点、原地竖直化、再单独规划到正面的动作。
终点 TCP 为底盘坐标 `(-0.80, -0.08, 1.50) m`；工具轴沿底盘 `-X`，
夹爪开合轴沿 `-Y`，钢筋沿 `Z`。
六个关节使用不同进度的五次平滑曲线，同步单向运动，避免多段往返。

本轮实际车载取筋后的关节分支与早期 GUI 候选不同，故用实测关节角重新优化。
采用此前完成过驶近和交接的终点关节分支。局部候选 TCP 路程约 `1.806 m`、
累计姿态变化约 `180.36°`；旧流程的指令姿态变化下界为 `270°`，减少约 `33.2%`。
这些是几何规划指标，旧 OMPL 实际轨迹未保存，不能据此比较完整旧轨迹的耗时。
轨迹计划执行时间为 `44 s`，速度不超过 URDF 上限的 `6%`，加速度不超过
`0.12 rad/s²`，再留 `10%` 时间裕量。

## 远程运行

Ubuntu：`arnold@192.168.0.103`，工作区：
`/home/arnold/Develop/ROS_ws/hongshi_mm_ws`。模型入口为
`seer_description/urdf/warehouse_finger_rebar_lab_demo.usda`。
在 Ubuntu 的 **bash** 中分别运行以下两个终端。每次完整实验前重新启动场景复位。

```bash
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_mm_ws/install/setup.bash
export ROS_DOMAIN_ID=133
export REBAR_VIDEO_DIR=/tmp/rebar_compact_demo
ros2 run seer_description start_warehouse_finger_rebar_lab_demo.sh --no-rviz --domain-id 133
```

等仿真就绪后，在第二个终端运行：

```bash
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_mm_ws/install/setup.bash
export ROS_DOMAIN_ID=133
export REBAR_VIDEO_DIR=/tmp/rebar_compact_demo
bash ~/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2/seer_description/scripts/record_rebar_workflow.sh --compact
```

不加 `--compact` 使用原来的分段流程。新流程仍按八个节点输出报告，
第 05 节点只确认第 04 节点已达到的姿态，不再重复移动机械臂。
单独运行新轨迹节点前必须已完成车载第 1 槽取筋：

```bash
ros2 run seer_description pickup_onboard.py --onboard-slot 1 --at-standoff \
  --compact-start --state-file /tmp/rebar_compact_demo/workflow_state.json \
  --output /tmp/rebar_compact_demo/pickup.json
ros2 run seer_description 04_compact_vertical.py \
  --state-file /tmp/rebar_compact_demo/workflow_state.json \
  --output /tmp/rebar_compact_demo/compact.json
```

`04_compact_vertical.py --validate-only` 只筛查并导出轨迹；
`--schedule-file /path/planning/report.json` 使用自定义优化参数。
完整流程可设置 `REBAR_COMPACT_SCHEDULE` 指向同类 JSON。

## 筛查与物理验证

执行前读取实际关节角、夹爪闭合位置和负载状态，更新钢筋、车载料架、
测试机及两个抗渗设备的 MoveIt 碰撞场景。逐一筛查 `1001` 个状态，
相邻关节变化不得超过 `0.5°`；筛查后底盘或机械臂发生漂移则拒绝执行。
执行时监测钢筋保持状态，保存 `04.trajectory.json` 和 `04.motion.json`。
导航节点等待新鲜底盘反馈；驶近测试台时连续 `8 s` 无进展则停止并报告。

驶近节点按横、纵方向各 `20 mm` 检查预停靠误差，航向误差小于 `0.03 rad`。
这段驾驶只控制纵向速度和转向，不能消除小的继承横向偏差；末端微调继续按
实测钢筋世界坐标对准测试机。之前的欧氏距离 `20 mm` 判据在横向已有约
`18 mm` 偏差时会造成纵向已到位却无法结束；诊断报告保存在 `diagnostics/`。
停止后可加 `05_drive_vertical_rebar.py --resume-drive` 在已检查的接近走廊内
恢复，仍校验钢筋保持、竖直姿态、底盘朝向及微调前的钢筋世界坐标。

第一次试验的另一终点分支虽通过 MoveIt 筛查及竖直化，但底盘在距目标约
`12 cm` 处停滞。这说明简化碰撞场景通过不能替代全流程物理验证。
因此改用已完成过交接的终点分支，重新优化并运行。
失败记录保存在 `rejected_branch/`，保留故障现场图片；没有覆盖失败记录。

## 视频

录像采用 `1280×720`、约 `4 fps` 第三视角采集，随阶段切换取料、导航和
测试台观察相机。拼接时加上中文阶段标注；第 04 段以 `4 fps` 播放，
其余段加速展示。视频覆盖装车、导航、测试台装填及夹紧松爪全过程。

```bash
python3 ~/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2/seer_description/scripts/assemble_rebar_video.py \
  /tmp/rebar_compact_demo /tmp/rebar_compact_demo/rebar_compact_automation.mp4 --compact
```

需要 Ubuntu 上的 `ffmpeg` 和 Noto CJK 字体。
`--stage-start-frame 02=帧编号` 可以裁去某阶段开头的等待时间；源帧清单保持不变，
视频 JSON 记录实际使用的起止帧和播放速度。

## 本地检查

```bash
uv sync
uv run pytest -q seer_description/test/test_compact_curve.py \
  seer_description/test/test_offline_rebar_model.py \
  seer_description/test/test_rebar_pose_planner_core.py
```

测试包括曲线与优化数据一致、关节周期角就近选择及限位、限速与加速度、
本地运动学和 GUI 规划核心。Mac 上不运行 Isaac 物理验证。

下载八个报告及第 04 节点轨迹和遥测后，可以生成汇总与执行对比图：

```bash
uv run python seer_description/scripts/rebar_pose_planner/analyze_compact_experiment.py \
  seer_description/test/results/rebar_compact_experiment_20260928
```

输出 `summary.json`、`execution_comparison.png`。实测关节经名义 URDF 计算的
TCP 与实际钢筋中心分别报告；反馈累计路程包含控制误差与采样噪声。
单次完整成功不能用来估计成功率。
