# 钢筋抓取与释放测试（Isaac Sim + MoveIt 2）

在 Isaac Sim 仓库场景中加入钢筋工位，用 finger 双指夹爪完成
**接近 → 下探 → 夹取 → 提起 → 搬运 → 释放** 的全流程自动化测试。
2026-09-16 在 Ubuntu 26.04 / ROS 2 Lyrical / Isaac Sim 6.x 上全流程验证通过
（10 项检查全部 pass，报告见 `test/results/`）。

## 场景组成

钢筋工位由 `generate_rebar_station.py` 生成（`urdf/rebar_station.usda`）：

- 工位位于机械臂臂展内（默认 x = -1.15 m，机械臂底座朝 -X 方向）
- 工位台（0.9 × 0.5 × 0.45 m，静态碰撞体）
- 两个**低摩擦**（friction 0.05）支撑块：钢筋放置其上，被夹起时可顺滑脱出；
  高摩擦块会导致 1 m 钢筋两端卡死、提起失败
- **钢筋**：动态刚体圆柱，24 mm 直径 × 1 m 长（与 object_detection 检测管线
  的标称试样一致），钢密度计算质量约 3.55 kg，锈色，高摩擦表面（0.8）

两个演示场景（wrapper 层叠，未改动任何既有 USD）：

| 场景文件 | 夹爪 | 说明 |
|---|---|---|
| `warehouse_finger_rebar_mono_demo.usda` | finger 双指 | **推荐**，可夹钢筋 |
| `warehouse_stick_rebar_mono_demo.usda` | stick 板式 | 仅场景预留，闭合间隙 50 mm，夹不住钢筋 |

> stick 夹爪两指为上下板式结构，全闭仍有约 50 mm 间隙，只适合 50–130 mm
> 厚的物体；钢筋测试必须使用 finger 变体。

## 启动

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to seer_description seer_aubo_finger_mono_moveit_config
source install/setup.bash

# 带 GUI 演示：
ros2 run seer_description start_warehouse_finger_rebar_mono_demo.sh --gui --domain-id 133
# 无头验证：
ros2 run seer_description start_warehouse_finger_rebar_mono_demo.sh --no-rviz --domain-id 133
```

启动脚本继承 `start_isaac_ros2_stack.sh` 的全部参数
（`--ros-bridge-mode`、`--renderer` 等），lyrical 主机自动使用 internal bridge。

## 运行测试

```bash
# 完整抓取/释放测试，JSON 报告写入 /tmp/rebar.json，退出码 0 = 全部通过
ros2 run seer_description test_rebar_grasp.py --output /tmp/rebar.json

# 仅规划不执行（验证 IK / 路径可行性）
ros2 run seer_description test_rebar_grasp.py --plan-only
```

测试流程与判定：

| 步骤 | 判定 |
|---|---|
| 张开夹爪 | 手指回到 0 ± 1.5 mm |
| TCP 标定 | FK 计算夹爪 TCP 相对 wrist3 偏移（finger 版为 (0, 0, 0.16)） |
| 预抓取 IK | 多种子 KDL IK + 角度解绕回 + 前向分支代价惩罚 |
| 接近 / 下探 / 提起 / 搬运 | OMPL 关节规划 + 笛卡尔直线路径，经 `/execute_trajectory` 执行 |
| 夹取判定 | 手指在到达指令行程前**停滞**（接触钢筋） |
| 保持判定 | 提起后手指位置不变（负载仍在） |
| 释放判定 | 手指完全回到 0 |

常用参数：`--rebar-x/-y/-z`（钢筋位置）、`--diameter`、`--close-position`、
`--approach-height`、`--lift-height`、`--release-dy`、`--tcp-y/--tcp-z`。

规划走 move_group 服务（OMPL + 笛卡尔路径），执行经
`/aubo_arm_controller/follow_joint_trajectory`（action_bridge）与
`/execute_trajectory`，与 RViz 的 Plan & Execute 同一条链路，不依赖 MoveItPy。

## 静态校验

```bash
~/isaacsim/python.sh test/test_rebar_station_usd.py   # 需 USD (pxr) 环境
```

校验钢筋的刚体/质量/碰撞 API、支撑块静态与低摩擦材质、wrapper 层叠完整性。

## 重新生成场景

```bash
ros2 run seer_description generate_rebar_station.py \
  [--station-x -1.15] [--radius 0.012] [--length 1.0] [--table-height 0.45]
```

## 已知事实（实测）

- finger 夹爪指面开口实测 ≈ 0.0464 m（非碰撞盒推算值）；夹 24 mm 钢筋时
  手指停在约 0.015 / 0.019 m，呈 V 形边缘夹持、两侧不完全对称，属正常现象。
- 抓取姿态：wrist Z 竖直向下，wrist X（闭合轴）水平横切钢筋（默认 `--wrist-x-axis y`）。
- 释放后钢筋会掉落滚动，**重复测试前需重启仿真**以复位钢筋位置。
- `start_isaac_ros2_stack.sh` 的就绪等待为 300 s：Isaac 冷启动可达 135 s 以上。
