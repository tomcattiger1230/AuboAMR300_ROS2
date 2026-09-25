# 钢筋拉伸测试机 Isaac Sim 场景

`urdf/warehouse_finger_etm6m_rebar_tester_demo.usda` 在现有复合机器人、仓库和 ETM-6M 抗渗仪场景中加入一台钢筋拉伸测试机。模型依据用户提供的照片制作，包含开放式双立柱、深蓝底柜、可分别升降的上下横梁、两组可分别张合的抱爪和独立控制台。这里的路径均相对于 `seer_description/`。

## 模型位置

| 场景 | 文件 | `/World/RebarTestMachine` 基点 | 绕 Z 旋转 | 操作面 |
| --- | --- | --- | --- | --- |
| 单台演示 | `urdf/rebar_test_machine.usda` | (4.1, 2.1, 0) m | 0° | 世界 `-X` |
| 双抗渗仪联合场景 | `urdf/rebar_test_machine_lab.usda` | (6.0, 4.0, 0) m | 90° | 世界 `-Y` |

坐标系为 Z 向上；钢筋拉伸区域在双立柱之间，控制台在机架的局部 `-Y` 一侧。`/World/RebarTestMachine/UpperCarriage` 与 `/World/RebarTestMachine/LowerCarriage` 分别包含横梁、抱爪头和左右爪片。上抱爪朝下，下抱爪朝上。两台抗渗仪及机器人在联合场景中的位置见 [钢筋实验室场景](README_REBAR_LAB.md)。

## 启动

```bash
ros2 run seer_description start_warehouse_finger_etm6m_rebar_tester_demo.sh --gui --no-rviz --domain-id 133
```

移动单台演示设备后重新生成 USD；联合场景的坐标在 `scripts/generate_rebar_lab_scene.py` 中修改：

```bash
python3 seer_description/scripts/generate_rebar_test_machine_scene.py --x 4.1 --y 2.1 --yaw 0
```

两个启动脚本都会通过 `--rebar-tester` 启用控制桥接。在运行仿真的 Ubuntu 上另开终端，加载 `/opt/ros/lyrical/setup.bash`，并设置与仿真相同的 `ROS_DOMAIN_ID`（示例为 133）。向各个 `std_msgs/msg/Float64` 目标话题发布数值即可分别移动横梁或张合抱爪。数值单位均为米；`z` 是设备局部竖直高度，开口是两个爪面的净间距。`_state` 话题返回仿真实际值；速度限制为升降 0.12 m/s、开口 0.08 m/s。

| 指令话题 | 状态话题 | 允许范围 | 初始值 |
| --- | --- | ---: | ---: |
| `/rebar_tester/upper/z_cmd` | `/rebar_tester/upper/z_state` | 1.50–1.88 | 1.68 |
| `/rebar_tester/lower/z_cmd` | `/rebar_tester/lower/z_state` | 0.92–1.34 | 1.05 |
| `/rebar_tester/upper/opening_cmd` | `/rebar_tester/upper/opening_state` | 0.024–0.16 | 0.08 |
| `/rebar_tester/lower/opening_cmd` | `/rebar_tester/lower/opening_state` | 0.024–0.16 | 0.08 |

例如，单独下降上抱爪并收紧下抱爪：

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=133
ros2 topic pub --once /rebar_tester/upper/z_cmd std_msgs/msg/Float64 '{data: 1.55}'
ros2 topic pub --once /rebar_tester/lower/opening_cmd std_msgs/msg/Float64 '{data: 0.024}'
ros2 topic echo --once /rebar_tester/upper/z_state
ros2 topic echo --once /rebar_tester/lower/opening_state
```

四路指令可按任意顺序单独发送；未指定的机构保持自己的上次目标。上下横梁中心始终保持至少 0.28 m 距离；超出行程的指令会被钳制到边界。实现位置：`scripts/rebar_tester_bridge.py` 接收 ROS 2 指令，`scripts/rebar_tester_control.py` 在 Isaac 中限速更新模型，并发布实际状态。

在联合场景中，两组抱爪闭合且钢筋已对准夹持线、竖直放置时，Isaac 将钢筋刚体保持在当前位置，并在 `/rebar_tester/rebar_gripped`（`std_msgs/msg/Bool`）发布 `true`。自动装填脚本收到该反馈后才松开机械臂夹爪；张开任一组抱爪会解除保持。该行为用于装填流程验证，不代表真实夹持力。

照片没有比例尺或机械图纸，尺寸和行程是仿真估计值。机架和控制台有静态碰撞体；移动横梁与抱爪为运动可视化模型，尚未模拟夹持力、钢筋拉伸/断裂、力传感或试验数据。自动装填脚本会临时向 MoveIt 加入机架与控制台碰撞体；详情见 [钢筋实验室场景](README_REBAR_LAB.md#机械臂抓取并放置钢筋)。获得实物尺寸或 CAD 后，可保留 `/World/RebarTestMachine` 路径替换外观模型。
