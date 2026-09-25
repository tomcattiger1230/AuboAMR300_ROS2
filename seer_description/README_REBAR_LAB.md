# 复合机器人、钢筋抓取与三台测试设备

联合场景 `urdf/warehouse_finger_rebar_lab_demo.usda` 包含 finger 复合机器人、原钢筋取料工位、可抓取动态钢筋、车载四槽料架、两台 ETM-6M 抗渗仪和一台照片参照的钢筋拉伸测试机。所有路径均相对于 `seer_description/`。北侧两组货架 `Rack_03`（中心 3, 4.4 m）及 `Rack_04`（中心 7.5, 4.4 m）以及地面圆锥 `Cone` 在该场景中停用；其他仓库场景不受影响。

## 模型位置与组成

| 设备 | Isaac prim | 世界坐标基点 (m) | 资产文件 |
| --- | --- | --- | --- |
| ETM-6M 1 | `/World/ETM6M_1` | (2.2, 4.0, 0) | `urdf/etm6m_lab_1.usda` |
| ETM-6M 2 | `/World/ETM6M_2` | (4.0, 4.0, 0) | `urdf/etm6m_lab_2.usda` |
| 钢筋拉伸测试机 | `/World/RebarTestMachine` | (6.0, 4.0, 0) | `urdf/rebar_test_machine_lab.usda` |

坐标是设备模型基点，世界坐标系为 Z 向上，单位为米。三台设备均绕 Z 轴旋转 90°，操作面朝世界 `-Y` 的南侧通道，并整体沿 `+Y` 移动 1.2 m，为机器人留出更宽的中部通道。两个抗渗仪的近似外形均为局部 X × Y × Z = 0.9 × 1.1 × 0.9 m。拉伸机主体位于其基点附近，独立控制台位于主体的世界 `+X` 一侧；上下运动部件路径及尺寸见 [钢筋测试机说明](README_REBAR_TEST_MACHINE.md)。相邻设备的外缘间距约 0.7–0.8 m；该距离依据当前近似碰撞体计算，不代表实物维护空间要求。

`urdf/warehouse_finger_rebar_lab_demo.usda` 将原 `urdf/warehouse_finger_rebar_loading_demo.usda`、三个设备资产叠加。机器人、钢筋工位与四槽车载料架沿用原场景；本场景没有重新定位它们。模型位置由 `scripts/generate_rebar_lab_scene.py` 内的 `equipment_usda(...)` 和 `machine_usda(...)` 参数确定。修改后运行下述生成命令，并重启 Isaac 才会生效。

## 启动与控制

在 Ubuntu 已构建的 ROS 2 工作区中：

```bash
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_mm_ws/install/setup.bash
ros2 run seer_description start_warehouse_finger_rebar_lab_demo.sh --gui --no-rviz --domain-id 133
```

启动入口含 `--rebar-loading` 和 `--rebar-tester`：前者将钢筋取料工位和车载料架加载到 MoveIt 规划场景，后者启动拉伸机的 ROS 2 控制桥接。另开终端并设置相同 `ROS_DOMAIN_ID`，可分别控制上下横梁升降与上下抱爪张合：

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=133
ros2 topic pub --once /rebar_tester/upper/z_cmd std_msgs/msg/Float64 '{data: 1.55}'
ros2 topic pub --once /rebar_tester/lower/z_cmd std_msgs/msg/Float64 '{data: 1.18}'
ros2 topic pub --once /rebar_tester/upper/opening_cmd std_msgs/msg/Float64 '{data: 0.16}'
ros2 topic pub --once /rebar_tester/lower/opening_cmd std_msgs/msg/Float64 '{data: 0.024}'
ros2 topic echo --once /rebar_tester/lower/opening_state
```

每条指令只修改对应机构，单位为米；`_state` 返回实际运动后的位置。行程、初始值、限速、上下抱爪方向及其他状态话题见 [钢筋测试机说明](README_REBAR_TEST_MACHINE.md)。原钢筋抓取脚本仍可使用，将钢筋从原工位装入车载槽位：

```bash
ros2 run seer_description test_rebar_grasp.py --onboard-slot 1 --output /tmp/rebar_lab_grasp.json
```

重新生成三台设备和场景 wrapper：

```bash
python3 seer_description/scripts/generate_rebar_lab_scene.py
```

## 机械臂抓取并放置钢筋

现有 `scripts/test_rebar_tester_load.py` 串联原钢筋抓取、底盘驶向测试机、钢筋竖直化与插入、测试机抱爪闭合、机械臂松爪和撤回。脚本运行时将测试机机架及控制台加入 MoveIt 规划场景，并通过钢筋实际位姿与抱爪状态核对过程。此前联机验证已到双抱爪闭合；测试机接管钢筋、机械臂松爪和撤回仍待在新 Ubuntu 主机上完成验证。运行方法和几何参数见 [自动装填说明](README_REBAR_TESTER_LOAD.md)。

抗渗仪尚未加入 MoveIt 规划场景；装填脚本会临时加入拉伸测试机的机架与控制台碰撞体。运动横梁与抱爪为可视化运动模型，不模拟接触力或拉断。拉伸机尺寸依据照片估计，尚无厂家 CAD 或实测尺寸。钢筋抓取本身仍使用原工位和原车载料架，不改变其位置或碰撞几何。
