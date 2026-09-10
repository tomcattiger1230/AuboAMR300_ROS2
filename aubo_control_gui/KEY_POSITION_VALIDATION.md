# 学生关键运动位置验证（Isaac，2026-09-10）

核对了四个存储位、一个放置位和四个过渡点。原始数据保存在 [student_key_positions.json](config/student_key_positions.json)，未改写用户提供的学生代码或现场坐标。

| 位置 | 静态碰撞检查 | MoveIt 规划 | Isaac 执行 | 腕部位置误差 |
|---|---|---|---|---|
| 放置位 | 通过 | 通过 | 通过 | 2.40 mm |
| 存储位 1 | 夹爪与底盘碰撞 | 拒绝 | 未执行 | — |
| 存储位 2 | 夹爪与底盘碰撞 | 拒绝 | 未执行 | — |
| 存储位 3 | 夹爪与底盘碰撞 | 拒绝 | 未执行 | — |
| 存储位 4 | 夹爪与底盘碰撞 | 拒绝 | 未执行 | — |
| WP1 | 通过 | 通过 | 通过 | 1.45 mm |
| WP2 | 通过 | 通过 | 通过 | 2.10 mm |
| WP3 | 通过 | 通过 | 通过 | 1.11 mm |
| COMMON | 通过 | 通过 | 通过 | 2.39 mm |

测试后已规划并执行回到测试前的关节位置。结果仅适用于本次模型、姿态和 MoveIt 场景。

## 坐标核对

学生位姿使用 `aubo_base_link`，参考末端为 `wrist3_Link`。五组保存关节角计算出的 FK 与学生笛卡尔位姿一致，最大位置差约 0.00121 mm；比较姿态时处理了四元数 q / -q 的等价性。
这说明问题不是角度单位、四元数顺序或基座坐标混用。
仿真机械臂相对车体安装位置为 `(-0.3, 0, 0.6) m`，绕 Z 轴旋转 π，不能把学生坐标直接解释为车体坐标。

## 四个存储位为何被拒绝

在当前模型中，四个位置均检测到 `gripper1_link ↔ base_link` 和 `gripper2_link ↔ base_link` 碰撞；夹爪张开、闭合两种状态都不通过。
底盘碰撞体是中心 `(0,0,0.35) m`、尺寸 `1.00 × 0.70 × 0.50 m` 的盒体。当前长夹爪的碰撞几何在这些腕部姿态下进入该盒体。
这是当前仿真模型的碰撞结论，不代表原学生台架或真实夹爪必然碰撞；后续需要核对工具尺寸、底盘碰撞近似和工作位布局。
本次没有关闭碰撞检查、缩小碰撞体，也没有擅自抬高原始目标。四个存储位不应直接作为已通过的位置启用。

## 验证范围与复现

放置位按原保存关节角执行，过渡点按原位姿求无碰撞 IK 后规划执行。位置误差由执行反馈的 FK 与目标 FK 比较获得。
验证的是关键点及本次 MoveIt 生成的连接路径，尚未验证学生原始逐段直线轨迹、所有存储位之间的往返组合或物体抓取。
Isaac 仓库场景几何尚未完整导入 MoveIt；碰撞判定基于当前 MoveIt 场景。

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=133
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 src/AuboAMR300_ROS2/aubo_control_gui/test/validate_key_positions.py \
  --execute --output /tmp/student_key_positions_result.json
```

去掉 `--execute` 仅检查 FK、IK、碰撞和规划。已知碰撞目标无论是否传入该选项都不会执行。

[完整实测记录](test/results/student_key_positions_20260910.json)
