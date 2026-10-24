# 两料区、三台同型测试机：工位参数化与离线任务流程

本轮完成代码和离线验证，未启动 Isaac、ROS、导航服务、仿真或训练。原393b策略及其optimizer文件只读核对SHA，没有加载或保存模型。原正式64/64、独立演示1/1属于已封存场景；本轮结果不加入该分母。

## 输入与范围

任务输入为 `task_id + source_id + machine_id + steel_id`。用户给料区的地图位置，系统要求具体钢筋由视觉定位。两料区和三台同型测试机的示例见 `config/multi_workcell_offline.json`。

**示例位置、导航站点名、视觉编号和标定ID均为离线样例，不是现场测量值。** `geometry_status=synthetic`。实际部署前必须绑定真实地图、测量工位、标定相机与TCP、配置身份识别及验证机器人可达/碰撞；将字段改成surveyed本身不会授权执行。

目前局部视觉技能仍按直径24mm、长600mm、可完整观察的单根钢筋、水平支承平面和同型竖直夹具设计。本轮没有验证堆叠分拣、其他钢筋尺寸、斜置夹具或不同机械结构。

## 工位配置

- `map_id / revision / calibration_id`：所有任务、回执和图像证据必须匹配；布局规范化SHA同时绑定具体坐标，修改位置后旧回执失效。
- `pose_map_xyz_yaw`：工位参考系在地图中的XYZ与偏航，位置m、角度rad，Z向上。当前只支持平面平移/偏航，未假装支持roll/pitch。机器人反馈的base原点必须与停靠目标、相机TF采用同一语义。
- `profiles`：料台模板保存支承面高度、搜索范围和观察停靠偏置；机器模板保存夹具目标、钢筋轴线、预插入偏置、撤离向量、停靠偏置。
- `stations`：实例只绑定模板、地图位姿、导航ID和唯一视觉ID。三台机器共用 `test_machine_v1`；增加实例不复制上料控制程序。
- 源料区只给支承面和搜索区域，代码没有从配置生成“已识别钢筋位姿”。视觉回执必须提供完整钢筋、三帧不同且按时间排序的稳定观测，中心范围≤4mm、轴向一致，并检查两端都在指定料区。

坐标链为 `地图 ← 工位模板 ← 局部目标`，再由实测底盘位姿变换到base坐标。目标点、预插入点、轴线、腕部姿态和撤离向量一起变换。示例machine_2旋转90°时，地图中的预插入偏置从+Y变为-X。

## 任务流程和回执

`navigate_source → observe_source → pickup_to_tray → navigate_machine → observe_machine → load_machine`

`MissionFlow`只产生意图并消费测量回执，没有网络请求、轮速写入或环境step。每次只有一个未完成command，绑定task/command/step/station/map/layout SHA。重复、乱序、错机或旧布局回执不能推进阶段；超时、忙机、失载、视觉错误时进入hold，要求停底盘、保持机械臂、保留夹持，不盲目松爪。

- 去料区允许空载，但必须定位有效、机械臂收拢、手指打开无载、停止状态及路径检查成立。
- 钢筋视觉确认要求新鲜同帧图像/K/TF、匹配标定、有效刚体旋转、料区身份、尺寸和三帧稳定性；配置坐标不能替代观测。
- 抓取放槽回执要求原技能阶段连续、真实抓取确认、托槽载荷/位置/轴向、手指卸载及收臂。载料导航复用现有`TransportGate`检查契约。
- 到机器后重新识别指定visual_id、确认机器未占用且就绪、底盘停稳，并以视觉精定位更新夹具参考系。偏离登记位置超过250mm或15°要求重新定位。
- 上料完成回执保留3mm/3°、四爪承载、总载荷≥40N、松爪卸载、撤离≥60mm、钢筋速度、unsafe≤250N及terminal/peak arm contacts<0.5N等边界。
- 只有观察阶段允许最多两次显式重试，每次使用新command ID和新观测。导航、抓取、上料不自动重派；取消保留夹持。

`succeeded`字符串不足以证明任务完成，必须同时通过该步骤的测量条件。这些回执检查不能独立认证生产者确实连接了真实传感器。

## 对现有代码的实际适配

`station_skill_adapter.py`提供三条显式连接：

1. 用实例地图位姿生成现有`PrecisionDock`目标。
2. 用料台支承面在当前base系中的高度创建现有`ImageObserver`。不注入钢筋中心或survey hint；新观测仍由现有图像模块产生。
3. 将已确认机器的几何传给现有`LoadingRoute.advance/targets`和`RobotServo.route_actions`。每次根据新鲜底盘反馈重算base系目标；新接口还检查底盘停稳、图像时效/标定、急停和测量有限性。

`LoadingRoute`新增可选`loading_geometry`。不传时保留原默认行为；传入时使用工位的预插入点、腕姿和撤离向量。进入插入阶段还需要实测腕姿与目标相差≤2°，缺失姿态时不能只靠位置到达进入。离线捕获servo请求的测试实际走了这些现有代码接口，没有创建仿真环境或应用动作。

原已验收`campaign_source`未修改；本轮修改的是工作树中的可选适配接口。`mobile_env.py`与`mobile_residual.py`未改，原冻结入口不会自动切换到新任务流程。

## 策略和运行连接边界

现有76维输入中的TCP/钢筋等已使用base系，但`base_xy`包含地图坐标，路线/阶段编码也有原技能语义。新任务的六步编号不能直接塞入PPO route向量。

因此新任务意图固定标记：

- `policy.execution_authorized=false`，SHA仍为393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82。
- `compatibility=unvalidated_new_layout`；没有改输入合同、权重或optimizer来伪装已兼容。
- `actuator_dispatch=false`；真实导航服务、设备就绪/身份信号、持续运动守卫和运行时任务执行后端尚未接通。

下一阶段应显式完成运行后端和新布局输入兼容性检查，再另行授权仿真。当前代码可编排、校验并适配数学目标，不能直接宣称新场景可以自主完成物理任务。Cosmos未进入阶段授权链。

## 离线使用

在仓库根目录使用项目Python；以下命令都不会启动仿真。

```sh
PYTHONPATH=rebar_training/src rebar_training/.venv/bin/python -m rebar_training.workcell_check plan \
  --layout rebar_training/config/multi_workcell_offline.json \
  --task-id example_delivery --source source_B --machine machine_3 \
  --report /tmp/example_delivery.plan.json

PYTHONPATH=rebar_training/src rebar_training/.venv/bin/python -m rebar_training.workcell_check validate \
  --layout rebar_training/config/multi_workcell_offline.json \
  --checkpoint rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt \
  --report /tmp/multi_workcell.validation.json
```

输出路径必须不存在。`validate`使用明确标为`synthetic_fixture`的接口假数据，验证全部2×3组合和20类反例；它不执行检测器、导航或策略。`--checkpoint`只核对SHA，不加载张量。

`replay`读取事件数组，每项为 `issued_s / received_s / result`，要求`recorded_measurement`来源；result含匹配命令身份、时间戳和步骤证据。CLI会保留拒收理由，返回非零状态。仅给假数据换一个origin标签不能把它变成真实物理验收，报告始终`physical_acceptance=false`。

## 验证入口

结构化汇总：`reports/multi_workcell_offline_20261024.json`。
完整组合、命令/回执、测试输出和前后SHA：`artifacts/multi_workcell_offline_20261024/`。

测试覆盖布局校验、60次整体平移/旋转的几何不变性、六种任务组合、错机/忙机/旧帧/错TF/失载/未卸载/阶段缺失/精度与撤离反例、任务超时/取消/有界观察重试，以及旧路由/观察/托槽恢复回归。正式物理成功率不由这些测试推算。

本轮最终离线验证：158项测试通过，六种任务组合6/6接口回放通过，20类反例20/20拒收。策略+optimizer及原视频的前后SHA一致。新布局的真实视觉、导航、碰撞/可达性和PPO兼容性仍未通过物理验收。
