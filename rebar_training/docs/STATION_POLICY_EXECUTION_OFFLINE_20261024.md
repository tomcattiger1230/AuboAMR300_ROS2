# 新布局策略输入适配与导航/任务执行接口：离线候选

本轮只做代码、CPU冻结前向和离线接口验证，没有启动仿真、ROS、导航服务或硬件连接。没有重新训练、创建/加载optimizer实例或保存策略。393b策略及optimizer文件完整SHA、52个张量及466 updates /170 actor updates /36602 policy steps均保持。

## 策略输入如何适配

内部网络仍使用原`mobile_visual_manipulation_residual_v1`：76维、原字段顺序/缩放、原16阶段one-hot、11维route编码、6维机械臂残差、0.08混合系数与原精对准增益。没有把六步任务编号送入网络，也没有给网络增加工位编号或新特征。

新增外层合同`station_relative_legacy_context_v1`，与冻结权重合同分开版本管理：

1. TCP、姿态、钢筋估计及速度已经在base系，不再重复旋转；关节、夹爪、载荷保留实际测量值。
2. `base_xy`通过`参考工位变换 × 实际工位逆变换 × 实测底盘位姿`映射回训练参考坐标。源参考系原点为[-1.15,0,0]，测试机参考系原点为[6,4,0]，偏航均为0。
3. 上料阶段的机器目标使用指定机器的精定位结果，转入实际base系。导航也始终使用指定机器的实际站点ID。
4. **取料/放槽阶段的`machine_goal_minus_tcp`被明确改成训练参考目的机的策略上下文。** 这是为了让独立局部取料技能不因未来配送目的地改变而改变输入分布，不是宣称该值仍是新场景真实机器的位置。实际目标与策略上下文在`CompatibleInput`中分开保存；上下文坐标不得传给导航或伺服。该语义适配需要后续物理验证，不能只凭同维度宣称泛化完成。
5. 拒绝原控制器不可达的阶段/route组合：pickup与pickup_tray为0..3，托槽路径阶段和tray_lift_high为4，loading为4/6/7/8/9/10。导航、观察、对接不调用残差策略。
6. 保留有效视觉、恢复状态、motion-ready及抓取初始对准的原活动掩码。不活动时不运行actor、不叠加残差；最终只改7维动作的前6维，夹爪保持局部技能输出。

`sample_from_legacy_control`从现有控制器的关节、FK、图像估计、载荷与底盘反馈采样，不读钢筋真值或evaluation字段。旧相对时钟通过固定会话offset映射到执行器时钟；不逐帧重写时间戳来掩盖过期。相机/机器人反馈、layout/map、任务/command身份必须匹配；底盘须停稳。当前合同只支持平面底盘，明显roll/pitch会拒绝，不悄悄压平成yaw。

`FrozenStationResidual`只载入原模型的CPU副本并做eval/inference-mode前向，冻结梯度。没有`update/save/record`学习接口，没有optimizer对象；文件及整个模型/optimizer张量payload前后复核。

## 导航与任务接口

`TaskExecutor`把原六步`MissionFlow`接到两个端口：

- `SeerNavigationPort`：按当前仓库AgvClient契约发送3051路径导航，读取1020导航状态与1110任务详情，使用3003取消。客户端和传感器均为显式注入，默认仅接受offline端口。`SeerClientTransportAdapter`可转换现有ApiResult返回值，不创建AgvClient或连接socket。
- `SkillExecutionPort → LegacyControlDriver → 注入的局部会话`：创建源观察器、采样原控制器数据、运行冻结输入适配和actor、调用原LoadingRoute/RobotServo数学入口，记录有界混合动作，消费独立技能完成回执。新接口不调用env.reset/env.step，不创建仿真环境；CPU验证中的动作只记录，没有应用到设备。

每条command只start一次，重复时间tick返回缓存，不重复下发；ACK只算接受，不能直接完成导航。到站还须检查精确request/target归属、地图/布局、实测位置/停止状态与承载条件。运输守卫持续检查，技能期间底盘移动、视觉过期或跨帧反馈会保持并阻止前向/阶段推进。任务完成回执不再触发一次额外残差输出。

现有SEER取消API是全局操作，没有任务ID参数。取消前重新查询并核对归属；归属不明不发送全局取消，已终止任务也不重复取消。无法确认/取消时记录未确认状态，不将“请求停止”当成物理停止证明。异常不自动重派导航或上料。观察失败可通过`TaskExecutor.retry_observation`显式申请最多两次新command/新会话重试，不复用旧帧，也不重发已完成导航。

**实际SEER回执的request/target字段布局尚未通过设备抓包验证。** `SeerStatusFields`要求显式字段路径，没有默认猜测或缺失ID回退；离线CLI使用带fixture前缀的样例字段。真实服务、实机/仿真会话工厂、碰撞与可达性、机器身份/就绪信号仍需单独接入验证，当前没有开放live dispatch。

## 离线证据的范围

- 六条料区—测试机组合经完整端口链通过，调用真实冻结actor和原局部计算接口；导航/技能测量与完成回执仍是显式synthetic fixture，不是真实行驶/识别成功率。
- 138组有效阶段/route状态比较独立工位换位、转向后的输入/actor输出；额外检查整体地图平移/旋转。
- 92组状态直接隔离执行未改的`MobileControl.learning_observation`函数，核对字段、活动掩码、冻结前向及应用后的残差。仅编译该函数AST，不导入Isaac环境模块，并固定原源文件SHA。
- 原演示318行日志只做白名单测量字段与阶段/route审计；它缺少重建完整76维所需的dq、部分速度、逐行底盘偏航与四爪载荷，不补零后冒充真实策略回放，也不把评分真值送入模型。
- 失载、旧图像、错误导航归属、导航失败等反例经相同端口链验证保持/不重派，物理停止本身未被验证。

这些证明输入适配和接口在离线条件下相容，不证明新布局物理任务成功。原64/64与演示1/1保留旧场景范围，不拼入本次统计。

## 运行

```sh
PYTHONPATH=rebar_training/src rebar_training/.venv/bin/python -m rebar_training.offline_execution_check \
  --layout rebar_training/config/multi_workcell_offline.json \
  --checkpoint rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt \
  --native-report rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/demo_holdout_9321042.json \
  --report /tmp/station_policy_execution.offline.json
```

输出路径必须不存在。`--help`不会载入torch、actor或模拟器。没有执行/训练/启动仿真的CLI开关。

汇总：`reports/station_policy_execution_offline_20261024.json`。完整离线请求、回执、参考输入对照、动作、测试和SHA证据：`artifacts/station_policy_execution_offline_20261024/`。

最终结果：219项测试通过；6/6离线端口任务链、4/4故障链通过；138组有效阶段/route在工位变换后的actor输出最大差0。92组原编码函数对照的输入最大差5.96046448e-8，actor最大差4.47034836e-8，应用后归一化机械臂残差最大差3.72529030e-9；活动掩码一致。52张量和466/170/36602计数及文件SHA保持，未创建optimizer。
