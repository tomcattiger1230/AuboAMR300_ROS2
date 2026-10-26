# Newton与Isaac Lab：本项目方案比较（2026-10-26）

本轮按用户确认仅调研与比较：没有安装Newton、修改共享SDK或依赖、启动任何物理仿真、训练或浏览器交互式模拟。同期Cosmos回放验证属于读取封存视频的独立推理任务。

Newton和Isaac Lab不处于同一层：Newton提供物理求解，Isaac Lab组织机器人资产、观测、动作、任务及学习/评估工作流。Isaac Sim则提供完整的仿真应用与渲染集成。Cosmos Reasoner提供视觉语言分析；它们可以组合，但不能互相替代验收职责。

| 组件/路径 | 本项目可能获得的价值 | 必须重新验证的部分 |
| --- | --- | --- |
| 当前Isaac Lab + Isaac Sim PhysX | 保留现有USD、相机、控制链和既有物理证据，作为参照 | 原多工位未通过组合依然未通过；Cosmos回放不能补足物理覆盖 |
| Isaac Lab + Newton MJWarp | 在任务层接口不变的前提下比较另一套GPU刚体动力学，研究批量评估 | 导入资产、关节顺序、驱动、时间步、接触与力输出；不能保证现有权重直接成功 |
| Newton VBD/MPM等 | 未来需要柔性体、材料或耦合场景时可评估 | 刚性钢筋上料不应仅因有新求解器就切换；具体功能和诊断可能仍为实验性 |
| Cosmos + 当前仿真 | 观察操作阶段、描述异常、生成供人复核的语义候选 | 不代替物理引擎、力测量、定位或真实成功回执 |

最新版本需要区分独立项目和集成的兼容组合。截至本轮查询，Newton最新正式版是[v1.7.0，10月14日发布](https://github.com/newton-physics/newton/releases/tag/v1.7.0)；Isaac Lab最新正式版是[v3.0.0，10月20日发布](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0)，已不是只提供旧实验分支的状态。Isaac Lab该版本面向的组合包括Isaac Sim6.2、Python3.12、Torch2.11/CUDA13、Warp1.17及Newton1.6.1。独立Newton最新版不等于该Lab版本推荐的依赖，后续应按锁文件选组合，而非直接混装最新版。

Newton基于Warp，并提供MuJoCo Warp等求解路径；Isaac Lab3.0允许按任务支持情况选择Newton或PhysX，并将物理、渲染和可视化分开。Kit-less可以减少完整Isaac Sim依赖，但不会自动提供现有RTX腕部图像的相同分布。[Newton说明](https://github.com/newton-physics/newton/tree/v1.7.0)、[Isaac Lab Newton后端](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/concepts/physics-backends/newton.html)

对当前代码做了只读核查：`isaac_cfg.py`明确使用`isaaclab_physx.physics.PhysxCfg`，不是Newton；目标共享Lab目录的VERSION为3.0.0，但该字符串不能证明它与新发布tag、完整依赖锁或未修改的官方源码一致。没有导入仿真模块验证版本，也没有更改它。

官方跨引擎指南强调：同一checkpoint可以用于sim-to-sim评估，但观测/动作顺序、尺度、坐标、归一化、状态复位和控制周期必须对齐，动力学轨迹仍可能不同。夹爪mimic关系在PhysX与MJWarp中的驱动图也可能不同。[跨引擎迁移指南](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/how-to/transfer_policies_between_physx_and_newton.html)

这对本项目尤其重要：

- 现有四夹垫受力、打开后卸载、钢筋速度和无臂接触等评分依赖真实传感/接触数据。换后端后，要先核对力的方向、法向/摩擦/合力含义、聚合窗口、采样频率与接触过滤，不能沿用数值门槛便宣称等价。
- 关节命名、排列、主从驱动、轮速接口、PD参数、质量惯量和碰撞体需要分别核对；相同USD文件不证明两套导入结果相同。
- 已冻结策略和optimizer应只做身份核对与冻结推理。物理后端比较应单列新证据，不能覆盖或混入原正式64/64、演示1/1及多工位3/6记录。
- 相机内外参、图像编码、腕部视角、时序与渲染材质也要对齐，否则视觉变化与物理变化会相互混淆。

我的建议是保留当前PhysX环境作为参照。后续若决定评估Newton，应在独立目录和环境中，先做资产/接口静态审计，再在获得仿真授权后进行短时固定控制与冻结策略对照；只有这些证据支持时，再考虑更多任务或训练。Newton本身不会修复Cosmos的对象误认，也没有依据认定切换物理引擎能解决此前测试机前的导航逻辑问题。

本轮不执行以上候选方案。源码、当前权重、optimizer、安装环境和既有验收状态均保持。
