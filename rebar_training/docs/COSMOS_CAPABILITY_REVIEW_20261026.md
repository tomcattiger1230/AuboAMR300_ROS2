# Cosmos能力边界与NVIDIA示例调研（2026-10-26）

本轮只调研官方资料并读取已有回放记录，没有运行新的模型推理、仿真、训练、安装或权重更新。冻结policy与optimizer保持。资料核查截至2026-10-26；当前官方示例主要在NVIDIA/cosmos-framework的`.agents/skills/cookbooks/cosmos3/`，本次读取文件更新时间为10月21日。旧NVIDIA/cosmos/cookbooks路径已不可用，不能把旧搜索结果当作现行入口。

目前适合讨论的定位是：Cosmos Reasoner承担视觉语义理解、事件解释与候选任务分解；对本项目的工位识别、持杆状态和底盘运动，必须先建立独立的可重复准确性证据。部署成功、JSON有效或一段合理文字，均不能直接作为物理成功凭据。

| 能力层 | 官方提供的能力 | 对当前钢筋任务的边界 |
| --- | --- | --- |
| Reasoner | 图像/视频问答、事件时间定位、视觉对象定位、语言任务规划 | 当前已部署的接口；可研究“看见什么、何时发生、下一步候选动作”，并不提供接触力或毫米精度保证 |
| Generator | 条件视频/世界演化与合成数据 | 与当前文本Reasoner接口分开；逼真的生成视频不能替代原生物理证据 |
| Policy-DROID | 专门训练的机器人动作策略 | 官方模型卡限定DROID的8维动作；不能直接输出AUBO+移动底盘可执行控制 |
| Nano-Sim-Bimanual | 动作与相机控制条件下的交互式视频模拟 | 新发布的专用模型，当前支持AgiBot；不是本项目现成的可用环境 |

这一区分依据[NVIDIA模型参考](https://docs.nvidia.com/cosmos/latest/cosmos3/model_reference.html)、[Policy-DROID模型卡](https://huggingface.co/nvidia/Cosmos3-Edge-Policy-DROID)和[Sim-Bimanual模型卡](https://huggingface.co/nvidia/Cosmos3-Nano-Sim-Bimanual)。后者于10月20日发布，是专门后训练、因果蒸馏的16B模型。其能力不能由通用Edge Reasoner自动继承，也不能假定换一份工位坐标便可迁移机器人本体。

官方模型卡明确说明对象状态、空间关系和时序可能推断错误，分布外场景会影响质量。因此“物体看似接触”与“确实形成稳定夹持”需要分开；单凭普通视频不能获得本任务实际四夹垫受力、钢筋相对滑动阈值或夹紧误差的验收证据。我们的夹具、灰白渲染与腕部视角可能构成领域差异，这是待验证假设，尚未证明是唯一根因。[官方能力局限](https://huggingface.co/nvidia/Cosmos3-Edge#limitations)

本次重点查看的官方示例及可借鉴内容：

- [Transformers notebook](https://github.com/NVIDIA/cosmos-framework/blob/main/.agents/skills/cookbooks/cosmos3/reasoner/run_with_transformers.ipynb)：Edge使用AutoModelForImageTextToText，BF16，只加载Reasoner。我们的实际类与此一致；官方也提供贪心生成示例，所以不能将现有do_sample=False本身判定为接入错误。视频示例使用2fps。
- [Reasoner提示指南](https://github.com/NVIDIA/cosmos-framework/blob/main/.agents/skills/cookbooks/cosmos3/reasoner/reasoner_prompt_guide.md)：示例包括对象框、事件时间段与机器人下一步建议；2D坐标按每个轴归一化到0–1000。文中展示答案明确基于Super，视频配置使用4fps，不能把这些展示答案当成Edge的已验证能力。
- [NIM任务目录与验证说明](https://github.com/NVIDIA/cosmos-framework/blob/main/.agents/skills/cookbooks/cosmos3/nim/reasoning.md)：18项样例覆盖描述、时序、空间和任务推理；验证基线是Super BF16、无视频剪枝，格式检查不代表语义正确。语言轨迹只是视觉坐标中的建议，不能直接发送机器人。
- [Streaming Video](https://github.com/NVIDIA/cosmos-framework/blob/main/.agents/skills/cookbooks/cosmos3/nim/streaming.md)：支持逐帧会话和有界历史，可作为后续在线观察接口参考。Edge需要相应配置且没有DFlash；这是一条需另行部署验证的路径，本轮没有启动服务。流式接口也不等于持续正确跟踪或实时控制。
- [最新Cookbook入口](https://github.com/NVIDIA/cosmos/blob/main/COOKBOOK.md)：把推理、动作策略、世界模拟和后训练分别列出。10月的新交互式世界模型属于另一个方向，不是当前回放误判的现成补丁。

结合现有记录，需要纠正或保留的判断如下。

| 已核对事实 | 可以得出的结论 | 还不能得出的结论 |
| --- | --- | --- |
| 固定Edge权重完整，CUDA加载无missing/unexpected/mismatched/error；三项输出数值有效、EOS完整 | 当前迁移与基本推理链路可用 | 模型语义已经可靠，或所有“不确定”均已定位 |
| 总览20秒只提供12帧，实际约0.6fps；官方常见样例2–4fps | 当前时序条件与官方示例不同，需要控制采样差异 | 稀疏采样一定是底盘漏报的唯一原因 |
| 腕部仍将夹具描述为书架/书本 | 对象身份识别这一前提尚未稳定 | 继续追问接触/持杆便能得到可信结论 |
| 旧FP32+ROI+思考组合曾识别平移；新CUDA迁移诊断关闭思考，只得到简短回答 | 两批不是后端完全对齐的准确率对照 | CUDA比MPS差，或BF16一定导致语义错误 |
| 当前使用官方支持的Transformers Reasoner路线 | 不必仅为“看起来官方”立刻替换后端 | 换成NIM/vLLM会自动改善同一权重的理解能力 |

当前固定config中的max_position_embeddings为131072；模型卡另有长上下文宣传值。应以固定权重、处理器、服务限制与实际显存实测为准。生成侧的256p/480p、帧数或灰度限制，也不能直接当成Reasoner的输入限制。

资源方面，官方当前NIM配置对Edge BF16列出12GiB下限；Nano列出23.1GiB下限，离散GPU还检查4GiB运行预留。我们的24GB 4090不能因为“约16–17GB的Reasoner模型内存”便被宣布满足Nano NIM部署要求。原生Transformers的内存组成不同，短输入Nano对照是否可行需另测，不能套用NIM下限或承诺必定装得下。[后端内存说明](https://github.com/NVIDIA/cosmos-framework/blob/main/.agents/skills/cookbooks/cosmos3/reasoner/README.md)、[NIM支持矩阵](https://github.com/NVIDIA/cosmos-framework/blob/main/.agents/skills/cookbooks/cosmos3/nim/support-matrix.md#reasoner-profiles)

供下一轮讨论的顺序，尚未执行：

1. 先在固定Edge上复现少量官方原始图片/视频与提示，保留真实输出。验证“官方熟悉场景能否基本看懂”，不要拿Super展示答案当逐字标准答案。
2. 再做原钢筋视频的小型人工核对集：先定位夹具/杆，再判断底盘相对固定设备的位移和事件时间。总览可用更短片段保留2–4fps，同时守住现有内存预算；腕部先验证对象定位，避免在错误物体身份上继续推理。包含静止、空手、遮挡和空白负例，不将拒答一律计作错误。
3. 对同一权重逐项改变采样、提示、思考设置，保留全部结果。仅在基线和资源条件明确后，再讨论冻结Nano或Super参照；本轮不下载新模型。
4. 达到何种准确性后进入任务层，需要共同确定。候选指标是对象框/身份准确率、运动误报漏报、事件时间误差和不可见情形的拒答质量；稳定夹持、力和毫米精度继续由物理/传感证据验收。独立跟踪可用于事后对照评分，但不向Cosmos提供标准答案，也不替它填补漏报。

对“两料区、三台同型测试机”的目标，我的工程判断是：设备外观相同就不能仅靠图像名称保证区分机号。机号应与任务ID、地图工位和定位关联，Cosmos可研究设备/障碍语义、操作阶段与异常解释。指定机号的唯一绑定、标定坐标和夹紧验收不应由一段自然语言回答代替。这是针对本项目的方案判断，不是官方已验证的AUBO案例。

暂不建议据现有三项诊断就开始微调，或改用生成世界模型替代当前物理仿真。先通过上述冻结推理对照区分接入条件、模型规模和领域差异，再讨论是否需要训练以及训练目标。本次调研没有修改迁移完成状态、现有策略、optimizer或控制源码。
