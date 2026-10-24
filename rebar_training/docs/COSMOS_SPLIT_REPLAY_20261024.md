# Cosmos分视图高分辨率回放复测

仅分析同一份已完成的316.4秒仿真录像；不启动仿真、不训练、不加载现有PPO策略或optimizer。原录像与393b策略+optimizer只按SHA核对。原正式64/64和独立演示1/1不被此次视觉分析替代。

## 处理方法

原录像的腕部画中画覆盖了部分总览；这些总览像素无法从MP4恢复。按原录制代码，将总览顶部90行阶段文字去除，把画中画区域涂为缺失区域并在模型提示中解释；单独提取(876,118)起的384×230腕部画面（原384×231，去掉最后一行以满足H.264偶数高度）。不通过放大虚构腕部细节。其后发现同回合原始腕部PNG已封存，最终使用这些实际高分辨率图像，不受MP4小窗分辨率限制。

分别保存16段总览和16段腕部视频，每段20秒、末段16.4秒，4fps，不覆盖上一轮混合视图结果。总览每段进一步选取首、中、末三张真实帧，保存实际时间和PNG SHA，按时间顺序送给同一固定Cosmos3-Edge Reasoner。总览模型实际输入1024×480，腕部最初的小窗试跑为384×224；最终改用同回合已封存的31张1024×615原始mono图（模型对齐后通常1024×608），按实际时间戳分配到时间窗，首窗1张、其余通常2张，不制造额外帧。原始PNG逐SHA匹配旧双端备份清单，K/TF、概率、接触力和成功标签不输入模型。上轮混合画面仅384×160，腕部只占其中一小块。

这是按时间窗作稀疏首末对比，不能声称模型分析了全部原10fps帧或捕获了窗内每次快速开合。保留全部4fps分视图视频用于复查。

总览明确指认“搭载机械臂的白色带轮车辆”，区别于固定试验机底座；比较车辆相对桌台/机器在首、中、末帧的位置。腕部视图不具有可靠的全局底盘运动可观测性，其base_motion由输出契约限定为not_observable；这项范围限制不是模型运动识别准确率。

杆件关系分为gripper、vehicle_tray、source_support、machine、transition、uncertain。只看见夹爪靠近或手指不可辨时不接受gripper结论；明确张开/分离与持杆结论冲突时拒收。双视图不一致时保留冲突，不选一个当真值。即便两视图都指向闭指抓持，也仅输出视觉建议，不授权实际抓持或任务通过。

## 格式与复现

`rebar_training/scripts/cosmos_reasoner_split_replay.py`独立于控制器。用lm-format-enforcer0.11.3的JSON Schema约束生成，另作矛盾和可观测范围检查。只约束格式与可用分类，不把原生阶段/位置/成功标签给模型。其Transformers适配器引用已移除的HF5导入别名，因此本客户端使用LMFE核心token-data/prefix API，未修改已安装库或共享SDK。[LMFE官方接口](https://github.com/noamgat/lm-format-enforcer)

模型固定为nvidia/Cosmos3-Edge revision344d602b128d1bbdacb43b08d0a3626f46343e29；Transformers源码固定50e35cc0ca18ec19b80653690de553e6decf74f5。每次执行保存脚本/公共工具/manifest SHA、模型revision、硬件、dtype和推理身份；恢复时身份不符就拒绝混用。

在完成同版本模型文件SHA校验、已有本轮输入manifest的仓库根目录：

```sh
.cosmos_reasoner_replay/venv/bin/python rebar_training/scripts/cosmos_reasoner_split_replay.py infer \
  --output rebar_training/artifacts/cosmos_reasoner_split_replay_20261024 \
  --model-path .cosmos_reasoner_replay/models/344d602b128d1bbdacb43b08d0a3626f46343e29
```

支持CUDA或Mac MPS BF16独立推理（本机BF16矩阵预检通过），实际硬件/精度写入每次执行凭据，不能当作严格相同后端的消融试验。环境仅位于`.cosmos_reasoner_replay/venv`，不改原Isaac环境。

## 对照与边界

完整结果与错误分析见`rebar_training/artifacts/cosmos_reasoner_split_replay_20261024/analysis_zh.md`；原始逐视图输出、执行凭据、三帧、分视图视频和失败历史分别保留。

预声明重点检查160–240秒四个持续导航时间窗和120–240秒六个托槽支承时间窗；同时检查非移动时间窗中的误报。使用既有原生日志和画面作事后对照，不向模型提供答案。分别记录正确判别、误判、uncertain/拒收，不能把保留判断算识别正确。本录像是同一已知回合的调试复测，不是新独立测试回合；视图、分辨率、提示、输入采样、格式约束及后端同时改变，不能单独归因于提高分辨率。

## 独立图像位移证据

模型单独看高分辨率视频、三帧对比及BF16/FP32单图诊断后，仍不能可靠判断底盘位移。最终加入单独标识的图像测量，不声称是Cosmos自身识别能力提升：对本录像车辆青色托槽特征做角点分组、金字塔Lucas–Kanade光流、前后向误差检查、静态场景配准及仿射一致性检查。只使用已录制总览像素，没有仿真位置或原生任务标签。

颜色先验只适用于该仿真外观，不是通用机器人检测器。特征失去原外观后立即退役；不能用贴到背景的跟踪点证明静止。确实观察到的早期运动可保留为时间窗内运动证据，但不推断遮挡后的终点位置。相机变化或证据不足时保留uncertain。移动阈值12像素、静止阈值4像素及至少6点等参数保存在输出；像素位移不是实际米制位姿。光流假设和方法见[OpenCV官方说明](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html)。

Cosmos总览输入会明确收到独立测量值及其来源；其原始判断、跟踪器判断和最终报告分别保留。模型忽略或误解测量时，报告仍显示图像运动证据及分歧，不把派生测量当作大模型自发发现。该后处理仅产生离线分析，不连接控制。
