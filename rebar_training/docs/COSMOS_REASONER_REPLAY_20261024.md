# Cosmos Reasoner 离线录像分析接入

本次仅分析既有完整演示录像，不启动Isaac/仿真、ROS或训练。现有策略和optimizer文件只校验SHA，不加载到Reasoner，不更新或保存。

## 接入实现

- 客户端：`rebar_training/scripts/cosmos_reasoner_replay.py`，包含prepare与infer两个独立入口。
- 模型：NVIDIA `nvidia/Cosmos3-Edge`，固定revision `344d602b128d1bbdacb43b08d0a3626f46343e29`；仅加载Reasoner条件生成类，不运行Generator。
- 独立环境：103节点 `.cosmos_reasoner_replay/venv`；Python3.13.15、PyTorch2.10.0+cu130、Transformers5.21.0.dev0，源码固定commit `50e35cc0ca18ec19b80653690de553e6decf74f5`。原Isaac环境没有修改。
- 推理：BF16、SDPA、eval模式、禁用梯度、greedy生成；输出为建议性文字/JSON，无控制连接。
- 输入：原316.4秒、10fps录像，SHA `7d85144259e009b40d14c1fc7924c1718caee48264b6792938f0eb1a0703f2ef`。
- 预处理：连续16段，每段20秒、末段16.4秒；4fps抽样；裁掉顶部90像素阶段/时间/说明文字，保留总览和腕部画中画。原视频不改动。
- 资源预算：每帧约384×160，80帧/20秒；部分遮挡、细小夹具/手指/钢筋接触信息可能无法识别。覆盖完整时长不等于分析每个10fps原始帧。

## 运行方式

在103隔离副本仓库根目录，已有本次manifest及校验过的固定模型后：

```sh
.cosmos_reasoner_replay/venv/bin/python rebar_training/scripts/cosmos_reasoner_replay.py infer \
  --output rebar_training/artifacts/cosmos_reasoner_replay_20261024 \
  --model-path .cosmos_reasoner_replay/models/344d602b128d1bbdacb43b08d0a3626f46343e29
```

脚本逐次核对原录像、策略及optimizer文件、模型清单、片段SHA。已有同身份响应会跳过；原始响应、提示词、时间、显存、token数全部保留。输出不符合JSON契约时明确标记schema_valid=false，不补造事件或把解析成功当成内容正确。

视频使用显式PyAV解码，保留4fps帧索引与时间元数据，避开torchvision与PyAV19接口不兼容的问题。初次烟测的错误JSON单独保留在attempts/initial_smoke；完整批次使用增加JSON示例的提示词，仍保留全部格式/内容问题，不筛选成功片段。

## 结果和边界

本次实际结果、模型原话、录像复核和两端SHA清单见 `rebar_training/artifacts/cosmos_reasoner_replay_20261024/analysis_zh.md`。

本客户端适用于离线回放分析。尚未部署HTTP/NIM服务、在线影子模式、实时监督或动作白名单。没有进行视觉扩增、策略微调或optimizer操作。Reasoner只能提示画面中的语义和可疑现象；毫米精度、载荷、真实接触、传感器有效性和最终任务通过仍依据原生独立验收。本次不会改变已经通过的正式64/64。

官方依据：[模型卡](https://huggingface.co/nvidia/Cosmos3-Edge)、[Reasoner服务接口](https://docs.nvidia.com/nim/cosmos/4.0.0/reasoning.html)。本次使用本地Transformers推理，未调用NIM。
