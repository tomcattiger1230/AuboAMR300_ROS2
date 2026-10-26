# Cosmos回放模型迁移至4090（2026-10-26）

目标arnold@192.168.3.133（arnold-TRX50-AERO-D），根目录/home/arnold/Develop/rebar-deployment-4090-20261026。不依赖极空间；不启动仿真、训练或机器人控制。

## 已完成

- 固定nvidia/Cosmos3-Edge revision344d602b128d1bbdacb43b08d0a3626f46343e29，16文件7735509866字节从Mac直传4090并在节点逐SHA核验。仅部署Transformers Reasoner接口，不运行Generator。
- Transformers固定源码commit50e35cc0ca18ec19b80653690de553e6decf74f5已从原已安装纯Python源重建平台无关wheel，源码Cosmos目录8文件SHA记录在transformers_wheel.source_verified.json。
- 目标已安装独立Python3.13.15并建立.cosmos_reasoner_replay/venv。目标访问官方PyPI/PyTorch超时，保留原失败日志；改为Mac从官方源准备Linux x86_64 CPython3.13依赖，Torch2.10.0+cu130、Torchvision0.25.0+cu130、Transformers5.21.0.dev0及固定回放依赖共21948文件5370093498字节，Mac逐SHA核对完成。
- 原回放/分视图/33项诊断材料和28项回放单元测试保留；Mac相关28项测试通过。诊断客户端增加显式--device cuda|mps|cpu，原Mac默认mps保持，代码commit5eb62fc只改变离线Reasoner脚本，钢筋控制源码无变化。
- Mac独立APFS克隆备份：/Users/weiluo/Develop/backups/cosmos_4090_migration_20261026_5eb62fc，模型16份及Linux依赖21948份全部SHA通过，不使用硬链接。原模型与Linux二进制均不可作为Mac推理环境混用；备份中未包含目标Linux Python解释器，恢复时使用已记录的Python3.13.15。

## 当前阻碍和未完成项

Linux依赖直传中Mac到4090的SSH出现Network is unreachable/Operation timed out。只读路由当前显示192.168.3.133经172.20.10.1，已保留所有续传文件，没有更改网络、电源、SSH身份，也未重启或重派任何仿真。

目标独立环境依赖仍待续传和完整校验；CUDA模型加载、实际回放及输出完整性均未核验，不能把已到达的权重当作部署验收通过。当前模型语义可靠性仍为未通过；不以解析成功或回答减少不确定替代准确性。钢筋控制部署仍保持已验证状态，393b策略+optimizer及7d85原视频SHA保持。

## 网络恢复后的接力步骤

1. 先有界SSH核验hostname和已有setup状态，确认没有已启动的Reasoner验证作业。不得重复创建Python环境或重跑原控制任务。
2. 续传Mac artifacts/cosmos_4090_migration_20261026/linux_site_packages/ 到目标.cosmos_reasoner_replay/venv/lib/python3.13/site-packages/，使用rsync -rlpt --partial --append，排除只限根目录/bin/及__pycache__/。必须保留嵌套torch/bin和triton/backends/nvidia/bin的可执行文件，8份嵌套工具已独立传过；仍以全部manifest核验为准。
3. 传输linux_environment_manifest.json、cuda_case_plan.json及模型source_verified proof到setup，逐文件比较Mac与目标全部21948项、模型16项。保留setup/install.status.json和torch.log的PyPI初始失败历史，独立记录offline恢复，不抹除失败。
4. 在新venv内检查Torch2.10 CUDA、Torchvision、PyAV19、OpenCV、Transformers类及8份源码SHA；用uv pip check --python ...venv/bin/python检查依赖，不改共享钢筋环境。
5. 创建唯一输出目录（如artifacts/cosmos_4090_validation_20261026）和只启动一次的.launch.json。GPU有足够空间时运行venv/bin/python rebar_training/scripts/diagnose_cosmos_replay.py --device cuda --dtype bfloat16 --attention sdpa --source rebar_training/artifacts/cosmos_reasoner_split_replay_20261024 --output 新目录 --cases setup/cuda_case_plan.json。预声明3项：原总览200–220秒ROI稀疏12帧视频、原高分辨率腕部图像、中性提示的人工空白对照。无跟踪器值、原生位置、接触力或成功标签输入模型。
6. 记录loading_info、所有原始token/分数检查、EOS、未知token、时限及回答。数值异常和语义不确定分开；没有完整EOS或存在NaN的输出不计完成。BF16如失败须保存原结果，再另预声明FP32尝试，不能覆写或挑成功。
7. 重新核对策略/optimizer、原视频和模型全部SHA，回传原始结果、环境freeze和验证凭据至Mac；随后才更新migration_complete和目标DEPLOYMENT_STATE的cosmos字段。

迁移与回放范围仅为离线分析；本轮没有部署HTTP/NIM服务或控制接口，也没有执行新的物理回合。
