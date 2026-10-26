# Cosmos回放模型迁移至4090（2026-10-26）

已恢复连接并完成迁移验收。目标为`arnold@192.168.3.133`（`arnold-TRX50-AERO-D`），独立目录`/home/arnold/Develop/rebar-deployment-4090-20261026`。本轮仅运行原视频离线Reasoner，不启动仿真、训练、机器人控制或Generator；不依赖NAS。

## 已验证的环境与身份

- 固定`nvidia/Cosmos3-Edge` revision `344d602b128d1bbdacb43b08d0a3626f46343e29`，16文件7,735,509,866字节，Mac与目标完整SHA映射一致。回放前后重新全量核对模型和21,948份Linux依赖（5,370,093,498字节），零差异。
- 独立Python3.13.15、Torch2.10.0+cu130、Torchvision0.25.0+cu130、Transformers5.21.0.dev0。Transformers源码固定commit `50e35cc0ca18ec19b80653690de553e6decf74f5`，8份Cosmos源码SHA一致；`uv pip check`通过，目标28项回放单元测试通过。
- `Cosmos3EdgeForConditionalGeneration`实际在RTX4090以BF16/SDPA加载，2,435,616,496参数。missing/unexpected/mismatched/error四项均为空；只加载Reasoner接口。
- 原`frozen_policy.pt`包含policy+optimizer，SHA仍为`393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82`。本轮Cosmos未加载policy或optimizer，也未创建学习更新。原316.4秒完整视频SHA仍为`7d85144259e009b40d14c1fc7924c1718caee48264b6792938f0eb1a0703f2ef`。

目标入口为`rebar_training/scripts/cosmos_python_4090`，使用独立venv并强制离线读取。调用相对路径前先进入部署根目录。已完成的输出目录不得复用，也不应重派本次验证。

## CUDA回放结果及局限

唯一验证作业自然exit0，三项均保留原始输入、完整token、分数审计、加载信息、固定身份前后凭据和日志。19份输出已回传Mac，逐SHA一致；验证结束后GPU计算进程为空。

| 预声明项目 | 输出完整性 | 语义结果 |
| --- | --- | --- |
| 总览200–220秒，ROI 410×300，原80帧均匀取12帧 | EOS完整、7 tokens、0.398秒、无NaN/未知token | 仅回答“The robotic arm is moving.”，遗漏相对固定设备可见的底盘位移 |
| 同区间原腕部1024×615，两帧 | EOS完整、98 tokens、0.848秒、无NaN/未知token | 将夹具描述为书架/书本；不能作为可靠持杆/接触识别 |
| 人工均匀灰色空白对照，两帧 | EOS完整、229 tokens、1.889秒、无NaN/未知token | 未虚构夹爪或钢筋，但将灰色画面描述为矩形物体；不构成任务语义验收 |

因此`migration_complete=true`和`runtime_integrity_passed=true`只代表部署与推理链路通过；`semantic_acceptance_passed=false`保持。没有输入跟踪器数值、原生位置、接触力或成功标签，也没有用跟踪器兜底替代模型回答。这三项是迁移诊断，不是重跑完整视频的语义验收。

本轮结果：`rebar_training/artifacts/cosmos_4090_validation_20261026/`。完整逐SHA凭据、语义复核、环境freeze和源/目标文件映射：`rebar_training/artifacts/cosmos_4090_migration_20261026/`。保留原Mac的33项诊断和历史资源/数值失败，不能将本次CUDA通过归纳为历史不确定问题均已解决。

## 本机迁移备份及恢复

Mac独立APFS克隆备份为`/Users/weiluo/Develop/backups/cosmos_4090_migration_20261026_5eb62fc/`。模型16文件、Linux依赖21,948文件的完整原备份保持；新增`recovery_20261026/`保存所有CUDA输出、目标setup及恢复材料，74文件79,337,412字节独立比SHA通过，无硬链接。最新元数据和Git bundle另有完成凭据，旧网络中断状态作为历史保留。

补齐了独立Linux Python解释器归档`recovery_20261026/target_setup/python-3.13.15-linux-x86_64.tar.gz`（36,616,304字节，SHA `a2e07f94a758dfcfa6fee8739a868fdb3124889bfe6eebeffbb1a13c92e695e0`），Mac逐条核验4,738份文件/符号链接及外层SHA。恢复不再依赖联网下载解释器；这些Linux二进制不能用作Mac推理环境。

在另一台兼容Linux x86_64主机的空目录恢复时：先按核心部署备份恢复仓库与原视频/策略，再解压上述解释器；使用其`bin/python3.13 -m venv --without-pip`创建目标独立venv，把备份`linux_site_packages/`复制到新venv的`lib/python3.13/site-packages/`（只排除根目录`/bin/`和`__pycache__/`，保留Torch/Triton嵌套bin工具），复制`models/<revision>`到`.cosmos_reasoner_replay/models/`。不要将旧venv的绝对Python链接直接带到不同路径。然后按保存的manifest全量校验、检查GPU驱动兼容性和导入，不覆盖共享钢筋运行时。当前133已安装好的环境不需要再创建。

核心控制迁移包与Cosmos备份分开保存；总索引为`/Users/weiluo/Develop/backups/REBAR_BACKUP_INDEX_20261025.json`。原控制物理覆盖3/6、旧正式64/64和演示1/1的既有范围保持，未运行新的4090物理回合。

## 保留的恢复历史

目标最初访问官方PyPI/PyTorch超时，原`setup/install.status.json`的failed记录和`torch.log`保持。随后Mac从官方源构建目标Linux依赖，SSH中断后通过partial续传恢复；没有更改网络、电源、SSH身份或共享SDK。恢复成功使用独立新凭据记录，不抹除原失败。
