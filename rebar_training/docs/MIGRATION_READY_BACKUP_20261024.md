# 当前项目迁移备份与恢复

本次备份为已完成任务的只读快照：正式整链64/64、独立演示1/1；Cosmos离线诊断33项，语义可靠性未通过。策略及optimizer保持393b47f8…，466/170/36602计数不变。恢复文件不会启动训练、仿真、ROS、旧调度器或已删除的automation。

## 备份入口

本机备份目录：`/Users/weiluo/Develop/backups/rebar_migration_20261024_142019/`。

- `rebar_migration.tar`：自包含内容去重归档。每份不同内容只存一次，完整清单保留每个文件的路径、SHA、大小、权限、修改时间、目录和符号链接。
- `source_manifest.json`：完整路径到内容对象的映射；`source_stability.json`检查打包期间源文件是否变化。
- `rebar_migration.tar.sha256`、`rebar_migration.verify.json`：整包SHA和完整对象/路径映射复核。**只有verify凭据存在且mismatches为空才算备份完成。**
- `migration_backup.py`：仅用Python标准库的校验与恢复工具，另以归档内的workspace版本复核工具SHA。
- `metadata/`：完整Git bundle、工作区/暂存差异、分支/提交信息、环境版本和依赖清单、迁移说明。

归档覆盖整个当前仓库工作树（包括未提交改动），全部rebar原始数据、失败/中断/历史归档、正式证据、冻结模型与optimizer、资产/纹理/配置/种子计划、所有Cosmos结果及固定模型分片、三套现有虚拟环境与对应Mac Python运行时。`.git`由独立的全refs Git bundle替代；跳过可再生的`__pycache__`、`.pyc`、测试/lint缓存和`.DS_Store`，完整排除列表随包保存。没有删除历史数据或用新结果覆盖旧结果。

当前仅检测到系统盘，没有已挂载的外置备份盘；103本次有界SSH重试超时。此处是可直接带走的本机迁移副本，**不是独立磁盘容灾副本，也不宣称本次已同步103**。将整个备份目录复制到外置盘/NAS/新主机后，在目标重新计算整包SHA并运行verify。

## 校验和恢复：先验证、后进入新目录

在备份目录运行，Python不需要安装项目依赖：

```sh
shasum -a 256 -c rebar_migration.tar.sha256
python3 migration_backup.py verify --archive rebar_migration.tar
python3 migration_backup.py restore --archive rebar_migration.tar --destination /新磁盘/rebar-restored
```

目标目录必须不存在，工具拒绝覆盖。恢复时逐对象校验SHA，并把重复内容复制为独立文件；默认不使用可能相互污染证据的硬链接。完整恢复所需空间按manifest的source_bytes估算，另留Git和系统余量。归档本身较小不表示恢复树同样小。

可先恢复关键内容（每个`--prefix`都对应归档内路径），其余原始历史继续完整留在归档中：

```sh
python3 migration_backup.py restore --archive rebar_migration.tar \
  --destination /新磁盘/critical-check \
  --prefix workspace/rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt \
  --prefix workspace/rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/full_policy_demo.mp4 \
  --prefix workspace/rebar_training/docs \
  --prefix metadata
```

Git历史在恢复目录的`metadata/repository.bundle`中。可先执行`git bundle verify`；随后在已恢复的workspace中初始化Git、fetch bundle的heads/tags，再按`metadata/git_state.json`设置HEAD分支，执行`git reset --mixed HEAD`重建索引，保留恢复的工作区文件。需要重建原暂存状态时，使用随包`git_staged.patch`执行`git apply --cached`；不要用`reset --hard`覆盖未提交工作。

## 目标主机的环境边界

数据、源码与固定模型跨主机可恢复；现有Mac虚拟环境、Mach-O Python二进制和绝对路径符号链接不能直接作为Linux/CUDA环境运行。随包保留它们用于追溯；在新主机按目标平台创建隔离环境，参考三份freeze、`uv.lock`、Transformers固定commit和已封存节点runtime_versions文件。Cosmos revision为344d602b…；重建后先核对全部16份固定文件SHA，不从main悄悄换权重。Mac诊断使用FP32和数值/EOS/prefill预算检查；CUDA本轮未重新验证。

Isaac Sim、IsaacLab共享SDK、GPU驱动/CUDA、操作系统库及Homebrew动态库不包含在本项目归档内；版本和路径证据随包保留，新Linux/NVIDIA主机需另外安装匹配版本。103不可达期间不能声称已备份其本轮不可见的新文件。已有本机保存、此前双端SHA通过的运行资产、源码和历史包全部纳入，未经授权不启动迁移回归或重跑正式64。

迁移后至少只读核对：393b策略+optimizer、7d8514原录像、冻结8a24控制源码、845f检测器、d543资产、cd2f视觉配置、e738计划，以及正式full_gate和所有原始备份proof。Cosmos分析不能替代物理验收，也不授权控制输出。SSH身份、账户凭据和全局网络/电源配置不属于迁移恢复动作。
