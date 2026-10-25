# 2026-10-25复测结果与迁移入口

本轮只读取已结束的103作业并补备份，没有启动仿真、训练或重试。A→测试机3的完整回合在Mac/103独立审核通过：报告SHA `8ac32b8f9c7dcdf5ec10c064959e6065545f6700f1df80993ee844cf21df01e4`，上料误差1.049mm、退让85.602mm、零异常臂接触；一次实际恢复85.6秒，居中复核0.718mm。与此前A→1/A→2保留成功合计物理候选覆盖3/6。B→1在源导航结束前发生回执空状态访问异常，没有完整回合；B→2/B→3未启动。机器视觉身份和完整MissionFlow验收仍待完成，旧正式64/64单独保留。

## 需要带走的两个部分

1. 完整基础目录：`/Users/weiluo/Develop/backups/rebar_migration_20261024_142019/`。必须复制整个目录（尤其objects/及source_manifest.json），不能只复制README。原始完整备份已逐SHA核验：169862文件，78318独立对象，唯一内容84441863479字节，实际关键恢复147文件、6项工具测试通过。基础清单SHA为`f6e7371f025934a75d2f3ec0dc2c0180ff8fb61dbeb4b4c10c92da1dde65b9f5`。
2. 本轮汇总增量：`/Users/weiluo/Develop/backups/rebar_migration_followup_20261025/`及同目录旁的`rebar_migration_followup_20261025.tar.gz`。它包含当前rebar源码/配置/测试/报告、v11全部成功与失败原始工件、全部已保存的10月25日归档及校验凭据、完整Git refs bundle和当前未提交工作区快照。`manifest.json`逐文件列SHA/字节；`LOCAL_VERIFIED.json`和`BOTH_HOSTS_VERIFIED.json`给出本轮真实核验结果。完成前不把目录存在当作通过。

NAS路径继续为`/mnt/zspace/rebar_migration_20261024_142019/`。本轮检测到挂载曾缺失，随后恢复为CIFS；只有确认挂载后才继续写入。最新原始结果包位于`supplements/workcell_results_177bbe120a0b.tar.gz`，406文件逐SHA通过。汇总增量位于`supplements/rebar_migration_followup_20261025.tar.gz`，最终以对应BOTH_HOSTS_VERIFIED凭据为准。NAS基础包本轮仅复核入口清单，完整对象核验是此前凭据；迁移前应在目标盘运行完整verify。

## 在新主机恢复

先把基础目录与本轮增量复制到目标盘，给完整恢复预留至少146GB加增量、Git及环境空间；约84GB指基础去重对象内容，不是完整展开大小。

```sh
cd /新磁盘/rebar_migration_20261024_142019
shasum -a 256 -c TOOLKIT.sha256
python3 migration_folder.py verify --backup .
python3 migration_folder.py restore --backup . --destination /新磁盘/rebar-base-restored
```

进入本轮增量目录运行`python3 verify_followup.py --backup .`，核对全部payload后，在全新目录从`metadata/repository.bundle`克隆完整Git历史。将增量的`workspace/`覆盖到新克隆工作区，随后将基础恢复中的大型artifacts/固定模型及历史证据合并进去；同名文件以经过核验的本轮增量为最新，不覆盖base备份目录本身。`archives/`保存10月25日历史归档，保留原包，不向live目录盲目全部解压。`metadata/working_tree_extra/`及patch保存GUI/地图等未提交快照，仅在确认需要时恢复；本轮没有stage或修改这些源文件。

当前正式物理候选实际执行源码为v11目录内不可变`source_v11/`，配置和job.json只用于审计。迁移恢复**不要运行旧launcher、manager、runner或恢复旧PID**；它们是历史作业描述。后续验证须先明确新任务，不能重复原正式64或已成功组合。

## 必须保留的身份和边界

冻结policy+optimizer SHA `393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82`，52张量，updates466/actor_updates170/policy_steps36602；检测器845f5aec…、资产manifest d543044f…、视觉配置cd2f72ff…。本轮Mac与103四文件重新SHA一致，两份原生报告也确认payload/counters未改。

基础包保留原视频、完整物理历史和Cosmos固定模型。Mac虚拟环境不能直接当作Linux/CUDA环境；Isaac Sim/IsaacLab、GPU驱动/CUDA及系统库须按基础包版本说明在新主机安装。SSH身份、账户凭据和全局网络/电源设置不属于恢复内容。恢复检查仅允许CPU文件/接口核验，不自动启动GPU任务。

下一项算法工作是离线隔离B料区源导航未完成的原因，并修复未获取视觉状态时的失败回执路径；本轮没有修改或重试该问题。

## 最终核验凭据

本轮核查与迁移备份已完成（2026-10-25T23:18上海）：A→3完整复测通过，B→1异常停止，B→2/B→3未启动，当前无本任务仿真进程。Mac/103重新核对冻结身份；386份节点文件逐SHA一致，406文件结果包双端完整核验。汇总迁移包700文件、2,020,400,470字节，SHA9c070aae017c243716b15cff091116a6f0f85135f345d29b5b40ce0ca2aa6a97；Mac与NAS均逐文件通过，实际恢复88份关键文件并独立克隆Git历史/审核器通过。迁移复制完整基础目录和`/Users/weiluo/Develop/backups/rebar_migration_followup_20261025/`（含final_state及完成凭据）；使用压缩包替代目录时也须带走同目录旁的`rebar_migration_followup_20261025.final_state.tar.gz`。数据就绪不代表目标GPU环境已安装；本轮未启动仿真、重试、训练或修改policy/optimizer。详细步骤见MIGRATION_CHECKLIST_20261025.md。

最终收尾资料位于迁移增量目录的final_state/，包含最新文档/报告、网络恢复证明、备份索引及仅文档变化的Git增量bundle（以主包807d9fb完整bundle为前提）。final_state/manifest.json和FINAL_STATE_VERIFIED.json单独给出收尾文件及NAS核验，主包700文件manifest保持不可变。恢复时先应用主包workspace，再应用final_state/workspace。
