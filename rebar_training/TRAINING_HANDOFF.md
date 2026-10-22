# 钢筋视觉移动操作训练：接力指导

## 最新迁移校验：4090 已接收，训练仍暂停（2026-10-22 上海 08:49）

已按用户“先完成迁移和校验，暂时不要启动训练”的要求，将迁移包传至 `arnold@192.168.3.133` 并解压到新专用目录 `/home/arnold/Develop/rebar-training-4090-return-20261021`。旧4090目录完整保留，未恢复任何旧PID或job。

- 包 SHA256 与本机一致，**2573 文件逐一匹配、零不匹配**。原始包及校验文件在目标 `/home/arnold/Develop/rebar-migration-back4090-20261022/`；当前运行依赖和全部v9证据已落盘。
- 4090 CUDA 实际加载 f70f6115 策略及optimizer，网络/optimizer有限，输出1×6，计数保持 **329 updates /122 actor /26712 steps**；未执行策略更新。源码2c、检测器845f及资产d543指纹一致，10个USD依赖闭合，SDK的OmniPBR材质文件存在。
- CPU全套 **239通过，无跳过**。初次检查发现缺少jsonschema，按 `uv.lock` 精确版本及wheel哈希安装到新目录 `.runtime_overlay` 后补验通过，未改共享Isaac Lab/SDK环境。手动环境文件为目标根目录 `MIGRATION_ENV.sh`，仅导出路径，不启动进程。
- **未启动训练、评估、SimulationApp或物理回归；无本任务进程，GPU空闲，automation-2继续PAUSED。** 用户明确恢复前保持此状态。迁移校验不等于物理回归或稳定验收通过。
- 恢复候选仍是f70f权重，v9抓取gate8/8、到站独立4成功0失败但gate未完成、最终full0/64；历史v8完整组16回合另2成功单列。物理回归和新种子的后续课程须待恢复授权，在新输出中执行。

目标现场入口 `MIGRATION_TARGET_STATE.json`，详细凭据 `reports/return_4090_20261022.json` 及 `review_20261020/return4090_20261022/`。后续文档更新仅覆盖迁移元数据；冻结源码、模型、资产和原生证据不改动，原包保留原始清单，目标迁移后文件以 `TARGET_FILES_MANIFEST.json` 复验；`target_metadata_overlay.verified.json` 单列迁移文档变更。


迁移包已完整校验：`artifacts/rebar_training_migration_back4090_20261021.tar.gz`，1,439,210,328 字节、2573 文件、零不匹配，SHA256 `766c0d05c86bf3c5864b918ce064282f0272dc2a2323d2d469c23e481f320e26`。同名 `.sha256`、`.verify.json`、`.recovery_status.json` 提供包外最终校验和暂停入口。4090已完成接收与校验；新旧设备均不自动恢复。


## 当前暂停：准备从 5090 迁移回 4090（2026-10-21 上海 23:35）

**用户最新要求“先停止训练，准备好迁移回4090”。本节覆盖下方所有旧恢复授权。5090 课程父进程及两个评估进程组已停止，无本任务残留进程、GPU 空闲；automation-2 已暂停。未经用户后续明确恢复，新旧设备均不得启动训练、评估、回归或演示。**

- 停止凭据：`artifacts/mobile_curriculum_20261020/review_20261020/migration_back_4090_20261021/pause.stopped.json`。原 controller 112563、评估 PG 119251/119252 仅作历史记录，不能恢复旧 PID 或在原输出目录重跑；原生报告和中断日志完整保留。
- 可回迁策略：v9 的 `dock_clamp_policy.pt`，SHA256 `f70f6115c984357a305f10b13eb3a1a6536740e0ea1e98545d093b517a5cd831`，含 optimizer，329 updates / 122 actor / 26712 有效步，网络与 optimizer 已复核有限。冻结源码 `2c7ef709`，检测器与资产保持原指纹。
- 5090/v9：抓取技能 gate **8/8 通过**，到站训练 **2/2 成功**，到站独立两组各完成 **2/4、累计 4 成功 0 失败**，未通过完整 8 回合 gate；两组第三回合因用户暂停中断，不算成功或物理失败。最终整链独立 **0/64**。
- 4090/v8 历史：完整组 **16/64**，另两中断组各 1 回合成功，共 **18 成功 0 失败**；单列保留，不与 v9 或未来回迁课程合并。
- 当前课程及迁移物理回归 **2212 文件、1,424,971,305 字节**已从 5090 同步至本机并逐一 SHA 核对。迁移包和逐文件校验清单见下方迁移指导；包含策略/optimizer、源码、原生报告/完整日志/图像、运行依赖和暂停记录。
- 原 4090 地址 `arnold@192.168.3.133` 本次只读 SSH 核查超时，未写入或启动任务。候选新目录 `/home/arnold/Develop/rebar-training-4090-return-20261021`；保留旧 `/home/arnold/Develop/rebar-training-4090`。收到恢复授权后，先校验目标 SDK/CUDA/冻结指纹和真实相机接触物理回归，再在新目录预声明未用种子推进课程和独立验收。

详见 `MIGRATE_BACK_4090_20261021.md` 和 `reports/back_to_4090_pause_20261021.json`。此时没有“最终稳定通过”的结论。


## 历史运行记录：RTX 5090（已被上方暂停指令覆盖）

**23:32 上海时间连接正常，原课程继续运行，本次没有手动启动、停止或重启仿真。当前为到站上料夹紧独立评估，必须先读取原控制器和新鲜报告。最近 45 分钟的 systemd suspend/hibernate 单元日志无记录（returncode 0、stderr 空）；不据此猜测先前断联原因，未改电源设置或 SSH 身份。历史网络记录保留在 review 目录。**





用户已明确要求“切换到 5090 的电脑上继续”，本节覆盖下方历史暂停说明。仅恢复仿真；4090 和 0.103 的旧目录均保持停止。

- 节点：`arnold@192.168.0.103`，RTX 5090 Laptop GPU（24 GB）；新目录 `/home/arnold/Develop/rebar-training-5090-resume-20261021`。
- 入口：`artifacts/mobile_curriculum_20261020/review_20261020/status.json`；**当前课程 `course_v9_5090_resume`，控制器 PID/PGID 112563**。先看当前课程 `curriculum.status.json`、`active_jobs` 和原生 job/status/progress/log。迁移验证控制器 108930 已以 `course_started` 完成退出，不要重启旧 PID/验证作业。
- 217 个运行文件逐一 SHA 核对；完整历史归档 5,350,801,884 字节在 Mac 和 5090 两端 SHA 一致。历史归档留在 `/home/arnold/Develop/rebar_training_migration_platform_20261021.tar.gz`，不得解压覆盖正在运行的新目录。
- CUDA 已实测加载冻结 `fe48b74d` 策略与 optimizer，计数 278 updates / 100 actor / 21833 steps，网络与 optimizer 有限；10 个 USD 运行层依赖闭合，仅 SDK 的 OmniPBR 材质由仿真运行时提供。源码仍冻结 `2c7ef709`，检测器和资产不变。
- 上海时间 **22:32 核查**：迁移物理回归 **3/3 全部成功、0 失败**。连续复位两回合误差 1.548/1.613 mm；30 帧部分抓取失视回合误差 1.502 mm，实际触发一次抓取中止，实测打开无载、托槽支撑且已返回后重新观察，半速恢复并完成四爪夹紧、卸载和退让。全部原生报告、源码/策略/检测器/资产及严格物理凭据独立复核通过，766 个完整原始文件两端 SHA 核验。回归不计最终独立分母。
- 节点 CPU 全套 **239 通过**；`tray_train` 已原生完整 **2/2 成功**，放槽误差 0.632/0.008 mm，实际支撑、打开卸载及约 179.7 mm 退让复核通过。新增 **25 次 PPO、12 次 actor、2790 个有效策略步**，最终 303 updates / 112 actor / 24623 steps；actor 最大权重变化 0.009190，模型与 optimizer 有限。输出策略 SHA 为 `d1e26e7bb23d43d6d111af24180d10e284120055fe9d0104f512f95a06b3d851`，130 个完整训练原始文件已两端 SHA 核验。训练完成不代表独立技能 gate 通过。
- 新版抓取放槽独立评估 `6131042/6141042` 已各 **4/4、合计 8/8 通过**。本机与节点独立复算 gate 完全一致：Wilson95% 下界 0.675592435 达到技能 0.60 门槛、零证据错误。放槽误差范围 0.243–1.007 mm，真实支撑/打开卸载/退让及冻结 d1e26/source2c/检测器/资产复核通过，456 个完整原始文件两端 SHA 核验。技能 gate 不计入最终 64 回合整链分母。
- 到站上料夹紧训练 `dock_clamp_train` 已原生完整 **2/2 成功**，夹紧误差 **1.619/1.856 mm**，真实四爪、松爪卸载和 96.7/88.7 mm 退让及稳定速度复核通过。新增 **26 PPO / 10 actor / 2089 有效步**，累计 329 updates / 122 actor / 26712 steps；actor 权重最大变化 0.007041，模型与 optimizer 有限，最终策略 SHA 为 `f70f6115c984357a305f10b13eb3a1a6536740e0ea1e98545d093b517a5cd831`；156 个完整训练文件已两端 SHA 核验。
- 当前原控制器 112563 已自动推进到站独立评估 `dock_clamp_holdout_6151042/6161042`，runner 119251/119252（以实时 `active_jobs` 为准），各 4 回合、同一 f70f/source2c/检测器/资产，最多两仿真并行。23:31 两组已各完成 2/4，累计 **4/8 成功、0 失败，较 23:25 新增 2 回合**；四爪实载、松爪卸载、退让和速度复核通过，夹紧误差 1.298–1.374 mm，新增不可变快照两端 SHA 已核对。第三回合运行中，日志新鲜；**到站 gate 和最终整链 gate 尚未通过**。
- 迁移回归全部通过后已自动创建 `course_v9_5090_resume`，按 `config/mobile_curriculum_recovery_v9_5090.json` 重跑训练及技能 gate。新训练种子 6031042/6041043/6051043；技能评估 6131042–6161042 各 4 回合；整链评估 6241042–6311042（间隔 10000）各 8，共 64。预声明种子与 115 个已存在作业/计划种子、检查点训练上下文均无重叠。
- 汇报口径：**4090／v8 历史完整组报告为 16/64，另两组各 1 个成功回合后用户中断，总计 18 成功、0 失败；5090／v9 另行统计。** 23:17 确认 **v9 最终整链独立 0/64，成功 0、失败 0、本轮新增 0**；抓取放槽 8/8 是技能 gate，另行统计。 迁移前成绩与新平台验收不合并。 每阶段 gate 必须实际通过，同一最终冻结策略/源码/检测器/资产，成功率 ≥95%、Wilson95% 下界 ≥90%、零证据错误，才宣布稳定。最多两仿真并行。
- 自动跟进已恢复，每 15 分钟汇报上海时间、累计回合/64、成功失败、较上轮新增、当前阶段与下一步；节点不可达时明确注明实时状态未知。详见 `reports/resume_5090_20261021.json`。失败则保存全部证据分析修复，用新目录和未用种子验证；不修改旧证据或放宽判据。

## 历史暂停记录：更换训练电脑（已被上方恢复授权覆盖）

**训练、独立验收及定时跟进均已暂停。用户未明确恢复前，不启动或恢复课程。**
3.133 的课程调度器及两组评估已停止，并在录视频前确认 GPU 空闲；旧进程号只作历史凭据。
当前冻结策略 `fe48b74da455c7562ff079d9990cd0a8b534d064d9217029a48431cea5bc235b` 包含 optimizer，源码仍为 `2c7ef709`。
暂停时整链独立验收共有 **18 个成功完整回合、0 失败**：5241042/5251042 两组各 8/8 完整结束；5261042/5271042 两组各完成 1 回合后被用户中断，均不是完整 8 回合报告。
正在执行的第二回合不计入完整回合数。**64 回合稳定性门槛尚未通过**，不能把 18/18 或两个技能 gate 当作最终验收。
用户单独授权的演示已 **1/1 成功录完**，视频约 **5 分 4 秒**，夹紧误差 **1.435 mm**；真实四爪夹紧、打开卸载及退让均通过复核，整段视频解码及关键画面检查通过。
[播放当前效果演示](artifacts/mobile_curriculum_20261020/review_20261020/migration_pause_20261021/current_policy_demo_labeled.mp4)。视频标注“稳定性验收未完成”，不更新策略、不计独立分母；录制后本任务全部仿真进程已退出、GPU 空闲。

迁移步骤见 [更换训练电脑指导](MIGRATE_PLATFORM_20261021.md)；最新入口为 `artifacts/mobile_curriculum_20261020/review_20261020/status.json`。
暂停凭据为 `artifacts/mobile_curriculum_20261020/PAUSED_20261021_PLATFORM_CHANGE.json`，摘要为 `reports/platform_pause_20261021.json`。
不要重启旧 PID、旧课程 `--worker` 或在原输出目录重跑命令；源码/策略/资产保持冻结，中断证据保留。

## 暂停前记录（历史进度及进程号，不是恢复指令）

用户已授权在 `arnold@192.168.3.133` 的 RTX 4090 继续；工作目录 `/home/arnold/Develop/rebar-training-4090`，旧 0.103 保持停止。
当前课程为 `artifacts/mobile_curriculum_20261020/course_v8_precision_envelope/`，控制器 PID/PGID 30740；抓取放槽独立评估已 **8/8 通过**，到站上料夹紧训练已完整 2/2 完成；到站独立评估也已 **8/8 通过**；`full_train` 整链训练已完整 2/2 成功结束；当前进入 64 回合独立整链验收，首两组 5241042/5251042 已各 8/8 完整成功；后两组 5261042/5271042（进程组 63577/64110）正在并行，不使用到站/托槽初始化或视觉提示包。
先读本机 `artifacts/mobile_curriculum_20261020/review_20261020/status.json`，再读当前课程 `curriculum.status.json`、`active_jobs` 和新鲜原生报告；已有作业时不要重复启动。

前置开发回归全部 **9/9 通过**：
- 原失败策略与两个失败种子：2/2，误差 2.129/2.114 mm。
- 整链训练回归：2/2，误差 2.031/1.830 mm；新增 71 次 PPO 更新、23 次 actor 更新、4930 个有效策略步，模型及 optimizer 有限。
- 30 帧闭爪失视：1/1，误差 1.761 mm。实测打开卸载、托槽支撑且已返回后才重观察，最多两次恢复均取得新视觉定位。
- 同进程连续复位：4/4，误差 1.809/1.860/1.777/1.877 mm；真实四爪夹紧、松爪卸载、退让与稳定速度均通过独立凭据复核。
原始完整文件两端 SHA256 校验：到站 359、训练 701、失视 778、连续复位 1038；对应冻结源码、图像/概率、关节/接触日志和策略 optimizer 均已在本机保存。
这些回合仅作开发回归，不计入正式独立验收。

当前课程冻结控制源码 SHA 为 `2c7ef709bc6ad5d6ed5128c9d4e7399179938cf230129cd0e1c33221f7010aa2`，计划 SHA 为 `308bf3611a6ad21e5a5723d2319b3076cdcc740cf6c12f700533266c7c3842e8`。
初始化权重为复制校验过的 d2875968；抓取放槽训练新增 22 次 PPO 更新、12 次 actor 更新、2808 个有效策略步，最终权重 SHA 为 19247f42e0705408a220b9d52052133dc390a833939f8e2cb2980c1bf41adcf5，模型及 optimizer 有限；175 个完整训练文件已两端 SHA256 校验。训练完成不等于独立阶段门槛通过。
抓取独立种子 5131042/5141042 各 4/4 通过，复算 gate 与节点完全一致：Wilson95% 下界 0.675592435，达到阶段 0.60 门槛，证据错误为零。
8 回合均为同一 19247f42 策略、2c 源码、检测器及资产；最大放槽位置误差 0.818 mm，真实支撑/松爪卸载/退让凭据复核通过，484 个完整原始文件已两端 SHA256 校验。
到站训练从 19247f42 初始化，已完整 **2/2 完成**，夹紧误差 2.037/1.405 mm；真实四爪、松爪卸载及退让通过复核。
新增 41 次 PPO 更新、11 次 actor 更新、2145 个有效策略步；最终策略 SHA 为 9b2a66f90b1c2ee33ab3f8c4db8f814ffd20872c264a56f3146b8f2ec6d808c5，模型及 optimizer 有限，231 个完整训练文件已两端 SHA256 校验。
到站独立种子 5151042/5161042 各 **4/4 通过**，复算 8/8 gate 与节点一致：Wilson95% 下界 0.675592435 达到阶段 0.60 门槛，证据错误为零。
8 回合误差范围 1.654–1.848 mm，同一 9b2a66f9 策略、2c 源码、检测器和资产；真实四爪、松爪卸载、退让与速度判据均复核通过，482 个完整原始文件已两端 SHA256 校验。
整链训练从 9b2a66f9 初始化，种子 5051043，已完整 **2/2 成功结束**，夹紧误差 2.009/2.161 mm；真实四爪、卸载和退让复核通过。第二回合部分抓取失视后，先实测打开卸载、托槽支撑且已返回，再完成一次半速重新观察恢复。
新增 **86 次 PPO 更新、24 次 actor 更新、4987 个有效策略步**；最终为 278 updates / 100 actor / 21833 有效步，网络与 optimizer 有限，321 个完整原始文件已两端 SHA256 核对。最终策略 SHA 为 `fe48b74da455c7562ff079d9990cd0a8b534d064d9217029a48431cea5bc235b`。训练回合不进入最终 64 回合独立验收分母。
正式训练种子 5031042/5041043/5051043；抓取评估 5131042/5141042、到站评估 5151042/5161042 各 4 回合。
抓取和到站两个独立 gate 通过、整链完整训练复核后已启动最终独立评估：5241042 至 5311042（间隔 10000），各 8 回合，共 64。同一冻结 fe48b74d 策略、2c 源码、检测器和资产，仿真最多两个并行；首两组已各 8/8 完整成功，累计 **16/64 成功、零失败**，误差范围 1.352–1.748 mm。1940 个完整原始文件已两端 SHA256 核验；独立复算零证据错误，Wilson95% 下界 0.806392319，仍未满足 64 回合和下界 0.90 的最终门槛。后两组运行中，保留所有后续回合，不修改冻结策略或判据。
**新版抓取放槽和到站夹紧两个技能 gate 已通过；64 回合完整连续整链稳定性验收和合格完整流程视频尚未完成。** 最终须同一策略/源码/检测器/资产，成功率至少 95%、Wilson95% 下界至少 90%、零证据错误。

保持 0.8 mm 视觉对准门控、3 mm/3° 独立评分、250 N、真实四爪夹紧、松爪卸载和退让判据；钢筋仿真真值不进入运行时控制。
当前是 76 维视觉/机器人反馈、6 维关节速度残差 PPO 叠加 RGB 伺服、阶段管理与里程计导航；尚未调用 Cosmos 3 推理。仅限仿真，不启动真实机器人。
旧 v7 到站独立评估失败且中断，不能复用旧阶段成功作为新版本验收。旧验证控制器 15451 已完成退出，不要重启旧 PID 或验证目录。
迁移基础包及增量归档在本机 artifacts；运行时 Python3.12 / Sim6.0.1 / Labv3.0.0-beta2.patch1 / CUDA 和 USD 依赖已核查，未修改原模型或 SDK。

## 此前恢复覆盖说明（2026-10-20）

用户已要求继续推进至稳定性验收，当前授权有效；仅仿真，直接 SSH 到 `arnold@192.168.0.103`。

上一轮 `course_v1` 的抓取放槽独立门槛已 **8/8 通过**；到站上料为 **7/8**，
课程已按门槛停在 `needs_analysis`，没有进入整链训练或宣称稳定。
失败是空腕返回时纹理跟踪短暂丢失后无法恢复，夹爪保持打开、钢筋仍在槽上，最终 `pickup_tray` 超时。
完整分析在 `artifacts/mobile_curriculum_20261020/review_20261020/failure_analysis.json`。

本次修复新增 `vision/recovery.py`：仅当空腕已经返回、夹爪打开且无载荷、托槽有支撑载荷时，
允许重新取得三帧完整端部观测，再以半速回放实际记录的关节路径；最多两次，不读取钢筋真值，
不修改钢筋位姿或放宽 3 mm/3°、力限值、夹紧卸载退让判据。已通过相关 52 项测试和节点 CPU 全套 199 项；
之后课程恢复/并行调度的相关 39 项测试通过。

本次开发回归已 **5/5 通过**：短中断 2/2（1.11/1.22 mm）、旧失败种子 2/2（1.13/1.42 mm）、
36 秒中断恢复 1/1（1.06 mm）。长中断实际触发重新观察一次：tick 2226 开始、3291 重新定位，
随后半速回放返回并完成真实夹紧和退让。这些 regression 不计入独立稳定性验收。

已启动新课程 `course_v2_recovery`，父进程 PID/PGID 25995，冻结源码 SHA
`ca355ca665d6e0d945335c78cd6b34cf61cc4d1b33b83a532711141070c6df58`；
当前抓取放槽训练（首个训练回合已成功，新增 6 次 actor 更新），初始检查点已复制到课程目录，当前**仍未通过新版本独立稳定性门槛**。
每 15 分钟由本对话自动跟进（automation-2），处理失败并推进，阶段 gate 仍必须实际通过。
自动跟进依赖本机及桌面应用保持运行；Ubuntu 后台课程使用独立进程组和 idle inhibitor。
读取 `review_20261020/status.json` 与新课程 `active_jobs`，不要重复启动旧 `course_v1`。

新计划为 `config/mobile_curriculum_recovery_v2.json`，所有正式评估种子均未用于此前调试。
训练种子 431042/441043/451043；抓取放槽评估 531042/541042（各 4 回合），
到站上料评估 551042/561042（各 4 回合）；整链评估八组 641042 至 711042、间隔 10000（各 8 回合，共 64）。
独立评估最多并行两个仿真作业，训练仍单作业。每个报告固定同一冻结源码、策略、识别器和资产。
整链门槛通过后自动另录一回合示范视频；视频回合标为 regression，不进入独立验收分母。

课程支持 `--initial-checkpoint`：先复制并记录权重 SHA，再重跑每个阶段与 gate，不跳过旧已通过阶段。
候选初始化是 `course_v1/dock_clamp_policy.pt`（SHA `fb14312a70f80454cfa13fb136746fb658f8d444471138f4555a519bb0b3a8b5`）；
它是旧版 **7/8 未达标** 权重，只作 warm start，不能视为合格部署策略。
旧奖励契约的早期纯 critic 检查点仍不可用；当前奖励契约为 `mobile_residual_potential_v2`。
当前仍未达到最终稳定性验收，Cosmos 3 尚未调用。

本地已完成原待办中的奖励与训练隔离修复：
- 有界势函数整形，终止势函数固定为零，不读取下一复位回合的势函数；gamma 与 GAE 统一为 0.9995。
- 未执行的随机残差不计动作成本。
- surrogate、entropy、anchor 均只使用 active 样本。全无 active 样本时不建立 actor 计算图，
  Adam 不更新 actor 动量；actor/critic 分开裁剪梯度，避免 critic 梯度范数影响 actor。
- 新契约 `mobile_residual_potential_v2`，检查点显式保存 gamma 和 actor_updates；旧奖励契约拒绝加载。
- 已补测试：失败进度不能增加折扣收益、终止不跨复位、势函数有界、critic-only 更新不改变已具有动量的 actor。

准确测试结果和恢复状态见 `artifacts/mobile_curriculum_20261020/resume_20261020/`。
接下来先核对 `course_v2_recovery` 的训练和独立评估进度，仍不能跳过 gate 或把小样本回归称为稳定。
旧增量备份是上次暂停时的快照，不包含本次新修复；恢复网络后另做版本化备份，不覆盖历史包。

## 上次暂停时的操作状态（历史）

**此为此前暂停记录；用户现在已经恢复授权，以顶部最新状态为准。**
本轮运行作业已停止；暂停时通过 `ps` 与 `nvidia-smi` 确认无残留仿真/GPU 训练进程。
`artifacts/mobile_curriculum_20261020/PAUSED.json` 保存停止与同步信息，是操作记录，不是强制启动锁。
没有设置自动唤醒、定时任务或后台课程。用户已授权恢复后直接 SSH，不再经过飞书。
所有工作仅限仿真；不要启动实际机器人、ROS 驱动、夹具硬件或修改真实控制配置。

## 项目与环境

| 项目 | 位置或实测版本 |
|---|---|
| 本机仓库 | `/Users/weiluo/Develop/github/AuboAMR300_ROS2` |
| 训练节点 | `arnold@192.168.0.103` |
| 实际 GPU | RTX 5090 Laptop GPU，24463 MiB；不是旧节点的 4090 |
| 训练工作副本 | `/home/arnold/Develop/rebar-training-0103` |
| Python overlay | `/home/arnold/Develop/rebar-training-runtime/bin/python` |
| Isaac Lab | `/home/arnold/Develop/IsaacLab-rebar`，`v3.0.0-beta2.patch1` |
| Lab 提交 | `ffff603eafc6b74264a5261cc0183d6a65390d78` |
| Isaac Sim | `/home/arnold/isaacsim`，6.0.1-rc.7 |
| GPU Python / Torch | Python 3.12.14，Torch 2.11.0+cu130 |
| 当前移动资产 | `rebar_training/artifacts/assets_mobile_v2` |
| 其依赖的完整工位资产 | `rebar_training/artifacts/assets_full_v2_precision` |
| 本轮产物 | `rebar_training/artifacts/mobile_curriculum_20261020` |
| 上轮已验收产物 | `rebar_training/artifacts/visual_control_20261019` |

训练模块独立于原 ROS 工作区。不要改原 ROS 模型或 Lab 源码，不要混用系统 ROS Lyrical/Python 3.14。
Lab 3 数据是 ProxyArray，张量需 `.torch`；四元数 XYZW；浮动根 Jacobian 的关节列需偏移 6。
运行脚本已经处理运行时路径，不要用 `uv sync` 替换 GPU 节点随 Isaac 安装的 Torch。
Mac 上 CPU 检查使用 `rebar_training/.venv/bin/python`，版本与 GPU 环境不同。

原 ROS 模型来源：`/home/arnold/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2`。
当前仓库另有 GUI、地图、根 README 和 `.gitignore` 的已有改动，不属于本任务；不要 reset/clean 或覆盖。
本轮修改集中在 `rebar_training/`，该目录目前仍是 Git 未跟踪内容，尚未提交。

## 当前能力边界，避免混淆结果

此前两个完整连续回合已经通过：视觉抓取→抬升→放入托槽→载料导航→停靠→再抓取→插入夹紧→松爪退让。
种子 119042、125042，分别 9041/9199 步，实际最终误差 2.12/2.47 mm。这是旧版视觉伺服原型的小样本结果。
完整录像为 `visual_control_20261019/mobile_full_125042_v7_recording.mp4`，306.6 s，10 fps。
另有固定底盘的视觉 BC/DAgger 神经策略和更早的状态 PPO；它们均不是移动整链 RL 权重。

当前移动流程仍使用视觉伺服、已知工位路点、观察运动、阶段/夹爪管理和里程计导航。
不是 Nav2 动态避障、任意仓库寻筋或独立整链神经策略。初始底盘位于已知取料区域。
料台、托槽、试验机位置已测定；钢筋精定位来自腕部 RGB/灰度图像。
运行控制受 `blinded_control` 约束，不能读取钢筋/料台真值、奖励阶段或按钢筋身份过滤的接触力。
真值只用于复位、奖励、标签和独立评分。每回合运行期间钢筋保持真实动力学，无附着、位姿写入或关闭碰撞。
四爪受力、250 N 失败阈值、实际 3 mm/3°、松爪卸载与至少 6 cm 退让不应放宽。
移动交接还保持约 1.5 s 验证；视觉触发夹紧采用 0.8 mm 内部余量，为视觉误差留出预算。

**尚未下载、部署或调用 Cosmos 3。** 当前检测器为本地灰度 U-Net；Cosmos 扩增只准备了离线接口。

## 本轮实验的准确状态

以下文件均在 `artifacts/mobile_curriculum_20261020/`，已同步到 Mac。

| 作业前缀 | 事实与结论 |
|---|---|
| `expanded_full_221042_v1` | 请求 4 回合；完成第 1 回合后自然失败，`stage timeout: tray_raise`，5091 步。之后终止旧版批次，因此是已完成 0/1，另外 3 回合未完成，不能写成完整 0/4。完整失败凭据在 `.progress.json`。 |
| `expanded_tray_221042_v2_retract` | 修复抬升路径，同种子 **1/1 抓取放槽通过**，2762 步、92.07 s。槽内真实中心约 `(0.292700,-0.000036,0.714999)`；不是整链夹紧成功。 |
| `residual_tray_231042_v1` | 初始钢筋更靠近底盘，原观察画面裁掉一端，拒绝初始化。试运行后停止；**4 次 critic 更新、policy_steps=0、没有完成的训练回合**。`residual_tray_v1.pt` 不能当作已训练策略或合格恢复起点。 |
| `expanded_tray_231042_v3_wideview` | 尝试只抬高相机，目标姿态不可达，120 s 后 `stage timeout: pickup`，完整 **0/1**。保留用于诊断，不继续使用这组高位候选。 |
| `expanded_tray_231042_v4_horizontal` | 新增横向观察、侧向偏置。暂停前最后日志 step=960：`valid=true`、已取得初始定位、正在回放实际关节路径返回，`returned=false`。没有完整回合结果，**不可记成功**。作业状态 `paused_by_user`。 |

原视野裁端证据：`residual_tray_231042_v1/wrist_step300.png`；本机另存 `wrist_step300_overlay.png`。
`*.job.json` 记录命令和源码 SHA，`source_snapshots/<作业名>/` 保存每个作业实际执行代码。
`.job.status.json` 的 `completed` 只指进程退出；最终是否成功须读新鲜、完整的任务报告，不能只看进程 exit 0。
Isaac Kit 异常有时也返回 0，现有 driver 用 UUID 和完整回合凭据再次确认。

暂停时 canonical 源码和旧 v4 快照略有不同；恢复后的实际版本以顶部 SHA 和各作业快照为准。
不要把不同源码的运行报告拼成同一稳定性验收。

## 已完成的实现

1. `randomization.py` / `reset.py` / `isaac_cfg.py`：基线、expanded、challenge 三档可复现复位。
   expanded 为料台/钢筋整体 XY ±60 mm、yaw ±0.10 rad、手臂关节 ±0.035 rad，底盘 XY ±20 mm/yaw ±0.02 rad；
   每回合图像增益 ±10%、偏置 ±4，逐帧高斯噪声 σ=1。challenge 扩至 ±90 mm、±0.16 rad，尚未验收。
   料台与钢筋整体复位以保持支撑；没有测试独立乱放钢筋、摩擦/质量、尺寸或标定误差随机化。
2. `mobile_env.py`：抓稳离台后，抬到 0.9 m 的同时将过远的视觉目标向内收到 x≥-1.10，解决最先发现的伸展极限卡死。
   托槽课程初始化的钢筋姿态现按底盘真实复位旋转变换，避免底盘 yaw 随机化后坐标不一致。
3. `vision/observation.py` 的 `PICKUP_SEARCH_VIEWS`：保留原观察位优先；若无法见到双端，尝试横向展开的候选。
   第一候选相机 `(-0.50,-0.10,1.25)`，reference=(0,1,0)，避免对称手腕奇异位，并利用 1024 像素长边。
   只有双端点、连续三张有效图像通过后才接受初始定位；全程保留真实外出关节历史以回放返回。
   新候选有离线 USD IK 和全部 expanded 边界端帽投影检查；恢复后的完整抓取放槽已在 v5 回归 **1/1 通过**。
4. `mobile_mission_check.py`：`--episodes`、`--randomization`、逐回合自动复位和原子进度；保留失败在分母中。
   新版保存终止前实际相机包、阶段、根位姿、关节、接触载荷与估计值，便于分析自动复位前的失败。
5. `mobile_residual.py`：纯 PyTorch 有界残差 PPO（与历史 RSL 状态策略分开）。
   契约 `mobile_visual_manipulation_residual_v1`，76 维输入只含视觉估计、测量反馈、已知目标和阶段。
   输出 6 个关节速度修正，默认归一化幅度 0.08，即最高 0.028 rad/s，叠加在视觉伺服上。
   夹爪时序、观察、阶段和底盘导航仍是既有控制器。底层 Gym 实际执行 7 维动作；残差 driver 完成 6→7 的合成。
   相机失效或观察引导期间 `active=false` 的样本不参加 surrogate/entropy/anchor；
   critic-only minibatch 不改变 actor 或其 Adam 动量，GAE 在回合终止处截断。
   检查点包含 actor/critic、optimizer、更新数、有效策略步数、训练种子与幅度契约。
6. `mobile_acceptance.py`：独立评分门槛，拒绝训练回合、回归回合、缺失凭据、不同权重/源码/识别器/资产混合、
   托槽初始化伪装整链、钢筋运行中写位姿或安全判据矛盾。整链至少 2 个种子/64 回合，成功率 ≥95%，
   Wilson 95% 区间下界 ≥90%；课程阶段至少 2 个种子合计 8 回合。**这些门槛尚未实际通过。**
7. `mobile_jobs.py`：单作业冻结源码、日志、进程组、可配置 SDK 路径；`mobile_curriculum.py`：有限课程调度，
   冻结计划与源码，独立验收失败即停在 `needs_analysis`，不会自动晋级。
   `config/mobile_curriculum.json` 预先区分训练/评估种子。**旧 course_v1 到站门槛失败；新 course_v2_recovery 正在重跑全部阶段。**

## 当前优先处理的问题

- 以 `review_20261020/status.json` 的 `current_course` 为准，读取 `course_v2_recovery/curriculum.status.json`、
  `active_jobs` 及每个阶段的新鲜完整报告。旧 `course_v1` 已终止，不要重启或重复创建作业。
- 新恢复逻辑已在 5/5 开发回归通过（包括 36 秒相机中断后重新观察），详见
  `review_20261020/recovery_validation.json`；这些不是独立验收样本。
- 新课程首个训练回合已成功，累计新增 6 次 actor 更新。冻结初始化权重不等于合格策略，
  阶段门槛仍需两组共 8 个独立回合，整链仍需新计划固定 64 回合。
- 本对话每 15 分钟自动跟进（automation-2）；元数据和提示词在 `review_20261020/stability_followup*`。
  没有可行动变化时保持安静，失败时分析修复后继续，通过且备份交付完成后停用。
- 换源码、识别器、资产或权重后重新做对应验收；不混用版本。评估失败经分析的种子成为开发样本，
  新最终验收须另预声明未使用种子。不能修改失败分母、夹紧力、真值隔离或 3 mm/3°。

## 继续核查现有作业

已有恢复授权，直接 SSH；所有命令仅限本项目仿真。先读取进程、GPU 和作业状态，不要杀掉他人的进程：

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 arnold@192.168.0.103
cd /home/arnold/Develop/rebar-training-0103
nvidia-smi
env PYTHONPATH="$PWD/rebar_training/src" python3 -m rebar_training.mobile_jobs --status
env PYTHONPATH="$PWD/rebar_training/src" python3 -m rebar_training.mobile_jobs \
  --artifact-dir "$PWD/rebar_training/artifacts/mobile_curriculum_20261020/course_v2_recovery" --status
cat rebar_training/artifacts/mobile_curriculum_20261020/course_v2_recovery/curriculum.status.json
```

课程已用冻结源码启动，入口命令记录在 `course_v2_recovery/launch.json` 和 `plan.json`，不要重复执行。
`launch.json` 包含课程父进程 PID/PGID，各 `*.job.status.json` 包含子作业的 runner PID/PGID。
若用户后续要求暂停，先核验命令行归属并停止课程父进程组，再停止其正在运行的子作业进程组，
防止父进程重新调度；保存状态、权重、完整日志和相机包，核对无本任务残留进程后再标记暂停。
当前没有新的暂停指令。

新计划顺序：抓取放槽训练→两组各 4 回合冻结评估；到站再抓取夹紧训练→两组各 4 回合；
最后连续任务训练→八组各 8 个完整连续回合。种子、初始化权重和具体计划均保存在新课程 `plan.json`、
`provenance.json`；最多两个评估作业并行，`active_jobs` 列出实际正在运行的作业。
`needs_analysis` 必须分析修复，不能绕过 gate，也不能把后台启动当作验收完成。
整链 gate 通过后自动另录 `full_policy_demo.mp4`；示范回合是 regression，不进入独立计数。

需要另建恢复课程时，使用新输出目录并显式 `--initial-checkpoint <可信检查点>`，计划必须预声明新种子，
初始权重会复制进课程并记录 SHA；仍重跑全部阶段，不以旧版本通过跳过 gate。
独立评估默认拒绝训练种子；开发回归须加 `--regression`，其报告不能进入稳定性验收。

每次启动前确认 GPU/内存和已有作业；`mobile_jobs` 自身不会替你限制并行作业数。
保持 120 Hz 物理、30 Hz 控制、10 Hz 相机；Hydra 参数必须是 `env.sim.render_interval=12`。
不要误用 `sim.render_interval=12`，必须带 `--enable_cameras`（作业 helper 已自动添加）。
GPU 仿真异常也可能 exit 0，以完整凭据为准。

## 测试、备份与迁移

恢复后节点全套 CPU 接口测试 182 项通过；纹理版本 Mac 全套 178 项通过、3 项跳过，
随后新增补点事务性测试另通过，节点相关 22 项通过。准确结果见 `resume_20261020/`；
`handoff_tests.json` 和同名 `.log` 是上次暂停时的历史结果。
这些是接口/离线测试，不能替代 GPU 物理回合验收。

本机已保留此前完整基础包：
`rebar_training/artifacts/rebar_training_transfer_20261020_visual_mobile.tar.gz`
（1190537163 bytes，SHA-256 `ed1b5af118aef3f6543ecfed1b9dfb8f2f984fd70308b7fd9e00986d1c9dc86f`）。
它包含完整移动模型依赖、识别器、既有策略、报告和录像；不包含本轮最新修改。
本轮另制作 `rebar_training/artifacts/rebar_training_handoff_20261020_delta.tar.gz`，
包含上次暂停时的训练源码、配置、测试、脚本、文档和当时产物/快照。
对应 `.sha256` 和 `.verify.json` 保存校验；基础包先恢复，再将增量解压到同一仓库根目录。
**增量包不能独立替代完整模型基础包，也不包含 SDK、Python 环境、SSH 凭据。**
本机当前仓库已经具有这些文件，直接接力时无需重新解压覆盖工作区。

恢复后的纹理修复另存 `rebar_training_resume_20261020_texture_fix.tar.gz`（502478 bytes，
SHA-256 `c25c09d9f8e8ca069b7ee6774db70aa8acc3c5020c0b1ca0ee2930628e9e8a7c`，126 文件校验通过）。
这是代码及已完成到站回归的增量快照，训练状态为 active；不包含此后更新的检查点或全部原始相机帧。
实际学习检查点另外保存在 `resume_20261020/checkpoints/`，这些是旧恢复点；当前课程权重在 `course_v2_recovery/`。

移动资产依赖 `assets_full_v2_precision` 和 manifest 所列原始源 USD、托槽 USD、纹理；不要只拷一个 robot.usda。
换电脑后重新设置 `ISAACLAB_ROOT`、`ISAAC_SIM_ROOT`、`REBAR_TRAINING_PYTHON`、`REBAR_REPO_ROOT`；
作业 helper 也支持 `--lab-root --sim-root --python --assets`。不要把旧机器绝对路径直接当成新机器可用路径。
每次 GPU 长训/验收后，把检查点、完整日志、报告、源码 SHA 与失败图像同步回本机，再更新恢复包。

任务完成标准是：声明范围内的冻结策略确实通过独立整链 gate，失败原因有凭据和修复，
用户能取到权重、报告与真实运行视频。当前尚未达到；接力应从上方未解决问题继续。

本轮最终抓取放槽训练检查点及最新文档另打包为 `rebar_training_resume_20261020_tray_checkpoint.tar.gz`；
配套 `.sha256`/`.verify.json` 逐文件验证。该增量含冻结源代码、权重及报告快照，
不包含全部原始相机帧或 SDK；训练继续 active，后续权重以当前状态文件指向的课程目录为准。

本次恢复已另存 `rebar_training_recovery_20261020_v2.tar.gz`，源代码、冻结计划、warm start 与新增学习权重、
失败 gate、5/5 回归和恢复记录共 383 文件校验通过；新课程仍 active，未通过最终稳定门槛。
后续 `rebar_training_recovery_20261020_v2_handoff.tar.gz` 包含更新后的明确接力说明与最新课程快照。
