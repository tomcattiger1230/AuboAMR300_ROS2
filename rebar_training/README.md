# 钢筋抓取与试验机上料训练

## 当前：同一冻结策略的独立验收（2026-10-22 上海23:54）

修复开发回归已 **11/11完整成功、0失败、零证据错误**；1557个完整原始文件两端SHA一致，两个旧失败案例与真实30帧失视安全恢复均独立复核通过。保持source8a24、393b策略+optimizer及466updates/170actor/36602steps，不PPO重训、不重置策略或optimizer。

阶段门槛：抓取8/8独立通过；到站8/8独立通过。技能与开发回归均不计正式64。

当前阶段`full`：完成6、成功6、失败0，其中完整原生组0行；该阶段gate尚未独立复核通过。280个不可变快照文件两端SHA及Mac逐行真实物理/handoff复算通过。
tray_holdout_9131042：4完成/4成功/0失败，completed，tray_retreat/route4。
tray_holdout_9141042：4完成/4成功/0失败，completed，tray_retreat/route4。
dock_clamp_holdout_9151042：4完成/4成功/0失败，completed，loading/route10。
dock_clamp_holdout_9161042：4完成/4成功/0失败，completed，loading/route10。
full_holdout_9241042：3完成/3成功/0失败，running，observe_tray/route4。
full_holdout_9251042：3完成/3成功/0失败，running，observe_tray/route4。

正式整链为 **6/64完成，成功6、失败0**，不与技能/开发或历史混算。冻结计划e7381e2f…：抓取9131042/9141042各4，到站9151042/9161042各4，最终9241042..9311042间隔10000各8；演示9321042。仅到站技能gate允许托槽/到站复位、无录制视觉hint，不计64；正式64均从源抓取开始，无初始化/hint。阶段完整报告、全原始备份及Mac严格物理/官方gate独立复算通过并写review凭据后才晋级，当前原作业保留，最多2仿真。调度器是一次性工具，不会后台自行派发后续阶段。

目录`/home/arnold/Develop/rebar-training-resume-0103-20261022/rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022`，inhibitor PID/PG26082仅在有限评估生命周期阻止idle，暂停时也须停止；不改全局网络、电源或SDK。12.7GB历史归档已两端SHA通过，未解压覆盖运行目录。旧v10封存26/64（24成功2失败）、v8完整16另2成功、v9到站4成功/full0保留分列。仅仿真，不ROS硬件/飞书，不动原模型/共享SDK或无关GUI地图。入口`reports/source_return_frozen_acceptance_0103_20261022.json`和主status.current_job_root；旧候选记录助手已禁止覆盖新阶段状态。

## 最新：103已恢复连接，同一候选继续物理回归（2026-10-22 上海20:21）

已核查原runner8825和8879，未重复启动：候选目前 **5/11完成、5成功0失败**，其中7271042连续2回合原生完整完成，7241042连续8回合组已完成3回合、第四回合pickup。全部5行真实四爪/打开卸载/退让/速度/无臂接触、连续阶段和实际handoff同帧视觉与守卫在Mac独立复算通过，误差均在3mm以内；完整2回合组320个原始/源码/日志文件两端SHA全部一致。仍不是11回合gate，724原失败episode5尚未运行至。

空出名额后一次性派发器13630严格核验2回合组后启动预声明241042受控失视（runner13642/native13643/GPU13655），派发器已退出；当前与原8回合组GPU8838共两个仿真，保留健康作业、不重复派发。当前8a24源码、393b策略+optimizer及466/170/36602计数保持不变，无新学习；CPU251通过、CUDA52张量有限。原始USD依赖审计通过，SDK自带OmniPBR材质路径已定位并保存只读闭合凭据，未改共享SDK/资产。

此前断联已恢复；完整12.7GB历史归档原传输断开后已续传，远端整体SHA仍待完成，不能宣称全量远端校验通过。本机原2/11开发成功和全部v8/v9/v10历史仍保留；当前连续组新输出重放同一候选，不从头训练、不因换电脑清空学习。v10封存26/64（24成功2失败）；v8完整16另2成功；v9到站4成功/full0，分别保存。

当前入口为主status.current_job_root和reports/resume_0103_same_candidate_20261022.json。旧4090和旧0103课程停止；禁止重启旧派发器或v10后续组。11/11严格原生完整成功、724episode5/727episode0实际handoff与受控失视恢复凭据全部复核才采用修复，再做冻结独立验收；纯硬件迁移不附加PPO重训。


## 最新：按用户确认迁回 103，接续同一候选（2026-10-22 上海 20:13）

用户已授权恢复并确认目标为 `arnold@192.168.0.103`（5090 Laptop）。新目录 `/home/arnold/Develop/rebar-training-resume-0103-20261022`，旧机器/旧课程保持停止。同一候选源码8a24、策略393b及optimizer，计数466updates/170actor/36602steps不变；CUDA52个模型与optimizer张量有限，CPU239+12通过。本次硬件迁移没有重置策略、optimizer、学习计数或添加PPO训练。

两组连续候选回归启动回执为runner8825（8回合7241042）、8879（2回合7271042），输出 `review_20261020/source_return_candidate_v2_regression_resume_0103_20261022_run2`。随后SSH三次超时、目标ping0/2而路由器2/2，**当前native阶段、完成/成功/失败数未知**，不能把启动回执当健康运行或通过；恢复连接先查现有进程/原生JSON/进度/日志和GPU，禁止重复启动。第三241042受控失视尚未派发；run2一次性派发器尚未启动。首次0103启动因迁移子集遗漏原始USD依赖而在资产审计阶段退出（零物理回合），证据保留；已补齐并逐文件核对全部原始资产，SDK材质路径闭合复核待连接恢复。

4090候选原先2/11成功0失败继续保留；因没有序列化完整仿真状态，中断的8/2连续开发组在新输出重放，仍是同一策略/候选/种子，不假称跨进程物理状态连续，也不从头重训。旧v10封存26/64（24成功2失败）；v8完整16另2成功；v9到站4成功/full0，均保留分列。完整12.7GB历史包本机逐文件校验通过，向103的归档传输尚未确认完成/远端SHA，不得标记远端全量备份已通过。

恢复入口为 `reports/resume_0103_same_candidate_20261022.json` 与主status的current_job_root。自动跟进按用户恢复授权继续，断联期间只核查，不重复启动、不修改SSH身份。全部11回合严格物理及两例真实handoff/受控失视凭据通过后再采用修复和独立验收；硬件迁移本身不要求新版本或再次训练。


## 当前暂停：准备迁移至5090D（2026-10-22 上海16:37）

**用户已要求暂停全部新内容。候选模拟、一次性派发器49758、旧STOP课程worker20758及父20756已全部退出，GPU空闲；automation-2已PAUSED。未收到新明确恢复授权前，不启动训练、评估、回归、演示或派发。** 旧STOP worker直接SIGKILL退出，未SIGCONT，避免恢复旧派发。

- 候选v2封存 **2/11完整成功、0物理失败**；两组第二回合均为用户中断，不计成功或失败，第三受控失视未启动。历史失败727首回合复验成功，724原失败第六回合尚未到达；11回合gate未通过，不宣称修复完成。
- 当前冻结权重393b47f8含optimizer，466updates/170actor/36602policy_steps；canonical/source2c与隔离候选8a24e313均保存，检测器845f/资产d543不变。候选仍非正式部署代码。
- v10封存26/64（24成功2失败），原完整组16行，其余为中断组已完成进度；v8完整16另2成功、v9到站4成功和full0/64全部保留单列，不跨策略拼分母。
- 将完整当前仿真目录和原始日志/源码/策略/optimizer/相机概率/失败证据，与完整历史归档和实际演示视频一起打包，生成逐文件及整包SHA。本包不包含共享Isaac SDK/IsaacLab、虚拟环境、SSH身份或Git凭据；附版本/依赖说明供目标独立核查。
- **硬件迁移本身不重训、不创建新策略版本、不清空已完成记录。** 新机校验后保留相同冻结身份；中断回合不具备完整仿真状态恢复点，不能伪称连续运行。当前未完成的8/2连续开发回归恢复时用新输出重做相应连续组（原开发种子可复用），原2成功证据继续保留，不自动更改权重。

迁移说明见`MIGRATE_5090D_20261022.md`，停止凭据`review_20261020/migration_5090D_20261022/pause.stopped.json`。目标5090D的SSH地址待用户提供；不猜测为原5090 Laptop，不覆盖任何旧目录。

迁移包已完整构建并逐文件核验：`artifacts/rebar_training_migration_5090D_20261022.tar.gz`，12,698,828,724字节、12000数据文件（含清单共12001文件），SHA256 `d8f0a99269cad0a58a953ccdf03ee3118b21f8d4087d598c0ee8aa8d7fe4f753`。当前目录11976原始文件两端SHA一致，完整历史归档f1b47a58已核对。包外`.sha256`及`.verify.json`是最终完成凭据，包内元数据为打包前暂停快照；目标地址待提供，所有任务仍暂停。


## 当前任务：优先候选v2的11回合物理回归（2026-10-22 上海16:31）

用户要求先保存并停止原v10两组，优先修复验证。15:54已仅对原726/727进程组38757/39556发SIGTERM并确认全部退出，无需SIGKILL，GPU释放；派发worker20758继续SIGSTOP，禁止恢复或派发728+。这不是暂停整个任务，候选验证已获继续授权。

- v10封存结果为 **26个已完成回合：24成功、2物理失败**。首两组完整原生报告16回合（15成功1失败）；后两组各完成5回合，分别5成功0失败、4成功1失败。两组进行中的第六回合记用户中断，不计成功或物理失败。原固定64已无法达到Wilson门槛，不再继续。
- 原两组全部相机/概率/关节接触、源码、原生进度和完整日志等 **1283文件已同步Mac并逐一SHA一致**，目录`course_v10_4090_return/stopped_for_candidate_20261022_155457`；停止凭据`stop.proof.json`，全部失败保留，不修改原生报告。
- 15:55在新输出`review_20261020/source_return_candidate_v2_regression_20261022`启动连续8回合7241042（runner46837）及连续2回合7271042（runner46911）。16:31已完成 **2/11回合：2成功、0失败，本轮新增0**；实际source8a24、393b/checkpoint、845f/d543一致，27个快照文件两端SHA及Mac逐行真实物理/交接守卫独立复算通过。当前阶段：7241042 step15450 navigate/route4；7271042 step15450 navigate/route4。历史失败727首回合复验通过=True；724原失败第六回合复验通过=False（未运行至该处不能宣称通过）。尚无完整11回合gate；第三241042受控失视1回合待真实名额空闲才启动，最多2仿真。
- 一次性派发器49758已运行（仅CPU等待，不占仿名额）：只有连续2回合组原生完整成功、独立物理/交接条件复核及实际仿真名额小于2时，才自动启动预声明的241042受控失视1回合；成功派发即退出，失败则记录needs_analysis。状态`outage_dispatch.status.json`。后续人工/自动跟进须先查此状态和同名job，不能重复启动；若用户暂停整个任务也须停止该派发器。

- 所有11回合必须完整严格成功，两个原失败案例须有真实source_return_handoffs触发及同帧RGB/概率/K/TF和打开无载、净空等条件凭据；受控失视须真实触发并在支撑/打开无载/已返回后半速新观察、最多2次。保持3mm/3°、250N、实际四爪、打开卸载、至少60mm退让、速度及0.8mm门控，禁止真值运行时输入。开发回归不计正式64。
- **换电脑本身不要求新策略版本或再训练**：同一冻结源码/策略/检测器/资产在迁移校验与物理回归后可以继续原课程。此前迁移后又进行了PPO学习，fe48→f70f→393b权重发生变化，故不能合并旧成绩。后续纯硬件迁移保留课程身份、权重和预声明分母；修复控制源码则须另行冻结及独立验证。
- 历史v8 fe48完整组16回合另2成功，共18成功0失败；v9到站4成功0失败、整链0/64。v10抓取/到站gate均8/8及训练2/2仍为历史能力凭据，不拼入候选/后续新源码验收。

当前恢复入口为`review_20261020/status.json`的`current_job_root`和`candidate_regression_directory`，详细见`reports/source_return_candidate_priority_20261022.json`。不要使用旧v10进度同步助手恢复“原两组继续”状态，也不要重启旧validator、旧平台或原v10模拟。

## 历史v10修复等待记录（已由上方停止与候选启动覆盖）：4090 / v10 已转源返回修复，原两组继续留证（2026-10-22 上海 15:39）

用户恢复授权有效，唯一新目录 `/home/arnold/Develop/rebar-training-4090-return-20261021`，课程course_v10_4090_return。首两组7241042、7251042现已各完成原生8回合，分别7成功1失败、8成功0失败；原runner30122/30121已结束，不重启。原controller20756已自动推进7261042/7271042各8回合，runner38757/39556（以实时active_jobs为准），未手动增加或重启仿真。

- 整链训练已原生完整 **2/2成功、0失败**，误差 **1.544/1.518mm**，轴0.049814/0.023461°、四爪约23.820–26.092N、unsafe峰26.092/26.051N、松爪卸载、90.786/89.966mm退让及稳定速度严格复核；两回合均验证抓取放槽、观察托槽、导航、到站、再抓取及加载连续阶段。无托槽/到站初始化、无视觉hint/regression。**291完整原始/源码文件两端SHA一致**。
- 从a1124cf1的395updates/147actor/31681steps开始，实际新增 **71PPO/23actor/4921有效步**，最终466updates/170actor/36602steps，actor最大变化0.01233708，model与optimizer有限；超过60新更新门槛。最终full_policy SHA`393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82`。完整report/checkpoint466/170为准，training进度465/169是末尾buffer刷新前记录。
- 当前新版独立整链进度 **24/64完整回合，成功22、失败2，较上轮新增0**，各组7241042 8/8、7251042 8/8、7261042 4/8、7271042 4/8。本轮无新增完整回合，继续核查原有回合的阶段进度。91个不可变部分快照文件两端SHA一致，入口`full_holdouts_snapshot_20261022_153955`。实时阶段：7261042 step44580 loading/route4，RGB有效、last_error空；7271042 step38790 pickup_tray/route0，RGB有效、last_error空。原生两组保留运行；派发worker20758保持SIGSTOP，不再派发728+；最终393b47f8 SHA未变。**完整组原生报告可计最终gate行数16，原固定64计划已无法达到门槛，转候选修复验证**；部分成功不宣称稳定。
- 完整原生组已核验 **16回合：15成功1失败**，节点与Mac独立复算一致、零证据错误，Wilson95%下界0.716712624，尚未达到64及0.90门槛。1819原始图像/概率/日志/源码/报告两端SHA一致，入口`completed_full_groups_verified_20261022_141045`，失败回合保留分母。
- 两完整组及失败分析/候选的恢复归档已校验：`rebar_training_recovery_20261022_v10_first_two_full_groups.tar.gz`，1972文件、1417322209字节，SHA`20f941f76575826c928f7bd85aee46cb83887c9bf455db153437549b48722ef0`，Mac与4090两端一致；依赖冻结策略/optimizer包fe0a57df，不解压覆盖运行目录。该包仅为当前16个完整组回合凭据及恢复材料，最终64未通过。
- 首例失败及历史v1候选（已由下方v2替代）：7241042第六回合原生判定`stage timeout: pickup`，计入分母；初始全长定位正常，空腕原路返回低位时源钢筋中心投影越出615px图像下边界，视觉失效后打开无载停住，3601步超时。残差在route0未启用，故不能以增加PPO更新解决。失败峰值41.666N、臂部接触为0，没有超力或真实夹取；原始回合及相机/概率/关节反馈已保留。隔离候选`source_return_handoff_v1`尝试在新鲜视觉及打开无载、返回路径末段时提前交给视觉伺服，**4090 CPU全套251通过，未部署、物理回归尚未通过**。不修改2c/393b现有评估，不拼接候选成绩。详细见`reports/v10_pickup_visual_failure_20261022.json`。
- 历史13:25的v1候选9回合开发物理回归预声明：7241042连续8回合（包含原失败第六回合）及241042受控部分抓取失视1回合，必须9/9严格通过并核查真实提前交接触发、连续复位和臂部接触；不计正式分母。当前两个仿真名额均被原有评估占用，**回归未启动**，不重复启动或增加第三仿真。入口`reports/source_return_regression_plan_20261022.json`。
- 第二失败与派发处理：7271042首回合也原生源抓取超时，打开无载、臂部接触0、峰41.799N；相机中源钢筋中心越出图像下界。两次失败后，固定64回合即使余下全成功也最多62/64，Wilson95%下界0.893027低于0.90，原v10计划已无法通过。**14:20仅SIGSTOP派发worker20758（launcher20756保留），726/727两个原生仿真未停止**；不恢复该派发器、不启动728+，原curriculum.active_jobs可能滞后，以native job/status及GPU核查。用户任务与自动跟进仍ACTIVE，转修复验证；策略、源码、旧证据不变。
- 候选升级为隔离v2（源码8a24e313），运动守卫与v1相同，增加真实触发条件和当帧RGB/概率/K/TF留证。新预声明开发回归11回合：724连续8、727连续2及241受控失视1；旧9回合计划从未启动，现由v2计划替代。候选v2已在4090 CPU全套251通过，物理回归未启动，不部署到冻结v10，也不拼分母。入口`reports/v10_source_return_failures_20261022.json`和`reports/source_return_regression_plan_v2_20261022.json`。
- 第二失败、派发暂停及v2候选的增量恢复包已两端核验：`rebar_training_recovery_20261022_v10_second_failure_repair.tar.gz`，192文件、73151644字节，SHA`5af601cf1133c02fb3f92b66fd29b4bf7aeaf80dd5d9a40a5e792b7da792f498`；依赖20f941f7完整两组包与fe0a57df冻结策略/optimizer包。当前v10失败保留，候选仍未物理通过，不解压覆盖运行目录。
- 64回合预声明为7241042至7311042间隔10000，各8回合、最多2并行；同一最终393b47f8策略、源码2c、检测器845f、资产d543、计划ef6828b0，评估期间不学习，不混用技能策略/开发回归/历史成绩。指纹入口`final_full_frozen_fingerprints.json`；后续累计须包括所有已完成组，不能只数active_jobs；完整原生报告才可进入最终gate。
- 抓取与到站独立gate均8/8通过，零证据错误，各自412/419完整文件已核验。迁移开发3/3、训练及技能gate不进入最终64分母。历史4090/v8 fe48完整组16/64另2成功，共18成功0失败；5090/v9到站独立4成功0失败/full0/64，继续单列保存。

下一步保持派发worker20758停止，仅保留当前7261042/7271042仿真运行，实际名额空闲后执行候选v2开发回归，核查新鲜完整JSON、全部连续阶段、真实四爪/打开卸载/退让/速度及冻结指纹；v2须11/11严格通过后才审计新种子并建立新课程，旧v10不再派发余下组、不拼接成绩。保持RGB机器人反馈、钢筋真值隔离、真实接触、0.8mm视觉门控、3mm/3°、250N及恢复预算所有判据；最多2仿真，仅runtime_python_4090补隔离overlay，不改SDK/原模型/无关GUI地图。至少64预定完整连续回合、至少2种子、成功率≥95%、Wilson95%下界≥90%、零证据错误后，才核验实际完整示范视频和全套备份并交付。

## 历史迁移校验：4090 已接收，当时训练暂停（2026-10-22 上海 08:49）

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

## 此前稳定性验收进度（2026-10-20）

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

## 上次暂停与接力入口（2026-10-20 07:05，上海时间）

用户当时准备离开训练节点所在网络，要求暂停。该轮仿真作业已停止，停止后通过进程列表与
`nvidia-smi` 确认没有残留仿真/GPU 训练进程。**不要根据下面的命令示例自动启动训练。**
下一位 agent 请先读 [训练接力指导](TRAINING_HANDOFF.md)，收到用户恢复指令后再继续。
新增源码、日志、失败图片、检查点及作业源码快照已从 `0.103` 同步到本机。
暂停记录为 `artifacts/mobile_curriculum_20261020/PAUSED.json`；它是操作记录，不是进程锁。

本轮已确认：偏远钢筋抬升卡死已修复，同种子抓取放槽 **1/1** 通过；
横向观察修复已取得有效视觉定位，但在返回抓取位途中被用户暂停，尚无完整回合结论。
残差 PPO 试运行只有 **4 次价值网络更新、0 个有效策略控制步**，不能视为训练完成的策略。
自动课程与 64 回合整链验收均**尚未启动**；指导文件记录奖励整形等继续训练前须处理的问题。

## 扩扰与残差 PPO 课程接口（2026-10-20）

新一轮在 `0.103` 的独立训练目录运行，记录于 `artifacts/mobile_curriculum_20261020/`。
`mobile_mission_check` 已支持同一物理进程内多回合自动复位、每回合原子进度报告，
失败回合保留在分母中。每个 GPU 作业使用冻结源码快照；本轮没有通过新的稳定性门槛。

| 随机化档位 | 料台/钢筋初始 XY | 初始偏航 | 手臂关节 | 底盘 XY / 偏航 | 图像 |
|---|---|---|---|---|---|
| baseline | ±30 mm | ±0.05 rad | ±0.02 rad | 0 / 0 | 原始灰度 |
| expanded | ±60 mm | ±0.10 rad | ±0.035 rad | ±20 mm / ±0.02 rad | 增益 ±10%、偏置 ±4、噪声 σ=1 |
| challenge | ±90 mm | ±0.16 rad | ±0.05 rad | ±40 mm / ±0.035 rad | 增益 ±20%、偏置 ±8、噪声 σ=2 |

料台与其上钢筋仅在回合复位时整体扰动，保持实际支撑；没有在运行中修改钢筋位姿。
图像增益/偏置按回合抽样，噪声逐帧生成。尚未加入质量/摩擦、尺寸或标定误差随机化。
首轮扩扰发现偏远位置的 0.9 m 原地抬升超出手臂工作空间，加入离台后的向内收回抬升后，
同种子复测在 2762 控制步（92.1 s）通过抓取放槽。初始视野裁端问题新增了横向观察候选，
其完整抓取回合已在恢复后的 v5 回归中 1/1 通过。

`mobile_residual.py` 是独立的纯 PyTorch 残差 PPO 实现，与历史 RSL 状态策略分开。
`mobile_visual_manipulation_residual_v1` 输入 76 维视觉估计、机器人反馈、载荷、任务阶段与已知目标；
输出六维关节速度修正，默认最大修正为原归一化速度指令的 0.08（即 0.028 rad/s）。
部署仍包含视觉伺服、观察运动、夹爪/阶段管理与里程计导航；不能称为独立整链神经策略。
初始均值为零修正；PPO 更新后保存网络、优化器、实际更新次数、有效策略步数及训练种子。
被观察引导或相机失效保持覆盖的动作不参加策略梯度，终止后优势估计不跨回合传播。

批量验证示例在上方移动命令中增加 `--episodes 4 --randomization expanded`。
课程训练增加 `--learn-residual --rl-iterations 40 --save-policy <新检查点路径>`；
恢复训练或冻结评估用 `--residual-checkpoint <已保存检查点>`，恢复训练须另指定新输出路径。
评估默认拒绝训练种子，已见种子的复测须显式标记 `--regression`。
`--start-on-tray --start-at-dock --until clamp` 是单独的到站夹紧课程，不能计入完整连续流程成功率。

`python -m rebar_training.mobile_acceptance <报告1> <报告2> --phase full --output <新报告>`
要求同一源码、视觉权重、资产与冻结策略、至少两个独立种子、64 个完整连续回合、
实测成功率 ≥95% 且 Wilson 95% 置信区间下界 ≥90%。阶段课程门槛为两个种子合计至少 8 回合。
训练回合、回归回合、缺失回合、托槽初始化和物理验收矛盾均不能通过整链门槛。
真实夹紧力、250 N 安全阈值、3 mm 实际误差和松爪退让判据保持原值。

## 当前视觉与移动进度（2026-10-20）

已在 `0.103` 接通腕部灰度识别和三维定位。运行控制只读 RGB、机器人反馈与原始接触载荷，
禁止访问钢筋/料台真值、奖励阶段或钢筋身份过滤力；真值用于复位、标签和独立评分。

- 完整测试机、固定底盘的视觉伺服 **4/4** 成功，最终实际误差 **1.02–1.60 mm**，保持 3 mm 判据。
- 视觉神经策略经 BC/DAgger 适配后，原失败种子回归 **4/4**、新独立种子 **4/4**。
  另一个新种子在相机中断 1 秒后恢复并成功交接，实际误差 1.38 mm；这些是小样本验证，未宣称百回合稳定性。
- 未固定底盘的“视觉抓取→旋转→放槽→松爪撤离” **1/1** 通过，无钢筋附着或运行中位姿写入。
- 独立托槽行驶/转向试验通过，最大相对位移 2.10 mm。
- 完整机柜场景的视觉载料导航与精确停靠课程 **1/1** 通过（101.8 s），最终槽内轴向位移 2.85 mm。
- 到站后再抓取/插入/夹紧课程 **1/1** 通过，实际误差 **1.90 mm**。
- **两组种子的完整连续流程 2/2 通过**：料台视觉抓取→放槽→载料导航→停靠→再抓取→插入夹紧→松爪撤离。
  seed 119042：9041 控制步、301.4 s，实际误差 **2.12 mm**；
  seed 125042：9199 控制步、306.6 s，实际误差 **2.47 mm**。3 mm 评分门槛未放宽。
  没有托槽/到站初始化、外部视觉提示、钢筋真值控制或运行中钢筋位姿写入。这仍是小样本验证。

当前识别器为本地训练的灰度 U-Net；**尚未部署或调用 Cosmos 3**。
详细输入假设、运行接口、实验边界与报告路径见 [视觉验证记录](COSMOS_OFFLINE.md)。
[本轮汇总报告](artifacts/visual_control_20261019/visual_mobile_completion_20261020.json)记录两组完整回合、源码/权重摘要及录像检查；
[GPU 验收环境版本](artifacts/visual_control_20261019/runtime_versions_0103.json)用于迁移复核。
新代码入口为 `visual_check`、`mobile_assets`、`mobile_check`、`mobile_mission_check`；
移动全链原型使用视觉伺服与里程计，不能称为 Nav2 避障或完整流程强化学习结果。
生成的移动资产依赖原完整机构资产；迁移时须一起保存 `assets_mobile_v2` 与 `assets_full_v2_precision`。
还须保留 manifest 所列源 USD、托槽 USD 与纹理；`scripts/build_transfer.py` 已覆盖移动托槽依赖并核对哈希。
当前视觉权重为 `artifacts/visual_control_20261019/segment_v1.pt` 和 `visual_bc_v4_dagger.pt`。
相机中断恢复录像为同目录 `vision_policy_123042_v4_outage_video_v2.mp4`，约 79 秒。
完整移动录像为 `mobile_full_125042_v7_recording.mp4`，306.6 s，10 fps；同名 JSON 保存独立评分与全部阶段记录。
旧 v6 回合的评分有效，但录像全景未跟随移动；已修复独立记录相机并用新种子重新录制 v7。

启用相机的移动整链验证（仿真，报告/录像必须使用新路径）：

```bash
REBAR_TRAINING_ASSETS="$PWD/rebar_training/artifacts/assets_mobile_v2" \
  bash rebar_training/scripts/run_isaac_python.sh -m rebar_training.mobile_mission_check \
  --headless --enable_cameras --until clamp --seed 119042 \
  --detector rebar_training/artifacts/visual_control_20261019/segment_v1.pt \
  --camera-config rebar_training/config/vision_runtime.json \
  --report rebar_training/artifacts/mobile_fresh.json \
  --video rebar_training/artifacts/mobile_fresh.mp4 env.sim.render_interval=12
```

先配置本机 `ISAACLAB_ROOT`、`ISAAC_SIM_ROOT`、`REBAR_TRAINING_PYTHON`，见下文运行时准备。
相机和视频需要本模块 `vision`、`video` 两组可选依赖；视频编码已在 imageio 2.37.2 / imageio-ffmpeg 0.6.0 验证。
取料区与试验机坐标已测定，初始底盘已在取料区；钢筋精定位来自图像。
当前移动部分为已知静态工位间的里程计伺服，不包含未知钢筋搜索或动态避障。
该已验收运行采用 120 Hz 物理、30 Hz 控制、10 Hz 相机/渲染；视频全景只用于诊断。

## 历史状态策略验收（2026-10-19）

用户已授权在新 Ubuntu 节点 `arnold@192.168.0.103` 上直接通过 SSH 恢复仿真验证与训练，
不再经过飞书。实测显卡为 RTX 5090 Laptop 24GB，CUDA/PyTorch 运算正常。
独立训练目录为 `/home/arnold/Develop/rebar-training-0103`，
Python 环境为 `/home/arnold/Develop/rebar-training-runtime`，
Lab 位于 `/home/arnold/Develop/IsaacLab-rebar`，Sim 沿用 `/home/arnold/isaacsim`。
现有 ROS 项目未修改。新机资产/运行时预检查通过，基础测试 101 项、
增加阶段采样、策略结构、PPO 恢复与导出检查后完整测试 108 项通过。
`loading_route_v1` 专家验证成功 128/128 回合、自动复位正常；这不是自主策略验收。
迁移的独立抓取、独立交接策略均在两组随机种子各通过 100/100 回合。
后续课程采集、迁移复测与训练记录位于
`rebar_training/artifacts/migration_0103_20261019/`。完整流程已通过当前仿真工位的稳定性门槛：

| 最终自主评估 | 完整成功 | 抓取抬升 | 掉落 / 超力 / 超时 | 最大最终位置误差 |
|---|---:|---:|---:|---:|
| seed=10042 | 100/100 | 100/100 | 0 / 0 / 0 | 2.193 mm |
| seed=20042 | 100/100 | 100/100 | 0 / 0 / 0 | 2.091 mm |

已完成 600 轮阶段策略拟合及 1500 次 PPO 更新；两组最终评估均只运行固定神经网络，
没有专家动作混入，包含自动复位后的重复任务。原有物理和成功判据未放宽。
权重及验收报告已备份到本机，任务完成后没有继续训练。
总报告为 `artifacts/migration_0103_20261019/training_completion_report.json`，
可恢复的完整权重为 `artifacts/migration_0103_20261019/final_loading_policy/model_1499.pt`。
同目录保存运行参数、`policy_jit.pt`、`policy.onnx` 及其外部权重 `policy.onnx.data`；
ONNX 文件须与外部权重一起保留。两种导出均已与原模型核对，覆盖 10 个实际任务阶段。

完整策略原速视频为 `artifacts/migration_0103_20261019/policy_video/rebar_rl_loading_zh.mp4`，
约 35 秒，含中文阶段字幕；对应 `video_delivery_report.json` 记录模型 SHA、编码和真实回合结果。
录像使用导出的冻结 RL 策略，seed=30042，完整上料成功，最终位置误差约 1.46 mm，
没有专家动作、掉落、超力或训练更新。`python -m rebar_training.record_policy` 可在启用
`--enable_cameras` 的 Isaac 环境复录，并要求提供已核对导出报告的 JIT 策略。

以下为本轮修复前的失败记录：首轮完整流程策略因接近阶段超力失败。阶段均衡采样后 BC 已能 32/32 完成抓取抬升，
但后续搬运/交接未通过；其后 DAgger 独立验收 0/32，主要在下降和搬运到位处停滞超时。
混合控制中的成功回合不计为自主成功。

`route_heads_v1` 神经策略首次独立验收（seed=90042）成功 **32/32**，全部实际抓取抬升，
没有超力、掉落或超时；随后完成上述 PPO 长训和最终验收。观测及七维关节速度/夹爪动作接口保持
`loading_route_v1`，按实测阶段选择独立网络头，去除上一拍动作和指令目标的输入捷径。
检查点显式记录网络类型和尺寸，评估按检查点选择结构；旧 MLP 权重仍可读取。
该策略只使用状态观测，采用既定工位路点；尚不是相机输入或无路点的端到端规划。
课程入口支持 `--initial-checkpoint` 与 `--initial-replay` 恢复已有拟合结果；
恢复后仍须重新通过独立 32 回合门槛，才进入 PPO 长训。

## 原 4090 节点的历史记录

这是现有机器人/钢筋资产的独立 Isaac Lab 训练模块。2026-10-19 在
`192.168.3.133` 的 RTX 4090 上完成环境准备、真实接触验证和独立技能长训。
**原节点曾按用户要求暂停，历史暂停报告保留。** 抓取、独立交接均完成 1500 次 PPO 更新，
分别在 seed=10042、20042 各成功 100/100 回合。连续完整流程尚未达标。
现有 ROS 2、MoveIt、机器人 USD 源文件保持原样。本轮没有接入 Cosmos 3。

## Cosmos 3 离线相机与远程调试准备

[采集数据格式、扩增请求与 Hermes/OpenClaw 调试接口](COSMOS_OFFLINE.md)已加入本模块。
这是原暂停阶段的接口准备。当前相机闭环已完成上方列出的 GPU 验证；Cosmos 服务仍未接入，
原 3.144→3.133 的飞书只读通道已验收，新 0.103 节点采用直接 SSH。

[3.144 Hermes / OpenClaw 只读调试部署步骤](REMOTE_DEBUG_DEPLOY.md)包含专用 SSH 密钥、
固定接收入口、两个原生插件和飞书验收方式。该通道针对原节点，不用于新节点的训练启动。

## 版本与环境

| 项目 | 固定目标 |
|---|---|
| Isaac Lab | `v3.0.0-beta2.patch1`，启动入口检查 Git 提交 |
| Isaac Sim | `6.0.1`，对应现有项目记录 |
| Python | `3.12`；与仓库根目录的 Python 3.14 环境分开 |
| 物理后端 | Isaac Sim PhysX；本模块不支持切换 Newton/OVPhysX |
| 强化学习 | RSL-RL PPO，可选学习先验回放；课程采用 BC + DAgger |
| 首次上线设备 | Ubuntu + RTX 4090，先 `num_envs=1` |

版本依据：[Lab 发布说明](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-beta2.patch1)、
[固定版本安装文档](https://isaac-sim.github.io/IsaacLab/release/3.0.0-beta2/source/setup/installation/index.html)。
这个标签的安装方式与当前 develop 不同，请按固定版本配置独立环境，保留已有演示环境。
不在仓库根目录安装 Isaac/CUDA 依赖。

## 离线准备与检查

以下命令在仓库根目录执行；需要首次联网安装 CPU Python 依赖，执行时不需要 NVIDIA GPU。

```bash
uv sync --project rebar_training --extra offline --extra vision --extra video
uv run --project rebar_training --extra offline python -m pytest -q rebar_training/tests
uv run --project rebar_training --extra offline python -m rebar_training.preflight \
  --build --report rebar_training/artifacts/offline_preflight.json
```

`artifacts/assets/` 生成 robot、station、rebar、machine、fixture 五个训练 USD，
机器人展开成本地快照，其余资产保留对源文件的相对引用。`manifest.json` 记录源 USD 层的 SHA-256 与目标版本；
源层改变后，需要重新 `--build`。生成资产、报告、日志和权重不入 Git。
如果只安装了项目根目录的开发环境，训练测试会因缺少 Torch/USD 而跳过；
上述独立环境的测试才是本模块的完整离线检查。

资产检查覆盖：关节名称、单一机器人 articulation root、钢筋圆柱碰撞形状与尺寸、
钢筋动态状态、碰撞开启、无连接钢筋的固定关节、四个物理夹持垫。
原资产中未使用的云端雷达及三处无效外观引用在训练快照中移除，避免 Kit 启动时等待云端资源。
构建时 OpenUSD 可能打印源文件的五处警告；最终训练快照的组合检查应没有这些警告。
钢筋用 Xform 容器下的 Cylinder 刚体，避免 USD 加载器覆盖根圆柱类型。
腕部与固定电机外壳有重叠，训练快照仅对这一对设置碰撞排除，其余自碰撞保留。

## 训练任务

| Gym ID | 起点与成功条件 |
|---|---|
| `Aubo-Rebar-Grasp-Direct-v0` | 已停靠底盘、张开的夹爪位于取料台上方；双指接触钢筋，抬升至少 0.12 m，稳定保持 0.5 s |
| `Aubo-Rebar-Insert-Direct-v0` | 已竖直持筋并接近试验机；对齐后物理夹持垫夹紧，机器人松爪并撤离至少 0.06 m，钢筋由试验机稳定保持 1 s |
| `Aubo-Rebar-Loading-Direct-v0` | 从取料台实际抓取、抬升，经持筋转运和竖直化后完成上述交接；必须同时满足抓取和交接判据 |

插入任务从“已持筋”的物理初始状态训练。新增 `Aubo-Rebar-Loading-Direct-v0` 连续任务，
从取料台开始，在同一回合内完成抬升、转运、竖直化和交接；中间不复位钢筋。
为让两种设备同时存在且不重叠，连续工位的试验机中心配置为 `(-0.30,-0.85,0)`、yaw=-90°，
夹持目标为 `(-0.30,-0.77,1.50)`；保留原模型尺寸。独立插入任务保留原布局。
当前复用已有轨迹作为起点：抓取来自 `rebar_grasp_20260916.json` 的 pregrasp IK，
插入基于 `rebar_tester_mission/compact_curve.py` 的终点分支做 IK 调整，
将起始 TCP 移到约 `(-0.84, -0.08, 1.50)`，避开实测到的上臂与台面碰撞。
插入初始钢筋姿态根据复位后腕部 FK 计算；初始化时允许写入位姿，
之后只受重力、摩擦、碰撞和机器人/夹持垫驱动作用。
不运行 `IsaacRebarTesterController`，不创建机器人到钢筋的固定关节，
不把钢筋设为 kinematic，也不在抓取/交接时关闭钢筋碰撞。

训练试验机保留现有机架模型，用四个可驱动滑动夹持垫代替演示中的外观抱爪。
夹持垫的尺寸来自原 jaw tip，驱动力、速度与摩擦是待标定参数。
这是一套用于接触学习的夹具近似，不包含拉伸试验、钢筋变形或拉断模型。
底盘固定，只训练手臂操作；连续任务包含持筋竖直化，移动底盘导航尚未加入。

## 任务接口

统一使用每个克隆环境的局部坐标，Z 向上，位置 m、角度 rad、力 N。
Lab 3 数据及复位接口的四元数顺序为 **XYZW**；USD 的 Gf 四元数由资产脚本转换。
TGS 物理配置启用了 `enable_external_forces_every_iteration=True`，避免静止持筋时残余速度读数阻止稳定判据。

动作 `[N,7]`，每个分量限幅到 `[-1,1]`：

- 前 6 维按 `shoulder, upperArm, foreArm, wrist1, wrist2, wrist3` 顺序控制关节目标速度，初始上限 0.35 rad/s。
- 第 7 维控制双指同一绝对闭合目标：`-1` 为张开 0 m，`+1` 为闭合目标 0.0285 m（源 USD 关节上限）。
- 积分关节目标受 URDF/USD 限位与实测位置前方 0.12 rad 的目标裕量约束，防止目标积累。
- 夹持垫在对齐稳定 0.3 s 后自动闭合，属于模拟试验机的控制逻辑，不属于策略动作。

策略观测 `[N,71]`，顺序如下：

| 字段 | 维数 |
|---|---:|
| 8 个受控关节的位置归一化、速度 ×0.1 | 16 |
| TCP 位置、工具轴、夹爪闭合轴 | 9 |
| 钢筋中心、轴向、线速度、角速度 | 12 |
| 独立技能：目标相对钢筋的位置；连续任务：当前路径目标相对 TCP 的位置 | 3 |
| 双指/四垫对钢筋的接触力 ÷100 | 6 |
| 四垫关节位置 ÷0.075 | 4 |
| 阶段 one-hot、任务进度（连续任务含明确的操作子阶段） | 6 |
| 上一个动作、当前积分目标 | 15 |

连续任务采用 `loading_route_v1`：后三项任务进度为路径子阶段 /10、双指接触计数 /8 和交接验证进度。
`route.py` 根据实测位姿、双指接触、抬升保持、关节位置及四垫支撑推进子阶段，输出当前目标供策略观测，
不会驱动关节或写入钢筋位姿。七维机器人动作始终由 actor 输出。
该接口修复了原连续策略在下降/闭爪/抬升之间的历史状态不明确问题；原 `state_v1` 连续权重不能加载到新接口。

首版使用仿真真实状态，属于状态策略，不依赖相机或 YOLO 权重。
Cosmos 视觉增强、相机到三维位姿估计和视觉策略蒸馏留作后续接入，
目前没有把生成视频当成物理轨迹或直接用于 PPO 在线循环。

## 自动复位

独立技能每轮最多 12 s，连续任务最多 60 s；物理步长 `1/120 s`，每 4 个物理步更新一次动作。
成功、失败和超时后由 DirectRLEnv 自动复位对应环境，不重启 Isaac Sim。

- 抓取：整个取料台在 XY 各 ±3 cm、yaw ±0.05 rad 内随机移动；钢筋随托座移动，保持正确支撑高度；起始臂关节扰动 ±0.02 rad。
- 插入：起始臂关节扰动 ±0.005 rad，钢筋按腕部 FK 放入夹爪，双指带预紧目标；所有试验机夹持垫重新张开。
- 清空钢筋/关节速度、上一动作、积分目标、阶段、连续保持计数和超时计数。局部复位不修改其他环境的运动状态。
- 复位采样使用独立 Torch generator；相同 seed、相同复位顺序可重现采样。物理轨迹的确定性仍需 GPU 验证。

当前先固定钢筋为直径 24 mm、长度 0.6 m；质量和摩擦沿用源资产。
改变尺寸前需要同步更新托座、夹具及判据，首版不对这些参数任意随机化。

## 奖励和终止

判据和奖励只在每个控制步的 `_get_dones()` 中更新一次；
`_get_rewards()` 读取缓存，避免保持计数被重复推进。
核心实现 `core.py` 同时用于 CPU 测试和 GPU 环境。

- 抓取有效：两指对钢筋的过滤接触力均 ≥0.5 N，TCP 与钢筋中心距离 <45 mm。
- 抓取成功：抬升 ≥120 mm，线速度 ≤0.04 m/s、角速度 ≤0.2 rad/s，持续 0.5 s。
- 插入对齐：中心误差 ≤8 mm、轴向偏差 ≤3°、速度满足上述稳定条件，且仍由双指持有，持续 0.3 s。
- 交接成功：四个夹持垫都实际接触钢筋；机器人双指位置 ≤3 mm 且无有效钢筋接触，TCP 撤离 ≥60 mm；钢筋位置/姿态/速度连续 1 s 满足要求。
- 失败：钢筋比初始中心低超过 100 mm、监测力 >250 N、输入出现 NaN/Inf 或轴向无效、存在辅助保持标志。
- 超时记为 `truncated`，不会记作成功；同一步出现失败与成功条件时，失败优先。

抓取阶段分数是接近奖励、双指接触奖励和抬升进度之和；
插入阶段分数是中心/轴向对齐、当前持有方接触及交接验证之和，最高均为 8。
每步奖励为 `dt × (分数 − 8) − 动作代价 − 动作变化代价 − 0.01`，
成功另加 20；失败扣除 `10 + 剩余步数 × 最坏单步代价上界`，默认上界约 0.9067。
这项终止代价避免策略通过提前掉筋或碰撞来省去后续运行代价。
非正的中间运行奖励避免停在中间姿态刷奖励。
阈值是初始训练配置，不能当成已验证的实际设备安全限值。
连续任务的峰值高度掉落检测只在钢筋既未被双指实际抓持、也未被四垫实际支撑时生效，允许持筋过程中的正常下降。

## 4090 环境与执行顺序

133 已配置独立环境：`$HOME/Develop/IsaacLab-rebar/.venv`，
通过 `scripts/run_isaac_python.sh` 加载 `$HOME/isaacsim` 的二进制运行库。
已有 Sim 环境保持原样；训练环境使用 Torch 2.11.0+cu130、RSL-RL 5.0.1。
机器是 Ubuntu 26.04.1，现有 Sim 为 `6.0.1-rc.7` 二进制；该组合的兼容范围以本次实测为限。

其他机器先按固定标签的官方文档配置 Isaac Sim 6.0.1 / Lab 的 Python 3.12 环境，
激活该环境；下面的 `python` 必须属于该环境。不要激活上面的 CPU 离线环境。
设置实际路径，保持 Lab 为未修改的固定标签检出：

```bash
# 在本项目仓库根目录运行
export REBAR_REPO_ROOT="$(pwd)"
export ISAACLAB_ROOT="$HOME/Develop/IsaacLab-rebar"
export ISAAC_SIM_ROOT="$HOME/isaacsim"

# 133 的 binary 安装使用这个函数；wheel 安装可直接使用环境内的 python。
rebar_python() { bash "$REBAR_REPO_ROOT/rebar_training/scripts/run_isaac_python.sh" "$@"; }

git -C "$ISAACLAB_ROOT" describe --tags --exact-match
uv pip install --python "$ISAACLAB_ROOT/.venv/bin/python" --no-deps -e "$REBAR_REPO_ROOT/rebar_training"

rebar_python -m rebar_training.preflight --build --runtime \
  --report rebar_training/artifacts/runtime_preflight.json

# 单环境冻结动作检查；失败则先处理导入、初始夹持或接触问题
rebar_python -m rebar_training.gpu_check --task grasp --num_envs 1 --steps 400 \
  --headless --report rebar_training/artifacts/grasp_gpu_smoke.json
rebar_python -m rebar_training.gpu_check --task insert --num_envs 1 --steps 400 \
  --headless --report rebar_training/artifacts/insert_gpu_smoke.json

# 小批量 PPO 冒烟，不代表策略已学会任务
rebar_python -m rebar_training.launcher train --task grasp --num_envs 4 \
  --max_iterations 5 --headless --run_name grasp_smoke
rebar_python -m rebar_training.launcher train --task insert --num_envs 4 \
  --max_iterations 5 --headless --run_name insert_smoke

# 从零开始的 PPO 基线；实际课程使用后文的演示、DAgger 和学习先验回放
rebar_python -m rebar_training.launcher train --task grasp --num_envs 32 \
  --max_iterations 1500 --seed 42 --headless --run_name grasp
rebar_python -m rebar_training.launcher train --task insert --num_envs 32 \
  --max_iterations 1500 --seed 42 --headless --run_name insert
```

检查点与参数保存到仓库根目录下 `logs/rsl_rl/aubo_rebar/`。
固定标签的上游训练入口可能在 Kit 快速关闭时返回 0；应同时确认终轮日志、检查点迭代号和权重有限性。
可通过 `launcher play --task grasp --checkpoint /实际路径/model_....pt` 回放。
评估使用不同于训练的 seed，读取已完成回合的最终状态，避免自动复位后误读成成功：

```bash
rebar_python -m rebar_training.gpu_check --task grasp --checkpoint /实际路径/model_1499.pt \
  --num_envs 4 --episodes 100 --seed 10042 --headless \
  --report rebar_training/artifacts/grasp_eval.json
rebar_python -m rebar_training.gpu_check --task insert --checkpoint /实际路径/model_1499.pt \
  --num_envs 4 --episodes 100 --seed 10042 --headless \
  --report rebar_training/artifacts/insert_eval.json
```

报告包含每轮成功、掉落、超力、超时、耗时步数与最终位置误差
（抓取为尚欠的抬升高度，插入为钢筋中心到目标的距离）。
无检查点的冻结动作 smoke 不报告策略成功率；400 步覆盖一次 12 s 超时和自动复位。
同时记录双指持有步数比例、底盘漂移及底盘/夹具克隆原点误差；插入冻结检查要求每个环境至少 90% 的步数满足双指持有。
检查由父进程核对带唯一运行编号的完成报告，避免 Kit 快速退出时把异常误报为退出成功。
独立随机种子只改变已定义的初始分布，不代表未见摩擦/材质/设备误差的泛化评估。

## 课程与独立验收

`expert.py` 只输出与策略相同的七维动作，通过实测 TCP/Jacobian、关节与接触反馈进行物理可行性检查。
它不会写入钢筋位姿或创建连接。完整任务采用避开相机支架与前臂碰撞的关节搬运分支，
然后做 Cartesian 对齐、四垫接管、松爪及向上退出。

训练分三步：`imitation` 从成功演示做行为克隆，`gpu_check --dagger` 在学生实际访问的状态上采集专家标签，
然后 `launcher train` 用 PPO 继续更新。离线拟合误差和 DAgger 混合控制的成功率都不作为策略完成任务的证据。
单独使用 BC 曾出现闭环状态偏移；DAgger 用于训练偏移后的恢复行为。

PPO 的可选 `agent.algorithm.replay_file` 启用 `AnchoredPPO`：冻结课程训练后的神经网络作为学习先验，
在初始数据和在线访问状态上回放该网络的动作标签，限制强化学习更新对接触技能的破坏。
回放数据只用于训练；推理直接运行当前 actor，不混入专家动作。
检查点保存学习先验和回放优化器状态，可恢复训练。

以下命令从完整演示开始执行连续任务课程，自动进行 BC、DAgger、PPO 和独立验收。
`--template` 需要同一网络结构的 RSL-RL 检查点，`--agent-config` 是其 `params/agent.yaml`。

```bash
rebar_python -m rebar_training.gpu_check --task loading --expert \
  --episodes 128 --num_envs 32 --seed 42 --trace-stride 0 --headless \
  --demonstrations rebar_training/artifacts/loading_demos.pt \
  --report rebar_training/artifacts/loading_expert.json

rebar_python -m rebar_training.campaign --task loading \
  --root rebar_training/artifacts/loading_campaign \
  --demonstrations rebar_training/artifacts/loading_demos.pt \
  --template /实际路径/model_4.pt --agent-config /实际路径/params/agent.yaml \
  --num-envs 32 --dagger-steps 6000 --iterations 1500 --eval-episodes 100
```

DAgger 的最后四分之一完全使用学生动作，继续采集专家标签进行更新；
每轮 DAgger 后先独立评估 32 回合，通过后才作为 PPO 的冻结学习先验。
默认验收要求：在 seed=10042 和 20042 下分别独立运行 100 回合，每组成功率至少 95%。
完整任务还要求每个成功回合实际完成抬升；失败、超时、缺少回合、非有限状态或专家/混合控制报告都不能通过。
`campaign_report.json` 记录最终检查点 SHA-256、两组评估路径及是否通过。
未通过时从当前策略再进行一轮 DAgger/PPO，默认最多三轮；达到上限会明确报错，不会将未收敛标成成功。
这项验收范围是已定义的工位布局、直径和初始扰动；相机识别、其他钢筋尺寸及实机迁移需要另行验收。

## 暂停状态与迁移（2026-10-19）

本地 `artifacts/curriculum_20261019/` 保存本轮原始报告、权重、数据与日志，
`native_logs/` 是 133 上 RSL-RL 运行目录的完整副本，`environment.requirements.txt` 记录远端软件版本。
本地离线检查 **66 项通过**；新子阶段接口已开始 GPU 专家验证，暂停时日志累计 64 个成功回合，
采集未完成、没有可用于新接口的完整演示文件，不能宣称新连续策略已收敛。

| 项目 | 当前证据 | 迁移后建议 |
|---|---|---|
| 抓取最终 `checkpoints/grasp/model_1499.pt` | 两个独立种子分别 100/100；384000 个 PPO transitions | 可回放、独立复测或续训 |
| 交接最终 `checkpoints/insert/model_1499.pt` | 两个独立种子分别 100/100；384000 个 PPO transitions | 可回放、独立复测或续训 |
| 旧完整流程 `loading_demos.pt` | 专家 128/128，约 31.7–32.4 s，最大中心误差 1.69 mm | 仅历史物理证据；观测为旧接口 |
| 旧完整流程 `loading_campaign/round_0/dagger.pt` | 混合控制 184/188，独立策略 0/32 | 失败样本；不部署、不接新接口 |
| 新完整流程 `loading_route_v1` | 已增加阶段/目标观测并通过 66 项离线检查 | 从新专家演示开始重新运行课程 |

迁移包是最小训练项目，含本模块、训练资产、源 USD/纹理、上述权重、数据、日志、版本及校验清单。
**不包含 Isaac Sim 二进制、CUDA/驱动或旧 `.venv`**；Mac 上保存备份，运行训练需要另一台 Linux/NVIDIA 机器。
解压至任意文件夹后，在新机器安装本页指定的 Sim/Lab/Python 版本，设置前三个环境变量，
再安装本模块；不要将 requirements 中旧机器的绝对 editable 路径直接用于安装。

```bash
export REBAR_REPO_ROOT="$(pwd)"
export ISAACLAB_ROOT="$HOME/Develop/IsaacLab-rebar"
export ISAAC_SIM_ROOT="$HOME/isaacsim"
uv pip install --python "$ISAACLAB_ROOT/.venv/bin/python" --no-deps -e "$REBAR_REPO_ROOT/rebar_training"
rebar_python() { bash "$REBAR_REPO_ROOT/rebar_training/scripts/run_isaac_python.sh" "$@"; }

rebar_python -m rebar_training.preflight --runtime \
  --report rebar_training/artifacts/migration_preflight.json
rebar_python -m rebar_training.gpu_check --task grasp --num_envs 32 --episodes 100 \
  --checkpoint rebar_training/artifacts/curriculum_20261019/checkpoints/grasp/model_1499.pt \
  --seed 20042 --headless --report rebar_training/artifacts/migration_grasp_eval.json
```

交接复测将 `--task` 与检查点子目录都改为 `insert`。
恢复完整任务时，使用上文课程命令重新采集 `loading_route_v1` 演示，新的输出目录避免覆盖旧失败记录。
最终权重包含 actor、critic、PPO 优化器、冻结学习先验和回放优化器，满足恢复训练的数据条件。
所有达标结论限当前仿真布局、固定底盘和状态观测；尚未验证 Cosmos 视觉、真实机器人或新尺寸钢筋。

## 2026-10-19 初次 GPU 调通记录（课程训练前）

报告、日志和检查点保存在 `artifacts/4090_20261019/`，
[验证汇总](artifacts/4090_20261019/validation_summary.json) 记录运行版本、源代码 SHA-256 和权重校验。

| 检查 | 抓取 | 插入 |
|---|---|---|
| 单环境，冻结动作 400 步 | 通过；超时后自动复位 | 通过；超时后自动复位 |
| 4 环境，每环境 400 步 | 通过，无掉落/超力 | 通过，无掉落/超力；双指持有比例均为 100% |
| 克隆底盘/夹具原点误差、底盘漂移 | 0 m | 0 m |
| 修正奖励后的 PPO，4 环境 × 5 次更新 | 640 transitions；保存 `model_4.pt` | 640 transitions；保存 `model_4.pt` |
| 检查点权重及推理数值 | 有限 | 有限 |
| 独立 seed=10042，8 个回合 | 0/8 成功 | 0/8 成功 |

本地及 Ubuntu 训练环境均有 **56 项测试通过**。冻结动作检查验证场景、初始支撑/持筋和复位，
并未演示抬升或试验机接管。短训检查验证采样、PPO 更新、保存和加载；它不验证收敛。
奖励修正前的报告带有 `before_reward` 后缀，供排查参考。

最终短训检查点在 133 的仓库根目录下：

- `logs/rsl_rl/aubo_rebar/2026-10-19_08-17-01_grasp_reward_smoke/model_4.pt`
- `logs/rsl_rl/aubo_rebar/2026-10-19_08-17-15_insert_reward_smoke/model_4.pt`

本地副本位于 `artifacts/4090_20261019/checkpoints/{grasp,insert}_reward/`。
这是课程训练前的历史记录；后续物理验证及学习结果以本页开头和课程报告为准。
`preflight` 的 `gpu_validated=false` 表示预检查本身不验证物理；GPU 实测以本节及运行报告为准。

## 物理与学习验收重点

1. 首先用一个环境检查 articulation、关节顺序、钢筋碰撞、接触过滤矩阵；观察未连接钢筋时的初始物理抓持是否稳定。
2. 检查多个克隆的底盘固定点、机架和接触是否位于各自的环境，不能串环境碰撞或共享固定点。
3. 确认双指能够抬升源资产的钢筋；校准接触步长、驱动增益、摩擦、求解器与过力阈值。
4. 确认四垫夹紧后，机械臂松爪和撤离，钢筋仍靠实际接触保持，未启用辅助连接。
5. 独立技能与连续任务分别长训，使用两个未参与训练的随机种子验收。移动流程和视觉输入另行验证。

`tests/test_adapter.py` 的 simulator doubles 只验证适配器的张量、调用与复位行为；
`tests/test_assets.py` 验证 OpenUSD 组合；它们都不提供接触物理或学习收敛的证据。
