# 更换训练电脑：暂停、迁移与接力

## 最新恢复：已切换到 RTX 5090（2026-10-21）

**23:17 上海时间连接已恢复，原课程继续运行，本次没有手动启动、停止或重启仿真。当前为到站上料夹紧独立评估，必须先读取原控制器和新鲜报告。最近 45 分钟的 systemd suspend/hibernate 单元日志无记录（returncode 0、stderr 空）；不据此猜测先前断联原因，未改电源设置或 SSH 身份。历史网络记录保留在 review 目录。**





用户已明确要求“切换到 5090 的电脑上继续”，本节覆盖下方历史暂停说明。仅恢复仿真；4090 和 0.103 的旧目录均保持停止。

- 节点：`arnold@192.168.0.103`，RTX 5090 Laptop GPU（24 GB）；新目录 `/home/arnold/Develop/rebar-training-5090-resume-20261021`。
- 入口：`artifacts/mobile_curriculum_20261020/review_20261020/status.json`；**当前课程 `course_v9_5090_resume`，控制器 PID/PGID 112563**。先看当前课程 `curriculum.status.json`、`active_jobs` 和原生 job/status/progress/log。迁移验证控制器 108930 已以 `course_started` 完成退出，不要重启旧 PID/验证作业。
- 217 个运行文件逐一 SHA 核对；完整历史归档 5,350,801,884 字节在 Mac 和 5090 两端 SHA 一致。历史归档留在 `/home/arnold/Develop/rebar_training_migration_platform_20261021.tar.gz`，不得解压覆盖正在运行的新目录。
- CUDA 已实测加载冻结 `fe48b74d` 策略与 optimizer，计数 278 updates / 100 actor / 21833 steps，网络与 optimizer 有限；10 个 USD 运行层依赖闭合，仅 SDK 的 OmniPBR 材质由仿真运行时提供。源码仍冻结 `2c7ef709`，检测器和资产不变。
- 上海时间 **22:32 核查**：迁移物理回归 **3/3 全部成功、0 失败**。连续复位两回合误差 1.548/1.613 mm；30 帧部分抓取失视回合误差 1.502 mm，实际触发一次抓取中止，实测打开无载、托槽支撑且已返回后重新观察，半速恢复并完成四爪夹紧、卸载和退让。全部原生报告、源码/策略/检测器/资产及严格物理凭据独立复核通过，766 个完整原始文件两端 SHA 核验。回归不计最终独立分母。
- 节点 CPU 全套 **239 通过**；`tray_train` 已原生完整 **2/2 成功**，放槽误差 0.632/0.008 mm，实际支撑、打开卸载及约 179.7 mm 退让复核通过。新增 **25 次 PPO、12 次 actor、2790 个有效策略步**，最终 303 updates / 112 actor / 24623 steps；actor 最大权重变化 0.009190，模型与 optimizer 有限。输出策略 SHA 为 `d1e26e7bb23d43d6d111af24180d10e284120055fe9d0104f512f95a06b3d851`，130 个完整训练原始文件已两端 SHA 核验。训练完成不代表独立技能 gate 通过。
- 新版抓取放槽独立评估 `6131042/6141042` 已各 **4/4、合计 8/8 通过**。本机与节点独立复算 gate 完全一致：Wilson95% 下界 0.675592435 达到技能 0.60 门槛、零证据错误。放槽误差范围 0.243–1.007 mm，真实支撑/打开卸载/退让及冻结 d1e26/source2c/检测器/资产复核通过，456 个完整原始文件两端 SHA 核验。技能 gate 不计入最终 64 回合整链分母。
- 到站上料夹紧训练 `dock_clamp_train` 已原生完整 **2/2 成功**，夹紧误差 **1.619/1.856 mm**，真实四爪、松爪卸载和 96.7/88.7 mm 退让及稳定速度复核通过。新增 **26 PPO / 10 actor / 2089 有效步**，累计 329 updates / 122 actor / 26712 steps；actor 权重最大变化 0.007041，模型与 optimizer 有限，最终策略 SHA 为 `f70f6115c984357a305f10b13eb3a1a6536740e0ea1e98545d093b517a5cd831`；156 个完整训练文件已两端 SHA 核验。
- 当前原控制器 112563 已自动推进到站独立评估 `dock_clamp_holdout_6151042/6161042`，runner 119251/119252（以实时 `active_jobs` 为准），各 4 回合、同一 f70f/source2c/检测器/资产，最多两仿真并行。23:17 首回合尚未结束，视觉有效、日志新鲜；**到站 gate 和最终整链 gate 尚未通过**。
- 迁移回归全部通过后已自动创建 `course_v9_5090_resume`，按 `config/mobile_curriculum_recovery_v9_5090.json` 重跑训练及技能 gate。新训练种子 6031042/6041043/6051043；技能评估 6131042–6161042 各 4 回合；整链评估 6241042–6311042（间隔 10000）各 8，共 64。预声明种子与 115 个已存在作业/计划种子、检查点训练上下文均无重叠。
- 汇报口径：**4090／v8 历史完整组报告为 16/64，另两组各 1 个成功回合后用户中断，总计 18 成功、0 失败；5090／v9 另行统计。** 23:17 确认 **v9 最终整链独立 0/64，成功 0、失败 0、本轮新增 0**；抓取放槽 8/8 是技能 gate，另行统计。 迁移前成绩与新平台验收不合并。 每阶段 gate 必须实际通过，同一最终冻结策略/源码/检测器/资产，成功率 ≥95%、Wilson95% 下界 ≥90%、零证据错误，才宣布稳定。最多两仿真并行。
- 自动跟进已恢复，每 15 分钟汇报上海时间、累计回合/64、成功失败、较上轮新增、当前阶段与下一步；节点不可达时明确注明实时状态未知。详见 `reports/resume_5090_20261021.json`。失败则保存全部证据分析修复，用新目录和未用种子验证；不修改旧证据或放宽判据。

## 当前授权与成果

用户于 2026-10-21 明确要求暂停训练和独立验收，准备更换电脑。
3.133 上的课程父进程及两组评估均已停止，录制前确认本任务无残留进程、GPU 空闲；定时任务 automation-2 已暂停。
另行授权的当前策略演示已 1/1 成功，约 5 分 4 秒，夹紧误差 1.435 mm；完整帧解码、物理凭据及阶段关键画面已核查。录像不学习、不恢复课程、不计独立验收分母，录制后全部本任务仿真进程已退出，GPU 空闲。
录像为 `review_20261020/migration_pause_20261021/current_policy_demo_labeled.mp4`（相对于训练 campaign 目录），原生录屏和原始单回合报告一并保留。
未收到明确恢复指令前，新旧设备都只能校验和准备文件。

- 本机工程：`/Users/weiluo/Develop/github/AuboAMR300_ROS2`。
- 原训练节点：`arnold@192.168.3.133`，`/home/arnold/Develop/rebar-training-4090`。
- 当前课程：`rebar_training/artifacts/mobile_curriculum_20261020/course_v8_precision_envelope`。
- 控制源码 SHA256：`2c7ef709bc6ad5d6ed5128c9d4e7399179938cf230129cd0e1c33221f7010aa2`。
- 当前策略及 optimizer：课程下 `full_policy.pt`，SHA256 `fe48b74da455c7562ff079d9990cd0a8b534d064d9217029a48431cea5bc235b`。
- 抓取和到站技能独立门槛各 8/8；完整整链训练 2/2，新增 86 次 PPO 更新、24 次 actor 更新、4987 个有效策略步。
- 整链独立验收：5241042/5251042 两组各 8/8 完整成功；5261042/5271042 各完成 1 个成功回合，随后正在运行的回合被用户中断。累计 18 个完整回合成功、0 失败，后两组不是完整 8 回合报告。
- 最终 64 回合门槛未通过；中断不算完整成功，也不是任务物理失败。保留原始日志、进度和停止凭据，不补写完整报告。

当前控制为相机视觉伺服、阶段/夹爪管理及里程计导航叠加有界关节速度残差 PPO。输入 76 维视觉/机器人反馈，输出 6 维残差；不是独立整链神经策略，尚未调用 Cosmos 推理。

## 迁移文件

准备的独立迁移包为 `rebar_training/artifacts/rebar_training_migration_platform_20261021.tar.gz`，同目录有 `.sha256`、`.verify.json`。
只有校验报告存在且 `mismatches` 为空时，才表示包已完成。包内状态是生成归档前的快照；归档本身最终 SHA 以包外同名 `.sha256` / `.verify.json` 为准，包外 `.recovery_status.json` 保存最终恢复记录。包内 `TRANSFER_MANIFEST.json` 逐文件记录 SHA256。
它应包含当前 v8 的权重和 optimizer、完整及中断的原生报告/日志/相机数据、冻结源码、预声明计划、前置物理回归、演示视频与凭据、生成 USD 及原 CAD/纹理依赖、检测器、配置、测试和本指导。

虚拟环境、Isaac SDK、SSH 身份、Git 凭据及此前的备份压缩包不随包迁移。旧归档保留在本机，但这个新包不依赖逐层解开旧备份。
已修正 `scripts/build_transfer.py`：`mobile-resume` 读取 `review/status.json` 的 `current_course` 和 `migration_required_directories`，不会再漏掉 v8，也拒绝越出当前产物目录的路径。

关键恢复入口：

| 相对 `rebar_training/` 的位置 | 用途 |
|---|---|
| `artifacts/mobile_curriculum_20261020/review_20261020/status.json` | 最新暂停、备份与录像状态 |
| `artifacts/mobile_curriculum_20261020/PAUSED_20261021_PLATFORM_CHANGE.json` | 停止的进程组、原进度及停止后核查 |
| `reports/platform_pause_20261021.json` | 可提交 GitHub 的暂停摘要 |
| `artifacts/mobile_curriculum_20261020/course_v8_precision_envelope/` | 当前策略、课程计划及所有原始证据 |
| `artifacts/mobile_curriculum_20261020/review_20261020/migration_pause_20261021/` | 本次录像、单回合凭据、运行时版本和视频核查 |
| `artifacts/mobile_curriculum_20261020/review_20261020/precision_resume_4090_20261021/` | 已完成的 9 回合开发物理回归 |

## 新电脑上的准备步骤（不会启动训练）

1. 将迁移包及两个校验文件复制到新电脑，先执行 `sha256sum -c rebar_training_migration_platform_20261021.tar.gz.sha256`。
2. 解压到新的专用工程目录，不能覆盖原 ROS 工作区；例如 `~/Develop/rebar-training-new-platform`。包中的路径均相对工程根目录。
解压示例（从存放迁移包的目录执行）：

```bash
mkdir -p "$HOME/Develop/rebar-training-new-platform"
tar -xzf rebar_training_migration_platform_20261021.tar.gz -C "$HOME/Develop/rebar-training-new-platform"
cd "$HOME/Develop/rebar-training-new-platform"
```

3. 在新目录核验逐文件 SHA256：

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib, json
root = Path.cwd()
manifest = json.loads((root / 'TRANSFER_MANIFEST.json').read_text())['files']
bad = []
for name, expected in manifest.items():
    path = (root / name).resolve()
    path.relative_to(root.resolve())
    if not path.is_file():
        bad.append(name)
        continue
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    if digest.hexdigest() != expected:
        bad.append(name)
print({'checked_files': len(manifest), 'mismatches': bad})
assert not bad
PY
```

4. 对照录像目录的 `runtime_versions.json` 和原预检查，准备新机的 Python 3.12、CUDA、Isaac Sim/Lab。原运行节点为 x86_64 / RTX4090，Torch 2.11.0+cu130、Sim 6.0.1；Lab Git 检出与安装包元数据须分别核对，不能只根据版本字符串判定环境相同。DGX Spark 是另一架构，不能复制 x86_64 虚拟环境；SDK/Torch 兼容性必须在新机实测。
5. 使用新机实际路径配置，不复用旧作业 JSON 中的绝对路径：

```bash
export REBAR_REPO_ROOT="$PWD"
export ISAACLAB_ROOT="$HOME/Develop/IsaacLab-rebar"
export ISAAC_SIM_ROOT="$HOME/isaacsim"
export REBAR_TRAINING_PYTHON="$ISAACLAB_ROOT/.venv/bin/python"
```

6. 校验 GPU/CUDA、检测器、全部 USD 依赖以及冻结源码和策略 SHA。不要用 `--build` 重新生成或覆盖当前验收资产。不要用 `uv sync` 随意替换 SDK 配套 Torch。
7. 原目录、原 PID 和课程 `--worker` 都不能直接重启。原控制器不支持在原输出目录安全接续，会重跑旧名称、覆盖路径或碰到已有文件。新平台必须用新输出目录、重新生成本机启动参数并预声明验证范围。

## 收到恢复授权后优先做什么

先核对新机 SDK/CUDA、相机输出、接触载荷和 checkpoint/source/detector/assets 指纹，再用当前 fe48 权重做明确标注的物理回归。
不要把旧电脑的 18 个回合与新平台回合直接拼成已通过的稳定性结论；在新平台预声明未用的独立种子并完成新的完整验收。
中断的 5261042/5271042 输出永久保留；不在原名称原目录重跑，不把录像或开发回归加入独立分母。
若策略、源码、检测器或资产发生改变，各阶段独立 gate 都应重新验证；不得通过放宽判据解决失败。

保持 RGB/机器人反馈控制及钢筋真值运行时隔离、0.8 mm 视觉对准门控、真实接触物理、3 mm/3°、250 N、四爪夹紧、松爪卸载和退让。空腕重观察只在打开无载、托槽有支撑且已返回时触发，最多两次。
最终验收仍为同一冻结策略/源码/检测器/资产下至少 64 个预声明连续回合、至少两个独立种子、成功率至少 95%、Wilson95% 下界至少 90%、零证据错误。
仅限仿真；不启动真实机器人/ROS 硬件，不经过飞书，不修改原 ROS 模型、Isaac SDK 或无关 GUI/地图。

## 可直接给下一位 agent 的接力提示词

> 用户已暂停训练和独立验收，准备更换电脑。先读 `rebar_training/README.md`、`TRAINING_HANDOFF.md`、`MIGRATE_PLATFORM_20261021.md`、`reports/platform_pause_20261021.json` 和 `artifacts/mobile_curriculum_20261020/review_20261020/status.json`。旧课程及评估已停止，automation-2 已暂停；未获明确恢复指令前只校验和准备文件。当前策略 fe48b74d 包含 optimizer，冻结源码 2c7ef709，抓取和到站技能各 8/8、整链训练 2/2，独立整链仅 18 个完整回合成功；后两组各 1 回合后被用户中断，最终 64 回合验收未完成。独立迁移包及逐文件 SHA 见 artifacts，勿依赖旧电脑的虚拟环境或绝对路径。收到恢复授权后先核查新平台环境、冻结指纹和真实相机/接触物理回归，再用新目录、新预声明种子推进完整验收，不拼接跨平台回合、不覆盖中断证据。演示视频单独标为当前效果，不是稳定性验收通过证明。保持全部物理判据、真值隔离及仿真边界，勿启动真实机器人或改无关文件。
