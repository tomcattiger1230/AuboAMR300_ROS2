# 迁回 4090：暂停与恢复交接

**2026-10-21 已按用户要求暂停。不要启动训练、物理回归或自动跟进；等待明确的继续指令。**

2026-10-21 06:56（上海时间）已停止旧课程父进程、两个评估子进程及新版排队控制器；停止后确认本任务无残留进程、GPU 计算进程为空。自动跟进 `automation-2` 已设为 `PAUSED`。

## 当前成果与失败边界

- v7 抓取放槽独立评估 8/8 通过；到站训练 2/2 成功，夹紧误差 1.297/1.447 mm，实际新增 10 次 actor 更新。
- v7 两组到站独立评估各完成首回合且失败；第 2 回合被用户暂停。两组均不是完整 4 回合报告，不能写成 0/8 完整验收，更不能晋级。
- 失败时 RGB 有效，视觉误差约 0.98–1.01 mm，高于原有 0.8 mm 对准门控；实际钢筋误差约 1.90 mm，但四爪载荷为零，未完成夹紧、松爪和退让。
- v8 新增仅用于最后插入对准阶段的二次残差收缩，使用原相机/机器人反馈输入。保持 0.8 mm 门控、3 mm/3°、250 N、真实四爪夹紧及卸载退让判据。
- 新修复通过本机 231 项（3 跳过）、原节点 237 项 CPU 测试；另有 2 项迁移打包测试通过。**新修复的物理回归仍为 0/9，尚未证明修复有效。**
- 完整 64 回合独立整链稳定性验收及对应完整流程视频尚未完成。

## 文件与迁移包

本机工程：`/Users/weiluo/Develop/github/AuboAMR300_ROS2`。
旧 Ubuntu 工作副本：`/home/arnold/Develop/rebar-training-0103`。

| 文件/目录（相对工程根目录） | 用途 |
|---|---|
| `rebar_training/artifacts/mobile_curriculum_20261020/PAUSED_20261021_MIGRATE4090.json` | 停止时间、进程组及停止后核查 |
| `rebar_training/artifacts/mobile_curriculum_20261020/review_20261020/status.json` | 最新全局恢复入口，暂停状态优先 |
| `rebar_training/artifacts/mobile_curriculum_20261020/review_20261020/precision_residual_fix/` | 新版冻结源码、9 回合验证调度、失败分析、原始策略、测试及种子预声明 |
| `rebar_training/artifacts/mobile_curriculum_20261020/course_v7_support_lifecycle/` | 旧版完整训练记录、独立评估失败/中断记录、策略与 optimizer |
| `rebar_training/config/mobile_curriculum_recovery_v8.json` | 新版课程及 64 回合预声明种子 |
| `rebar_training/scripts/prepare_resume.py` | 只准备新目录，不执行 SSH、仿真或训练 |

独立迁移包：`rebar_training/artifacts/rebar_training_migration_4090_20261021.tar.gz`。
同目录的 `.sha256` 与 `.verify.json` 用于校验；包内 `TRANSFER_MANIFEST.json` 记录每个文件的 SHA256。
它包含当前代码、生成的移动/完整测试机资产及源 CAD/纹理依赖、灰度检测器、诊断图像、当前课程原始日志/报告/源码快照、权重及 optimizer、冻结 v8 输入与暂停记录。

Python 虚拟环境、Isaac SDK、SSH 身份和此前多轮备份大包不打入迁移包。早期历史完整备份继续保存在本机 `rebar_training/artifacts/`，不会删除；迁移包覆盖当前移动任务恢复所需文件。
GitHub 保存代码、配置、测试、暂停摘要和指导文件；大体积资产、模型与原始日志随本机迁移包保存。

## 更换电脑后先完成准备

以下只准备文件；环境核查也应在用户允许开始验证后执行。尚未连接或验证新的 4090 主机，旧 IP、账号、目录均不能直接假定适用。

1. 把迁移包及其 `.sha256` 拷贝到新 Ubuntu。在同目录运行 `sha256sum -c rebar_training_migration_4090_20261021.tar.gz.sha256`。
2. 解压到专用的新目录，例如 `$HOME/Develop/rebar-training-4090`，不要覆盖原 ROS 工作空间。包内保留工程相对目录，不包含原 SDK 或虚拟环境。
3. 使用 Python 3.12，核对 Isaac Sim **6.0.1**、Isaac Lab **v3.0.0-beta2.patch1**。原节点 Torch 是 2.11/cu130；4090 驱动、Torch CUDA 与现有 SDK 的兼容性须在新机实际核查，不能凭上一台电脑的检查结果通过。
4. 在新工程根目录配置路径，按新机器实际位置修改：

```bash
export REBAR_REPO_ROOT="$PWD"
export ISAACLAB_ROOT="$HOME/Develop/IsaacLab-rebar"
export ISAAC_SIM_ROOT="$HOME/isaacsim"
export REBAR_TRAINING_PYTHON="$HOME/Develop/rebar-training-runtime/bin/python"
export REBAR_TRAINING_ASSETS="$PWD/rebar_training/artifacts/assets_mobile_v2"
```

恢复后的 agent 应先核对 SHA、CUDA/版本、USD 资产依赖及 CPU 测试，再进行物理验证。`run_isaac_python.sh` 使用上述路径，不依赖旧用户名。

## 收到明确继续指令后

新控制源码 SHA：`2c7ef709bc6ad5d6ed5128c9d4e7399179938cf230129cd0e1c33221f7010aa2`。
诊断失败策略 SHA：`b86ea96e72364169acf6ba0562391347a8d1ae425fdc7ca5546989e01e6eb4aa`。
正式 warm start SHA：`d287596859c4f9627bdbe4d9765283fb40fb66d2d58f494663765adc63d11554`。

先准备一个全新验证目录，不能直接重启旧 `validate_precision.py`：旧目录已有 `cpu_tests.log`、PID、旧主机绝对路径以及暂停状态。准备脚本会校验源码、计划和策略哈希，并只复制不可变输入。

```bash
python3 rebar_training/scripts/prepare_resume.py \
  --template rebar_training/artifacts/mobile_curriculum_20261020/review_20261020/precision_residual_fix \
  --output rebar_training/artifacts/mobile_curriculum_20261020/review_20261020/precision_resume_4090_20261021
```

此命令只写文件，不恢复训练。agent 得到继续授权后，才可在新目录使用冻结 `validation_source`、新机器的 `REBAR_REPO_ROOT` 和隔离运行时，启动 `validate_precision.py`；并把实际新目录写入全局状态。新输出目录已存在时必须换新名字，不能清除旧证据。

调度顺序为：相同失败策略/种子 4151042、4161042 各 1 回合诊断回归 → 可信 d287 权重的整链训练回归 2 回合/至少 60 次新更新 → 闭爪失视 1 回合 → 复位 4 回合，至少 9 回合全部成功。它们都不计入最终独立验收。

之后重新训练所有课程阶段并通过新的独立门槛，不能复用 v7 的 8/8 结果。v8 训练种子 5031042/5041043/5051043；抓取评估 5131042/5141042、到站评估 5151042/5161042 各 4 回合；完整连续流程使用 5241042 至 5311042、间隔 10000，各 8 回合，共 64。它们已经预声明，均尚未执行。

独立仿真并行最多两个。最终必须同一冻结策略/源码/检测器/资产，成功率至少 95%、Wilson95% 下界至少 90%、零证据错误，保留所有失败回合。达标后再录制、解码核验实际完整流程视频并校验备份；此前不得宣称稳定完成。

## 可以直接交给下一位 agent

> 当前测试已由用户明确暂停，准备迁回 4090。先读 `rebar_training/README.md`、`TRAINING_HANDOFF.md`、`MIGRATE_4090.md` 和 `artifacts/mobile_curriculum_20261020/review_20261020/status.json`。所有旧/新父子仿真进程已停止，automation-2 已暂停；未得到用户明确继续指令前，只能核对和准备文件，不能启动仿真或恢复自动跟进。新修复源码为 2c7ef709，CPU 测试通过但物理验证 0/9；先用 prepare_resume.py 建立全新验证目录并核对新机 SDK/CUDA，再按授权进行同一失败策略/种子物理回归。旧 v7 独立到站评估两组各首回合失败、第二回合中断，不能当作合格策略或完整 8 回合报告。新版本所有阶段 gate 及 64 回合整链验收必须重跑，最终再交付实际完整流程视频。不得放宽 0.8 mm 控制门控、3 mm/3°评分、250 N、四爪夹紧、松爪卸载及退让要求；只限仿真，不启动真实机器人或修改原模型/Isaac Lab/无关 GUI。
