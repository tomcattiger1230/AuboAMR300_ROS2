# 更换训练电脑：暂停、迁移与接力

## 当前授权与成果

用户于 2026-10-21 明确要求暂停训练和独立验收，准备更换电脑。
3.133 上的课程父进程及两组评估均已停止，录制前确认本任务无残留进程、GPU 空闲；定时任务 automation-2 已暂停。
仅另行授权当前策略的一回合演示录像，不学习、不恢复课程、不计独立验收分母。录完后也停止仿真。
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
只有校验报告存在且 `mismatches` 为空时，才表示包已完成。包内 `TRANSFER_MANIFEST.json` 逐文件记录 SHA256。
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

1. 将迁移包及两个校验文件复制到新电脑，先执行 `sha256sum -c 包名.tar.gz.sha256`。
2. 解压到新的专用工程目录，不能覆盖原 ROS 工作区；例如 `~/Develop/rebar-training-new-platform`。包中的路径均相对工程根目录。
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
