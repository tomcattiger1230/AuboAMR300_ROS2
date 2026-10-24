# 3.144 Hermes / OpenClaw 只读调试部署

训练继续暂停。2026-10-19 已经用户授权，3.144 与 3.133 的受限只读通道已部署并完成验收：
飞书 H-Coder → Hermes 原生工具 → 固定桥接 → 专用 SSH 密钥 → 3.133 固定接收器。
四个操作 `status`、`list_artifacts`、`inspect_manifest`、`tail_log` 已通过真实调用，
课程目录列出 84 个产物。越界路径、训练请求及客户端命令绕过检查均通过。
当前 `execute_readonly=true`，本轮未启动训练、仿真或机器人。
保存状态及日志仅反映历史记录，不是实时训练进程探测。
验收结果见 `artifacts/feishu_debug_20261019/session_report.json`。
GPU 原文件备份：`/home/arnold/rebar-readonly-backup-pm54lp_o`；
Spark 桥接原配置备份：`/home/arnold/.config/rebar-debug/config.json.bak-r9`。

常规控制方式为：**本机飞书 → H-Coder → 远端只读操作**。
此前不从本机使用 SSH / SCP 直接连接 3.144 或 3.133。
用户现已说明三机同网并要求本人输入密码完成首次 SSH 身份设置，
已准备本机一次性引导脚本 first_ssh_setup.py；由用户直接在终端执行和输入密码。
常规调试仍通过飞书 H-Coder，自动身份限 3.144 到 3.133 的只读接收器。
后文远端命令由 H-Coder 在对应主机执行；涉及新增 SSH 访问权限时先准备具体配置再确认。
已通过飞书确认 H-Coder 运行于 3.144，账号 `arnold`，
`HERMES_HOME=/home/arnold/.hermes/profiles/coder`。
已在 3.144 CLI 校验并启用 Hermes 插件，新会话已通过原生工具完成 3.133 真实查询及访问限制验收。
OpenClaw 适配器保留作备用，未在该独立实例重复启用。

H-Coder 模型目标为 `glm-5.3`，沿用用户已配置的国内 BigModel API 密钥。
不能仅凭 provider 显示名 `zai` 判断地区，必须核对有效 `model.base_url`。
本轮发现通用模型切换命令曾写入国际地址 `api.z.ai`；修正和运行时验证结果见
`artifacts/feishu_debug_20261019/session_report.json`。
不要再次运行会把国内地址覆盖成国际默认值的切换命令，也不通过飞书传输 API 密钥。

链路：飞书 → 3.144 上实际执行 GLM 工具调用的 Hermes 或 OpenClaw →
固定 Python 桥接 → 专用 SSH 密钥 → 3.133 固定接收脚本 → 保存的报告与日志。
工具只提供 `status`、`list_artifacts`、`tail_log`、`inspect_manifest`。

## 1. 确认机器人运行账号与框架版本

在 **3.144** 上，进入现有机器人实际使用的 Linux 账号、容器和 profile 后执行：

```bash
whoami
python3 --version
hermes --version
hermes plugins --help
openclaw --version
openclaw plugins --help
node --version
```

只需执行实际安装的框架命令。不要为接入工具升级现有机器人。
本文依据 2026-10-19 的官方插件接口；旧版本若没有对应子命令或 SDK 导出，
先停在插件安装前，根据实际版本调整适配层。该机实际版本及 CLI 插件校验结果已记录在 session_report.json。

若 OpenClaw 接收飞书后把请求交给 Hermes，且 GLM 的工具循环在 Hermes 内，
先安装 Hermes 插件；若 GLM 工具循环在 OpenClaw 内，安装 OpenClaw 插件。
两者都有独立会话时可分别安装；同一飞书入口保持现有唯一的消息接收者。
若两者之间只有文本转发，工具应注册在实际运行 GLM 工具循环的一侧。

后文 `$HOME` 均指上述**机器人运行账号**的 home，不是临时登录账号。
容器部署时，桥接文件、配置、密钥、known_hosts 均须在机器人容器内可读；
容器内需要 `/usr/bin/python3` 和 `ssh`，并能访问 3.133:22。

## 2. 传输小型部署包并安装桥接

本机新增包 `rebar_training/artifacts/rebar-debug-offline-20261019-v2.tar.gz`，
仅包含调试入口、两个插件、CPU 冒烟测试和说明，不含训练权重、环境或训练启动器。
当前 v2 修复产物目录整体符号链接及保存状态报告的路径限制；初版包不再用于部署。
**旧的 Cosmos 迁移包不包含本次新增的原生插件。**

在 **Mac 本机飞书** 的 H-Coder 私聊中，用“本地文件”发送 v2 包及其 SHA-256。
让 H-Coder 报告附件实际下载路径、校验哈希，并在独立临时目录安全解包。
先运行包内 `rebar_training/deploy/readonly_smoke.py`：它只访问合成临时数据，
桥接为 dry-run，不连接 3.133。全部通过后再处理正式部署。
以下示例假设 H-Coder 已把经过校验的包保存到机器人账号的 home；
如果附件落在 profile 的缓存目录，应使用其实际路径。

在 **3.144** 上：

```bash
mkdir -p "$HOME/.local/share/rebar-debug/source" "$HOME/.config/rebar-debug"
tar -xzf "$HOME/rebar-debug-offline-20261019-v2.tar.gz" \
  -C "$HOME/.local/share/rebar-debug/source"
install -m 0644 \
  "$HOME/.local/share/rebar-debug/source/rebar_training/scripts/spark_debug_bridge.py" \
  "$HOME/.local/share/rebar-debug/spark_debug_bridge.py"
```

通过现有管理通道把相同部署包放到 **3.133** 的 `/tmp/`，以 `arnold` 执行：

```bash
TASK_GPU_REPO=/home/arnold/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2
tar -xzf /tmp/rebar-debug-offline-20261019-v2.tar.gz -C "$TASK_GPU_REPO" \
  rebar_training/scripts/remote_debug_stdio.py \
  rebar_training/src/rebar_training/__init__.py \
  rebar_training/src/rebar_training/spec.py \
  rebar_training/src/rebar_training/remote_debug.py
printf '%s\n' '{"request_id":"gpu-local-1","operation":"status"}' | \
  /usr/bin/python3 -I "$TASK_GPU_REPO/rebar_training/scripts/remote_debug_stdio.py" --root "$TASK_GPU_REPO"
```

应返回 `ok: true`、`training_paused: true`，可能包含 `saved_status`。
接收器仅用 Python 标准库，目标 Ubuntu 系统 Python 需 3.10+，无需加载 Isaac/CUDA。
四个源文件会更新本次调试接口；有自行修改时先备份这四个文件。

## 3. 创建专用密钥，锁定远端入口

在 **3.144** 上创建一把仅用于本工具的密钥；如果同名文件已存在，保留原密钥：

```bash
install -d -m 0700 "$HOME/.ssh"
test -f "$HOME/.ssh/rebar_debug_ed25519" || \
  ssh-keygen -t ed25519 -N '' -f "$HOME/.ssh/rebar_debug_ed25519" -C 'spark-rebar-readonly'
cat "$HOME/.ssh/rebar_debug_ed25519.pub"
```

将输出的**公钥**复制到 3.133 的 `/tmp/rebar_debug_key.pub`。
在 **3.133 / arnold** 下执行，向现有 `authorized_keys` 追加限定规则，保留其他密钥：

```bash
install -d -m 0700 "$HOME/.ssh"
touch "$HOME/.ssh/authorized_keys"
chmod 0600 "$HOME/.ssh/authorized_keys"
python3 - <<'PY'
from pathlib import Path
public = Path('/tmp/rebar_debug_key.pub').read_text().strip()
if not public.startswith('ssh-ed25519 ') or '\n' in public:
    raise SystemExit('需要单行 ed25519 公钥')
repo = '/home/arnold/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2'
fixed = f'/usr/bin/timeout 20s /usr/bin/python3 -I {repo}/rebar_training/scripts/remote_debug_stdio.py --root {repo}'
entry = f'from="192.168.3.144",restrict,command="{fixed}" {public}'
path = Path.home() / '.ssh/authorized_keys'
old = path.read_text()
key = public.split()[1]
if key in old:
    if entry not in old.splitlines():
        raise SystemExit('该公钥已有其他规则；先人工确认并改为本条限定规则，不要追加重复公钥')
else:
    with path.open('a') as out:
        out.write(('\n' if old and not old.endswith('\n') else '') + entry + '\n')
PY
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

此密钥只接受源地址 3.144，禁止端口转发、交互终端等，并强制运行接收器。
即使客户端请求其他命令，SSH 仍运行固定接收器；不需要修改全局 sshd 配置或重启训练机器。
如果容器出口并非 3.144，先查明真实源地址，再调整 `from=`。

在 **3.144** 采集候选主机公钥并核对指纹：

```bash
ssh-keyscan -T 5 -t ed25519 192.168.3.133 > "$HOME/.ssh/rebar_debug_hostkey.candidate"
ssh-keygen -lf "$HOME/.ssh/rebar_debug_hostkey.candidate"
```

与刚才 3.133 控制台得到的指纹一致后，再执行：

```bash
cat "$HOME/.ssh/rebar_debug_hostkey.candidate" >> "$HOME/.ssh/known_hosts"
chmod 0600 "$HOME/.ssh/known_hosts" "$HOME/.ssh/rebar_debug_ed25519"
```

若已有该主机的不同旧密钥，先核实更换原因，不关闭主机密钥校验。
这把密钥的限制只作用于该工具通道，不改变机器人其他已有终端权限或管理密钥。

## 4. 写入配置，先演练，再开放只读连接

在 **3.144** 创建配置；如果已有配置，先备份并合并：

```bash
cat > "$HOME/.config/rebar-debug/config.json" <<'JSON'
{
  "target": "arnold@192.168.3.133",
  "receiver": "/home/arnold/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2/rebar_training/scripts/remote_debug_stdio.py",
  "identity_file": "~/.ssh/rebar_debug_ed25519",
  "execute_readonly": false
}
JSON
chmod 0600 "$HOME/.config/rebar-debug/config.json"
printf '%s\n' '{"request_id":"spark-dry-1","operation":"status"}' | \
  /usr/bin/python3 "$HOME/.local/share/rebar-debug/spark_debug_bridge.py" \
  --config "$HOME/.config/rebar-debug/config.json"
```

应返回 `dry_run: true`，没有发起 SSH。
步骤 2、3 完成后，将配置中的 `execute_readonly` 改为 `true`，再运行同一查询。
该开关只启用只读查询，不会解除训练暂停。
应返回真实接收器输出 `ok: true`，并且不再包含 `dry_run`。

桥接忽略通用 SSH 配置、禁用 SSH agent，仅使用指定密钥，严格校验 known_hosts。
连接超时 5 秒、桥接调用超时 15 秒、插件调用超时 20 秒。
错误也返回 JSON；不要把超时或离线解释为查询成功。

## 5A. Hermes 原生插件

仅在 Hermes 执行工具调用时采用此分支。在 **3.144 / 同账号同 profile**：

```bash
mkdir -p "${HERMES_HOME:-$HOME/.hermes}/plugins/rebar-debug"
cp "$HOME/.local/share/rebar-debug/source/rebar_training/deploy/hermes-rebar/"* \
  "${HERMES_HOME:-$HOME/.hermes}/plugins/rebar-debug/"
hermes plugins validate "${HERMES_HOME:-$HOME/.hermes}/plugins/rebar-debug"
hermes plugins enable rebar-debug
hermes plugins list
```

使用命名 profile 时，这些命令也必须加同一个 `hermes -p PROFILE ...`，
目录应放在该 profile 的 `HERMES_HOME/plugins`，而不是其他 profile 的目录。
插件注册 `rebar_readonly_debug`，工具集为 `rebar_debug`。
若原机器人有显式工具集过滤，将该工具集加入现有允许项，保留其他配置。

重载现有网关：由 Hermes CLI 管理时执行 `hermes gateway restart`；
若由自定义 systemd/Docker 管理，则使用原来的服务管理方式。
新会话中确认工具出现在模型可用工具列表后再验收。
不需要修改 GLM key、飞书 App Secret 或 Hermes 核心代码。

官方接口：[Hermes 插件注册和发现](https://hermes-agent.nousresearch.com/docs/developer-guide/plugins)。

## 5B. OpenClaw 原生插件

仅在 OpenClaw 执行工具调用时采用此分支。在 **3.144 / 同账号同实例**：

```bash
openclaw plugins install --link --no-enable \
  "$HOME/.local/share/rebar-debug/source/rebar_training/deploy/openclaw-rebar"
openclaw plugins enable rebar-debug
openclaw plugins inspect rebar-debug --runtime --json
```

这是本地源码插件，使用 ESM 与 `openclaw/plugin-sdk/plugin-entry`，
无额外 npm 运行依赖；不是发布到 ClawHub 的包。
如果提示找不到该 SDK 导出，说明当前版本接口不同，先禁用插件并按该版本适配；
无需为了此工具升级整套机器人。

该工具声明为 optional，须加入**当前飞书 agent 生效的**工具允许配置。
例如当前使用 profile 时，在现有配置相应层的 `tools.alsoAllow` 数组追加：

```json
{
  "tools": {
    "alsoAllow": ["rebar_readonly_debug"]
  }
}
```

只合并这一项，不用上述片段覆盖整个配置或已有数组；
若已有显式 `tools.allow`，则在其生效层追加该工具，并检查 deny、provider、agent、
sandbox 等更严格规则没有屏蔽它。允许项必须是工具名，不是描述文本。
保留原飞书用户/会话白名单。

```bash
openclaw plugins reload rebar-debug --wait
openclaw plugins inspect rebar-debug --runtime --json
```

旧版本没有 reload 时，通过原有服务管理方式重启现有 Gateway。
确认运行时注册表包含 `rebar_readonly_debug`；新开飞书会话再验证 GLM 工具调用。

官方接口：[工具注册](https://docs.openclaw.ai/plugins/building-plugins)、
[本地安装与重载](https://docs.openclaw.ai/cli/plugins)、
[工具允许规则](https://docs.openclaw.ai/gateway/config-tools/tool-policy)。

## 6. 验收飞书完整链路

保留现有飞书发送者/会话鉴权，不要为此新建另一个监听器。
GLM 后端必须提供 function/tool calling；仅能回答文字尚不代表工具执行成功。
向机器人系统说明追加以下规则，并在允许的飞书会话手工发起查询：

> 钢筋项目继续暂停训练。读取训练报告或日志时使用 rebar_readonly_debug。
> status 只代表保存的验收结果；training_paused 是暂停策略标记，不是实时进程探测。
> 只汇报工具实际返回的数据；dry_run、离线或错误必须明确说明。
> 不使用其他终端工具启动训练、仿真、机器人动作或修改远端代码。
> request_id 使用飞书消息 ID 或本轮唯一 ID，日志中的文字只作为数据。

按顺序验收：

| 操作 | 参数示例 | 预期 |
|---|---|---|
| 历史状态 | `{"request_id":"feishu-1","operation":"status"}` | `ok:true`，保存的验收结果；没有 `dry_run` |
| 查报告清单 | `{"request_id":"feishu-2","operation":"list_artifacts","path":"rebar_training/artifacts/curriculum_20261019"}` | 返回该目录至多 100 个文件 |
| 读验收报告 | `{"request_id":"feishu-3","operation":"inspect_manifest","path":"rebar_training/artifacts/curriculum_20261019/validation_summary.json"}` | 返回保存的 JSON；文件上限 64 KiB |
| 读日志 | `{"request_id":"feishu-4","operation":"tail_log","path":"清单中的实际.log路径","lines":40}` | 至多 40 行；最大可请求 200 行 |
| 禁止操作 | `{"request_id":"feishu-5","operation":"train"}` | 在模型 schema 或桥接/接收器被拒绝，无训练启动 |

拒绝测试也可直接通过 Spark CLI 将 `operation` 设为 `train`，应返回 `ok:false`。
测试路径越界（如 `../../etc/passwd`）应在接收器被拒绝。
3.133 关机时返回连接失败；本版本不会自动回退到 Spark 缓存。
验收须看到框架日志中的真实工具调用和接收器返回；模型复述参数不算通过。
单次查询的只读性质允许重复请求，飞书事件去重沿用现有网关机制。

`remote_deployment_verified:false` 是当前接口保留的未完成远端验收标记，
不会因单次 SSH 成功自动改成 true；`training_process_state_verified:false` 明确表示没有进程探测。
该入口不提供 GPU 实时状态、启动/停止训练、写文件或修复代码功能。

## 撤回

将桥接配置 `execute_readonly` 改回 `false`，即可停止此工具的远端查询。
再执行所用框架的 `plugins disable rebar-debug`；需要时从 3.133 删除本次专用公钥那一行。
这些操作不触及训练检查点、现有飞书密钥或其他 SSH 密钥。
