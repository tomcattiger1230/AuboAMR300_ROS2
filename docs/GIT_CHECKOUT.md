# 仓库克隆、更新和跨平台兼容性

## 新电脑克隆

```bash
git clone --recurse-submodules https://github.com/tomcattiger1230/AuboAMR300_ROS2.git
cd AuboAMR300_ROS2
python3 scripts/check_git_portability.py
git submodule status --recursive
```

Git 和 Python 3 足以完成这些检查，不要求 ROS、Isaac 或 Git LFS。
构建及启动仍按各功能 README 配置对应环境。

## 已有副本更新

先保存本地开发修改，再执行：

```bash
git -c submodule.recurse=false pull --ff-only
git submodule sync --recursive
git submodule update --init --recursive
python3 scripts/check_git_portability.py
```

更新了 `.gitmodules` 后必须 sync，再初始化子模块，使本地 Git 配置采用新地址。
macOS 上修复前的副本可能已有大小写冲突造成的文件修改状态；保留旧副本，先在新目录递归克隆验证，再迁移自己的修改。

## Lyrical 子模块发布修复（2026-09-16）

此前主仓库锁定驱动提交 `a145c19`，驱动又锁定模型提交 `ba077a0`，
但官方远端均无法提供这两个本地 Lyrical 适配提交。主仓库 push 不会上传子模块提交；
新电脑的递归克隆及递归 pull 因此报 `not our ref`。

现将适配提交发布到维护 Fork 的 `lyrical` 分支，并从内向外更新地址及驱动指针：

| 仓库 | 发布地址 |
|---|---|
| 模型子模块 | https://github.com/tomcattiger1230/aubo_description |
| 驱动子模块 | https://github.com/tomcattiger1230/aubo_ros2_driver |

原 ROS 2 Lyrical 的 CMake、ros2_control API 和 realtime_tools 适配均保留。
主仓库及驱动中的 `.gitmodules` 记录 HTTPS 地址，克隆无需配置 GitHub SSH 密钥。
子模块以父仓库锁定的提交为准，常规更新不使用 `git submodule update --remote`。

维护顺序为：先提交并 push 内层模型，再在驱动中更新模型指针并 push，
最后在主仓库更新驱动指针并 push。官方仓库保留为 Ubuntu 子模块的 `upstream` remote。

## STL 独立重命名

以下大写与小写文件具有不同内容。大写版本独立归档重命名，小写活动版本继续保留，
每个归档文件及活动文件的 Git blob 哈希均保持原值：

| 原大写路径（meshes 下） | 归档新名称 | 活动版本 |
|---|---|---|
| adapter.STL | adapter_legacy.stl | adapter.stl |
| gripper_cube.STL | gripper_cube_legacy.stl | gripper_cube.stl |
| gripper_stick.STL | gripper_stick_legacy.stl | gripper_stick.stl |
| motor.STL | motor_legacy.stl | motor.stl |

路径前缀为 `seer_description/meshes/`。当前 stick URDF/Xacro/SDF 使用小写活动版本；
finger 变体仍使用 `finger_centered.stl` 和 `motor_new.stl`。
此次重命名不改变当前夹爪几何、坐标或钢筋装载参数。

`python3 scripts/check_git_portability.py` 检查已跟踪路径，发现仅大小写不同的重名时退出码为 1。
递归克隆成功、子模块状态没有 `+`/`-`、新工作区 `git status --short` 为空，才算完成 Git 验收。
