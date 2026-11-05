# 海康 MV-CH100-60UM 相机内参标定

本流程用于实机 MV-CH100-60UM、12 mm 镜头。完成原始图像采集、内参/畸变求解、独立图像验证及结果预览；不执行机械臂运动。手眼、TCP、工作平面标定另做。

## 已检查的 Jetson 环境

2026-11-04 检查：`jetson@192.168.88.123`，aarch64，Python 3.10.12，OpenCV 4.5.4（含 aruco），NumPy、PyYAML、MVS 已安装。USB 相机序列号 `DB3017447`，原图 `4096×2460 Mono8`。检查时相机手动曝光 3000 µs、增益 0 dB。

已部署独立副本到 `/home/jetson/aubo_camera_calibration`，布局与本节命令一致。该副本来自本仓库的 `seer_description/scripts/`、`seer_description/config/calibration_board.json` 和本说明；不覆盖 `/home/jetson/aruco_test` 中的现有程序。

现有 `/home/jetson/aruco_test/hik_web_preview.py` 服务运行在 Jetson 的 `127.0.0.1:8080`，独占相机。推荐 `--source preview`：通过它的 `/api/snapshot` 获取**原始分辨率无损 PNG**，脚本在 Jetson 本机读文件；不从 `/preview.jpg` 取标定图像。该接口也会在原预览程序的 `captures_web` 留一份快照。

## 1. 准备标定板

默认 `config/calibration_board.json`：ChArUco，**7×5 个方格**，单格 **30 mm**，标记 **22 mm**，`DICT_5X5_100`；共 24 个内角点。尺寸单位统一为米。

Jetson 已生成 `/home/jetson/aubo_camera_calibration/board/board.png`。重新生成时使用新的输出目录：

```bash
ssh jetson@192.168.88.123
cd /home/jetson/aubo_camera_calibration
python3 scripts/calibrate_camera_intrinsics.py board \
  --board-config config/calibration_board.json --output board_new
```

在 Mac 另一终端取回图案：

```bash
scp jetson@192.168.88.123:/home/jetson/aubo_camera_calibration/board/board.png ~/Downloads/hik_calibration_board.png
```

图案区域（不含白边）应为 **210×150 mm**。默认生成图每格 300 像素、周围白边 100 像素，因此**整张图片打印尺寸应为 230×170 mm**；可用 A4 横向排版。PNG 不保证打印软件自动使用正确的物理尺寸，不要使用“适合页面”。打印后实测多个格子的总长及两个方向尺寸，确认单格 30 mm、标记 22 mm，并平贴到刚性哑光板上；纸张翘曲、非等比拉伸应重新制作。

若使用现有板，先修改 JSON：`charuco` 的 columns/rows 是**方格数**；`chessboard` 是**内角点数**。普通 10×7 方格棋盘填写 `kind: "chessboard", columns: 9, rows: 6`，`square_m` 必须换为实测值。ChArUco 还需匹配实际字典和标记尺寸。旧版 OpenCV 印制的偶数行 ChArUco 图案可能有兼容性差异；建议使用本流程生成的板，勿混用板型。

## 2. 看画面并固定设置

Mac 终端建立转发（如果本地 8080 已占用，可将命令中的第一个端口改为 8081）：

```bash
ssh -N -L 8080:127.0.0.1:8080 jetson@192.168.88.123
```

浏览器打开 <http://127.0.0.1:8080>。让标定板处在实际工作距离，对焦、调节光圈和照明，锁紧镜头与支架。保持手动曝光和固定增益，白格不过曝、黑格有层次、角点清晰。不要在采集中改变曝光/增益、ROI、图像翻转、分辨率、binning/decimation；也不要修改焦点。预览源会检查服务报告的尺寸、曝光和增益是否改变；ROI/翻转和镜头设置仍需人工固定并写入 notes。

**本脚本采集与现有网页中的“棋盘格自动采集”是两套流程。** 使用下面的脚本时，网页只用于观察画面和调曝光；默认 ChArUco 板不匹配网页现有的 9×6 普通棋盘检测器。

## 3. 无桌面采集（当前 Jetson 推荐）

在另一个 SSH 终端运行：

```bash
cd /home/jetson/aubo_camera_calibration
python3 -u scripts/capture_camera_calibration.py \
  --source preview --headless \
  --board-config config/calibration_board.json \
  --output calibration_data/session01 \
  --count 40 --every 2 \
  --notes 'MV-CH100-60UM DB3017447; 12mm; full 4096x2460; exposure 3000us; gain 0dB; focus/aperture locked; record actual working distance and ROI'
```

请把 `--notes` 换为实际设置。当前来源不用关闭现有预览，也不再次打开 SDK。脚本每隔约 2 秒尝试采集；只保存通过角点数、清晰度、板面积和视角差异检查的图像。`SAVED` 表示真正保存，`SKIP` 表示需要调整。40 张指**成功保存总数**，默认 32 张训练、8 张独立验证；每第 5 张划入验证集。

建议动作顺序：中央正视 → 左上/上方/右上 → 左侧/右侧 → 左下/下方/右下，再在各区域补充前后、左右倾斜和远近变化。倾斜约 ±15°～35°，确保清晰；每次调整后停稳再采集。可以固定机械臂、手动移动标定板完成内参标定。背景、相机和板均应避免振动。

默认近似重复的视角会被拒绝；倾斜或平移板后再停稳。连续 300 秒没有成功保存会停止，断流 10 秒会报错，Ctrl+C 保留已有数据。重新采集使用 `session02` 等新目录；不续写或覆盖旧数据。不要为了凑数量关闭重复检查。

文件结构：

```text
calibration_data/session01/
  session.json          # 板型、实际图像尺寸、相机信息、采集参数
  frames.jsonl          # 原始帧信息、时间戳、清晰度及覆盖率
  train/*.png          # 原始分辨率 Mono8，无文字/角点覆盖
  validation/*.png     # 不参与 K/D 拟合
```

## 4. 求解内参并查看报告

```bash
cd /home/jetson/aubo_camera_calibration
python3 scripts/calibrate_camera_intrinsics.py solve \
  --dataset calibration_data/session01 \
  --output calibration_data/result01 \
  --camera-name mv_ch100_60um_DB3017447
```

输出：

- `camera_intrinsics.yaml`：ROS `camera_info_manager` 格式，含 K、五参数 D（k1,k2,p1,p2,k3）、R、P、图像尺寸。
- `report.json`：总体/逐张 RMS、验证误差、拒绝图像、九宫格覆盖、估计倾角/距离变化及待检查项。
- `previews/train/`、`previews/validation/`：每张角点叠加图，各集合前 3 张的原分辨率去畸变预览。叠加图不用于求解。

初始图像验收目标：训练整体 RMS ≤0.5 px、每张训练/验证 RMS ≤1.0 px，至少 25 张训练与 5 张验证，角点覆盖全部九宫格，估计的两个方向倾角跨度均 ≥15°、距离变化 ≥1.2 倍。最后两项是采样多样性提醒，不是物理精度认证。

`PASS` 返回码 0；`REVIEW_REQUIRED` 返回码 2，**仍输出候选 YAML 和报告**；输入/运行错误返回码 1。检查报告状态后再使用参数。误差高的图片不会自动删除：先看角点图，检查模糊/反光/板型/尺寸，补拍或人工记录后移走问题图片，再输出到 `result02`。禁止只删高误差图片凑合格值。训练不足 8 张无法求解，尺寸混用和重复图像（含训练/验证重复）直接拒绝。

验证图像只用固定 K/D 估计每张板姿态并计算重投影误差；不回填训练集。低重投影误差不能证明抓取已达到毫米精度。需要结合真实直线、已知尺寸、工作距离范围进一步检查，之后再做手眼标定。

取回结果：

```bash
scp -r jetson@192.168.88.123:/home/jetson/aubo_camera_calibration/calibration_data/result01 ~/Downloads/hik_result01
```

## 5. 其他采集入口

### 独立 MVS 采集

仅在没有现有预览/其他驱动占用相机时使用。先在原终端正常退出占用程序。SDK 默认自动寻找 aarch64/x86_64 的 MvImport，不在默认目录时用 `--sdk-path` 指定。

```bash
cd /home/jetson/aubo_camera_calibration
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 scripts/capture_camera_calibration.py --source mvs --list-devices
python3 -u scripts/capture_camera_calibration.py \
  --source mvs --serial DB3017447 --headless \
  --exposure-us 3000 --gain-db 0 \
  --board-config config/calibration_board.json \
  --output calibration_data/session_mvs01 --count 40 --every 2
```

MVS 入口关闭触发、自动曝光和自动增益，设为 Mono8；未指定曝光/增益时保留打开相机时的数值。记录可读出的 ROI、采样和翻转设置，不自动修改它们。设置不写入设备 UserSet；退出后不保证恢复此前的会话设置，恢复预览时检查实际读值。

有本地桌面可去掉 `--headless`：`s` 保存训练图、`v` 保存验证图、`a` 开关自动采集、`q`/Esc 退出。窗口缩小仅影响显示，保存和检测保留原图像素。无 DISPLAY 的 SSH 使用 headless 模式。

### ROS 2 原图采集

在运行真实相机驱动的 Ubuntu ROS 2 主机上，从仓库根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select seer_description
source install/setup.bash
ros2 run seer_description capture_camera_calibration.py \
  --source ros --topic /camera/image_raw --headless \
  --board-config "$(ros2 pkg prefix seer_description)/share/seer_description/config/calibration_board.json" \
  --output calibration_data/session_ros01 --count 40 --every 2
ros2 run seer_description calibrate_camera_intrinsics.py solve \
  --dataset calibration_data/session_ros01 --output calibration_data/result_ros01
```

按实际安装替换 ROS 发行版、话题和 ROS_DOMAIN_ID。ROS 入口使用 sensor-data QoS；驱动需要发布原始未去畸变的 `mono8`（也支持标准 RGB/BGR8 转灰度），拒绝静默缩放 10/12/16 位图像。相机设置通过驱动固定并写入 notes。不要误采 Isaac 仿真的同名话题。

### 本地依赖与结果加载

Jetson 当前依赖已齐全。其他 Ubuntu 可安装 `python3-opencv python3-numpy python3-yaml`，ROS 入口还需对应发行版的 `cv_bridge`、`sensor_msgs` 和 `rclpy`。使用独立虚拟环境时安装 `opencv-contrib-python>=4.5,<5`、`numpy`、`PyYAML`；不要在同一环境重复安装多个 OpenCV wheel。OpenCV 4.5 的旧 ChArUco API 和 4.10 的新 API 均受支持。

将验收后的 YAML 交给真实相机驱动的 `camera_info_manager` 加载，驱动需按原图时间戳和 `camera_optical_frame` 发布 `/camera/camera_info`。本流程不自动改驱动、URDF 或启动第二个 CameraInfo 发布器；各驱动的参数名不同，应按实际驱动配置。全幅 K 不可直接配 1024×615 预览、ROI 或其他采样模式；去畸变输出也必须使用对应的 K/P，避免重复去畸变。

## 实现参考

- [OpenCV ChArUco 标定](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html)
- [ROS 相机内参格式与 CameraInfo 支持](https://github.com/ros-perception/image_common)
- [海康 MVS 下载中心](https://www.hikrobotics.com/en/machinevision/service/download/?module=0)，实际 API 对照 Jetson 已安装的 SDK `Samples/aarch64/Python/General/GrabImage/GrabImage.py`。
