# ETM-6M 抗渗仪与复合机器人 Isaac Sim 场景

场景 `urdf/warehouse_finger_etm6m_demo.usda` 叠加现有 SEER AMB-300 + AUBO i16H + finger 双指夹爪 + 单目相机复合机器人、仓库和 ETM-6M 设备。设备默认中心位于世界坐标 `(2.8, 0, 0)`，操作面朝向 `-X`，可从原点附近驶近。

## 启动

在已有 Isaac Sim / ROS 2 工作环境中构建并加载环境后：

```bash
ros2 run seer_description start_warehouse_finger_etm6m_demo.sh --gui --domain-id 133
```

也可以直接在 Isaac Sim 中打开 `urdf/warehouse_finger_etm6m_demo.usda`，仅查看场景。若要移动设备并重新生成 USD：

```bash
python3 seer_description/scripts/generate_etm6m_scene.py --x 2.8 --y 0 --yaw 0
```

启动脚本复用现有 finger 机器人控制栈；不要同时运行另一套发布相同关节、TF 和 action 的仿真实例。

## 设备模型范围

[厂家产品页](https://yhjtkj.com/list_52/881.html)将产品标为 **ETM-6M 型全自动密封混凝土抗渗仪**，网页给出的主机尺寸为 900 × 700 × 1100 mm，适用试件为 **Φ175 × Φ185 × H150 mm**，六模同时密封。按本项目的设备摆放要求，场景外包络改为 **长 900 × 宽 1100 × 高 900 mm**（局部 X × Y × Z，正面横向为宽）。网页正文个别位置写作“TDM-6M”；本场景沿用标题和产品照片上的 ETM-6M。USD 建立机柜、六个试位、手柄、挡板、压力表和触控屏的可辨认代理几何；机柜、托盘与试位等静态部件有碰撞。

模型是依据单张产品照片和公开尺寸制作的**近似静态资产**。试位中心距、内部结构、密封动作、压力与流体行为没有厂家 CAD/接口数据，当前没有仿真这些功能。设备未加入 MoveIt 规划场景；机械臂路径规划和自动上下料前，需补入相同位置与尺寸的规划碰撞体，并核对实物和夹爪尺寸。

生成脚本为 `scripts/generate_etm6m_scene.py`。厂家 CAD 到位后，可替换 `urdf/etm6m.usda`，保留 `World/ETM6M` 路径和场景 wrapper。
