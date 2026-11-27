# 启元 Q1 探索版

## 独立关节卡扩展

本分支 `feat/q1-joints` 以 PR #584 来源提交 `16e578f7d2fb2a8ac3892e22b0b58f397f90a798` 为基线，仅增加 joints/model。原有 battery 保留，不包含表情、独立 IMU 卡、麦克风、ASR 或姿态控制。

`joints` 订阅 `/aima/hal/joint/state`，按名称校验 22 个活动关节，20 Hz 发布 `/primebot_q1/state/joints`，格式 `sensor/skeleton`。字段包括 q（rad）、dq（rad/s）、tau（N·m）、温度、源时间戳及 waiting/live/stale/error。关节卡内部订阅原始 IMU，将归一化的 wxyz 四元数放入 imu_quat，无须添加或连接另一张 IMU 卡；姿态超过 0.5 秒失效，关节超过 1 秒失效。该映射沿用调试设备以 pelvis 为参考的配置，其他固件仍需核对。

`model` 返回配套精简 Q1 v3 URDF，只含运动学结构，不含实体 mesh 或动力学惯量，不用于仿真训练。模型来自用户提供的 q1_v3 包；公开发布前需确认再分发许可。

已有 Core 骨架渲染器可使用本分支数据。实体模型、叠加模式和自动取景属于 Core 配套改动，保留于父项目的 core-changes 补丁包，不放入 Driver 分支。

### 验证与已知问题

已完成 6 项无需 ROS 的契约测试。此前机载验证曾收到 22 关节、约 20 Hz 及姿态数据；后续发现肘部角度显示差异时，实际数据为 waiting，不能宣称已完成肘角精度验收。等待数据时现有前端可能显示默认或旧姿态，后续须先恢复 live，再核对零位、方向和模型匹配。

```bash
PYTHONPATH=primebot/q1 python3 -m unittest discover -s primebot/q1/tests -p test_joints.py -v
```

镜像及启动脚本均已加入 joints.py 和 q1_v3.urdf。ARM64 镜像仍需在适配环境构建验证，本次未修改机器人运行环境。

`battery` 是只读传感器卡。在 Motus 画布添加「启元 Q1 探索版 → battery」后，打开「查看数据流」可查看实时电池读数。驱动启动后，卡片每 0.5 秒向 Core 发布一次 JSON，默认话题为 `/primebot_q1/battery`，格式为 `data/json`。

数据来自 `/aima/hal/bms/state`（`aimdk_msgs/msg/Bms`）。画布 JSON 的 `battery` 对象包含以下字段。

| 指标 | 字段 | 单位 |
| --- | --- | --- |
| 荷电状态（SOC） | `percentage` | %，直接取自 BMS 剩余容量百分比 |
| 电压、电流、输出功率 | `voltage_v`、`current_a`、`power_w` | V、A、W，电流为负表示放电 |
| 电池温度 | `temperature_c` | °C |
| 剩余容量 | `remaining_capacity_mah` | mAh |
| 完整循环次数 | `cycle_count` | 次 |
| 累计充放电容量 | `cycle_total_capacity_mah` | mAh |
| 厂商电池状态位 | `status_bits` | BMS 原始位掩码 |

`info` 返回卡片状态、输出话题与最近读数。driver 默认持续发布电池数据，Motus 的系统动作 `stop` 和 `start` 可以暂停和恢复发布。`battery.state` 为 `waiting` 表示等待首个样本，`live` 表示样本在 5 秒以内，`stale` 表示最后读数已过期。`age_s` 给出样本年龄。 已保存的旧画布需要将 `status` 卡换成 `battery`，并将下游连接指向 `/primebot_q1/battery`。

## ARM64 Docker 部署

Apple 芯片 Mac 和 Linux ARM64（Jetson／板载机）使用同一套 Ubuntu 22.04 / ROS 2 Humble 驱动镜像。先启动同一台部署主机上的 Motus Core，并接通 Q1 的机器人网段。主机需要 Python 3、Docker 和 Docker Compose v2。在驱动仓库根目录运行：

```bash
python3 primebot/q1/deploy/q1.py build
python3 primebot/q1/deploy/q1.py start
python3 primebot/q1/deploy/q1.py status
python3 primebot/q1/deploy/q1.py stop
```

`build` 在首次部署或更新驱动源码时运行，生成 `primebot-q1:local`。已有 ARM64 驱动镜像时，可用 `start --image <镜像名>`。启动后，`status` 显示 Core 注册情况和最近的电池样本。看到 `registered` 和 `live` 后，在 Motus 画布添加 `battery` 卡片并打开「查看数据流」。`stop` 停止 Q1 服务，Core 继续运行。

- **Apple 芯片 Mac**使用 Docker Desktop。启动命令同时管理宿主机的 DDS 转发、容器内的 UDP 转发和 driver。见 [Mac 有线联调说明](tests/mac/README.md)。
- **Linux ARM64** 上，Core 和 Q1 driver 都使用主机网络，driver 直接通过 Q1 网段读取厂商 DDS。脚本默认读取主机的 `/opt/phanthy-motus/dds-local.xml`，并连接 `https://localhost:15678`。若本机 Core 使用 HTTP 或 XML 在其他位置，可运行 `start --core-url http://127.0.0.1:15678 --core-dds /实际路径/dds-local.xml`。团队的 Linux 部署流程也可继续使用原有 [部署配置](deploy/service.yml)。

镜像编译启元 SDK 提交 `618d995d4362f981ab5945624f8be9343c2cb68b` 的 `aimdk_msgs` 包。构建从[启元开发者社区](https://dev.primebot.com/download)下载官方 v1.1.8.0 SDK，校验 ZIP 的 SHA-256，仅将 `aimdk_msgs` 放进编译工作空间。相同 ZIP 的镜像地址可用构建参数 `Q1_SDK_URL` 指定。colcon 和 rosidl 生成消息接口，`nav_msgs`、`sensor_msgs` 是厂商包声明的消息依赖，Fast DDS 和 PyYAML 分别用于通信与加载驱动配置。机器人读取进程在域 0 使用厂商 DDS 配置，Motus 侧在域 42 使用 Core 的 DDS 配置。Core 通过 `http://localhost:15748/mcp` 访问驱动。驱动沿用 X2 的 Motus 公共运行时。[启元消息源码](https://github.com/PrimebotTech/primebot-q1-sdk/tree/618d995d4362f981ab5945624f8be9343c2cb68b/aimdk_msgs) · [X2 驱动](../../agibot/AimDK_X2/main.py)

在已接入 Q1 的 Linux ROS 环境加载启元工作空间后，可用下列命令核对原始 BMS 消息。

```bash
timeout 5 ros2 topic echo /aima/hal/bms/state --once
```

启元官方快速入门给出开发者模式、Ubuntu 22.04 / Humble 环境、网线直连和 BMS 读取步骤。电池消息字段与状态位可查接口手册。[SDK 快速入门与集成指南](https://github.com/PrimebotTech/primebot-q1-sdk/blob/main/docs/zh/SDK快速入门与集成指南.md) · [接口说明](https://raw.githubusercontent.com/PrimebotTech/primebot-q1-sdk/main/docs/zh/接口说明.md)

本地集成测试及真机验证范围见[验证记录](验证记录.md)。
