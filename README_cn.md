# 灵境守护 LingJing Guardian

面向康复训练和健身动作监测的多模态原型系统：RDK X3 进行人体姿态识别和数据融合，ESP32-C3 采集肌电，STM32F103 驱动二维舵机云台；RDK 通过 WebSocket 向微信小程序和 Unity 输出实时数据。

本仓库按“源码可复现、构建产物不入库”的原则整理。完整精简版约 **2 MB**，远小于 25 MB；不包含开发工具安装包、固件二进制、日志、备份和本机配置。所有文件均在压缩包内，可以自行根据本教程下载进行使用

## 系统架构

```text
USB Camera -> RDK X3 / TROS body detection -> pose fusion bridge -> WebSocket -> Mini Program / Unity
                                      |                                      
                                      +-> UART3 -> STM32F103 -> 2-axis servo pan-tilt

ESP32-C3 EMG node -> BLE Notify -> RDK pose fusion bridge
```

## 仓库目录

仓库仅保留各模块的核心源码、主要工程文件和通信文档：

```text
.
├── README.md
├── README_cn.md
├── docs/
│   └── Unity人体模型通信协议及使用方法.md
├── rdk/
│   ├── button_monitor.py
│   ├── gimbal_to_stm32.py
│   ├── rdk_pose_fusion_bridge.py
│   └── run_all.sh
├── miniprogram/
│   ├── app.js
│   ├── app.json
│   ├── app.wxss
│   ├── project.config.json
│   ├── sitemap.json
│   ├── pages/
│   └── utils/
├── esp32_emg/
│   ├── .gitignore
│   ├── platformio.ini
│   ├── README.md
│   ├── src/main.cpp
│   └── docs/ble_protocol.md
└── stm32_gimbal/
    └── LingJingGimbal/
        ├── Project.uvprojx
        ├── Start/
        ├── Library/
        ├── Hardware/
        └── User/
```

### 模块内容

| 模块 | 内容 | 说明 |
|---|---|---|
| RDK | `rdk_pose_fusion_bridge.py`、`gimbal_to_stm32.py`、`button_monitor.py`、`run_all.sh` | 姿态/肌电融合、云台串口桥、按键看门狗和启动脚本 |
| 微信小程序 | `小程序2` 的 `app.*`、`project.config.json`、`sitemap.json`、完整 `pages/`、完整 `utils/` | 当前使用的训练与监测小程序源码 |
| ESP32 | `肌电检测分支最终版/esp32_emg_firmware/esp32_emg_firmware` 中的 `.gitignore`、`platformio.ini`、`README.md`、`src/main.cpp`、`docs/ble_protocol.md` | PlatformIO 工程核心源码和 BLE 包协议 |
| STM32 | `舵机云台/舵机云台` 中的 `Project.uvprojx`、`Start/`、`Library/`、`Hardware/`、`User/` | Keil 工程及其实际引用的标准外设库源码 |
| Unity | `Unity人体模型通信协议及使用方法.md` | WebSocket 协议、COCO 17 点映射和 `RdkPoseClient.cs` 接入示例 |
| 仓库说明 | 本文件和 `README.md` | 中英文使用说明 |

### 未包含的内容

- 开发工具安装包、工程压缩包、固件镜像和其他二进制发布包。
- ESP32 的 `.pio/`、`.venv/`、`.vscode/`、`compile_commands.json`、`__pycache__/`、`*.pyc`、`*.bin`、`*.elf`、`*.map`。
- STM32 的 `Objects/`、`Listings/`、`DebugConfig/`、`*.uvoptx`、`*.uvguix.*`、`*.axf`、`*.o`、`*.d`、`*.crf`、`*.lst`、`*.htm`、`*.bak*`、工程压缩包。
- `rdk_backup/`、历史调试记录、日志、截图、录屏、系统镜像、开发者本机配置。
- `miniprogram/project.private.config.json` 和 `sourcemap.zip`。

小程序配置使用 `touristappid` 作为公开示例。使用者应在本地改为自己的 AppID；不要提交个人项目配置或私有开发者设置。

## 使用方法

### 1. ESP32-C3 肌电节点

在 VS Code 中用 PlatformIO 打开 `esp32_emg/`，然后构建、上传和查看串口：

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

- 默认采样引脚为 ESP32-C3 `GPIO0 / ADC1_CH0`，状态 LED 为 `GPIO8`。
- 第一块板保持 `DEVICE_NAME` 为 `LS_ARM_BICEPS`；第二块板烧录前改为 `LS_ARM_TRICEPS`。
- `platformio.ini` 中的 `upload_port` 和 `monitor_port` 是旧现场值，必须改成电脑当前枚举的串口，或删除两行由 PlatformIO 自动检测。
- 上电后约 8 秒为基线校准期，此时保持肌肉放松。BLE 服务和特征 UUID 见 `docs/ble_protocol.md`。

### 2. STM32F103 云台

用 Keil uVision 打开 `stm32_gimbal/LingJingGimbal/Project.uvprojx`，选择 `STM32F103C8` 目标后编译、下载。

- `PA10 / USART1_RX` 接收 RDK 的 `E{error}\n` 串口帧。
- `PA0 / TIM2_CH1` 输出 50 Hz 舵机 PWM。
- OLED 使用 `PB8/PB9` 软件 I2C。
- 舵机需要独立稳压供电，必须与 STM32 和 RDK 共地；不要从开发板 3.3V 引脚直接给舵机供电。

### 3. RDK X3

RDK 需要预先安装 TROS/ROS 2 人体检测环境，并能提供 `/hobot_mono2d_body_detection` 话题。将 `rdk/` 中四个文件复制到 RDK `/root/`：

```text
/root/button_monitor.py
/root/gimbal_to_stm32.py
/root/rdk_pose_fusion_bridge.py
/root/run_all.sh
```

安装 Python 依赖（以 RDK 实际镜像为准）：

```bash
python3 -m pip install bleak websockets pyserial
python3 -m py_compile /root/button_monitor.py /root/gimbal_to_stm32.py /root/rdk_pose_fusion_bridge.py
chmod +x /root/run_all.sh
```

`button_monitor.py` 还依赖 RDK 的 `Hobot.GPIO`（或兼容 GPIO 库）；融合桥依赖系统提供的 `dbus-python`、PyGObject、ROS 2/TROS `rclpy` 与 `ai_msgs`。

运行 `run_all.sh` 前，必须确认 RDK 已有匹配自身 TROS 版本的 `/root/start_ai.sh`，它负责启动摄像头和 `mono2d_body_detection`。该脚本与 TROS 镜像、模型和摄像头参数强绑定，当前精简仓库不包含它；不能用别的板子的脚本直接替换。

```bash
bash /root/run_all.sh
```

默认只连接左侧肌电板：

```bash
python3 /root/rdk_pose_fusion_bridge.py --ws-port 8765 --emg-device LS_ARM_BICEPS:biceps_l
```

如果已经烧录并打开两块 ESP32，再额外加入：

```bash
--emg-device LS_ARM_TRICEPS:biceps_r
```

RDK 通过 `ws://<RDK_IP>:8765` 以默认 20 Hz 广播 `pose_fusion` JSON。运行前按实际设备检查 `/dev/ttyS3`、摄像头、GPIO 引脚和蓝牙适配器。

### 4. 微信小程序

在微信开发者工具导入 `miniprogram/`：

1. 本地局域网调试使用 `ws://` 时，在开发者工具中关闭合法域名校验。
2. 在页面设置中填写 RDK WebSocket 地址，或在 `utils/rdkSocket.js` 修改默认地址为 `ws://<RDK_IP>:8765`。
3. 使用自己的 AppID 时，在微信公众平台把开发者加入项目成员；公开仓库仍保留 `touristappid` 或示例 AppID。
4. 正式发布通常必须配置 `wss://` 和合法 Socket 域名，局域网 `ws://` 仅用于开发调试。

### 5. Unity

Unity 作为 WebSocket Client 连接：

```text
ws://<RDK_IP>:8765
```

阅读 [Unity 人体模型通信协议及使用方法](docs/Unity人体模型通信协议及使用方法.md)。其中提供完整 JSON 字段、COCO 17 点映射、断线/空值/乱序处理要求，以及基于 NativeWebSocket 和 Newtonsoft Json 的 C# 示例。

仓库提供协议和接入示例，不包含可直接运行的 Unity 场景或 C# 工程。`X Bot@Shoved Reaction With Spin.fbx` 为可选动画模型资源，但单文件约 27.86 MiB，超过本仓库 25 MB 限制，应通过 Release、Git LFS 或其他独立下载方式分发。接入时需要按所用模型的 T-Pose 轴向校准骨骼旋转。

## 数据与安全边界

- 肌电通道为 `biceps_l` 和 `biceps_r`。设备断开或 3 秒未收到数据时，`force_pct` 与 `activation` 为 `null`，客户端必须显示为未连接，不能当作 0% 肌力。
- 单目二维姿态无法可靠重建深度；Unity 人体模型应通过骨骼旋转和 IK 近似驱动，不要直接把二维坐标赋给三维骨骼位置。
- 当前 WebSocket 是局域网明文连接，未包含认证机制。不要把 8765 端口暴露到公网。
- 当前 RDK 只取第一个 17 点人体目标，遮挡或多人进入画面时可能发生目标切换。

## 推荐 `.gitignore` 规则

```gitignore
# ESP32 / PlatformIO
.pio/
.venv/
.vscode/
compile_commands.json
__pycache__/
*.py[cod]
*.bin
*.elf
*.map

# Keil build and user state
Objects/
Listings/
DebugConfig/
*.uvoptx
*.uvguix.*
*.axf
*.o
*.d
*.crf
*.lst
*.htm
*.bak*

# Mini Program private/generated files
project.private.config.json
sourcemap.zip
*.zip
*.exe
```

## 许可证与复现说明

仓库尚未包含许可证文件。公开发布前请由项目作者选择并加入许可证（例如 MIT 或 Apache-2.0），并补充硬件接线图、RDK 系统/TROS 版本和所用模型版本。不同 RDK 镜像的启动脚本、模型路径和 Python 系统依赖不同，克隆仓库后不能假设可以在任意 RDK 上直接运行。
