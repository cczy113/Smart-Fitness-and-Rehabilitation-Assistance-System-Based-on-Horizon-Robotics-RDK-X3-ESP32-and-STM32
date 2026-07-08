# 灵境守护 LingJing Guardian

面向康复训练和健身动作监测的多模态动作分析系统。项目使用 RDK X3 做人体姿态识别与数据融合，使用 ESP32-C3 采集肌电信号，使用 STM32F103 驱动云台舵机，并通过 WebSocket 向 Unity 和微信小程序输出实时动作/肌电数据。

> 当前项目处于比赛原型阶段，代码以可复现、可调试、可演示为主。硬件接线、IP 地址、串口号和 AppID 需要按实际设备修改。

## 功能特性

- RDK X3 运行人体骨骼关键点识别，提取 COCO 17 点姿态数据。
- 计算上肢动作指标，包括左右肘关节屈伸角、左右角度差。
- ESP32-C3 通过 BLE Notify 发送真实肌电数据包，包括 `raw`、`acRMS`、`smooth`、`baseline`、`activation`。
- RDK 接收 ESP32 BLE 肌电数据并和视觉帧做时间戳对齐。
- RDK 通过 WebSocket 输出 Unity/小程序可用的实时 JSON。
- STM32F103 通过串口接收 RDK 视觉误差，输出 PWM 控制二维云台舵机。
- 物理按键启动/停止 RDK 全链路程序，带看门狗自动重启能力。
- 微信小程序可连接 RDK WebSocket 展示左右上臂肌电、左右肘角、动作状态和报告。

## 系统架构

```text
USB 摄像头
   |
   v
RDK X3 / TROS / ROS 2 / mono2d_body_detection
   |                         |
   | UART3 /dev/ttyS3        | WebSocket ws://RDK_IP:8765
   v                         v
STM32F103C8T6            Unity 人体模型 / 微信小程序
   |
   v
二维舵机云台

ESP32-C3 肌电采集节点
   |
   v
BLE Notify -> RDK X3 融合桥
```

## 目录结构

```text
.
├── button_monitor.py              # RDK 按键启动/停止与看门狗
├── gimbal_to_stm32.py             # RDK 姿态误差到 STM32 串口桥
├── rdk_pose_fusion_bridge.py      # RDK 姿态、肌电、Unity、小程序融合桥
├── run_all.sh                     # RDK 一键启动脚本
├── unity通信协议规范.md           # Unity WebSocket 数据协议说明
├── 项目介绍和调试记录.md          # 项目细节和调试历史
├── 舵机云台/                     # STM32F103 Keil 工程
├── 肌电检测分支最终版/            # ESP32-C3 EMG PlatformIO 工程
├── 小程序/                       # 微信小程序工程
└── rdk_backup/                   # RDK 脚本备份
```

## 硬件清单

- RDK X3 开发板
- USB 摄像头
- STM32F103C8T6 最小系统板
- ESP32-C3 开发板，两块更适合左右臂双通道演示
- 肌电传感器模块
- 二维云台和舵机
- 按键、10k 上拉电阻、杜邦线、外部舵机电源

## 硬件连接

### RDK X3 到 STM32

```text
RDK X3 Pin 8  UART3_TXD -> STM32 PA10 USART1_RX
RDK X3 Pin 10 UART3_RXD <- STM32 PA9 USART1_TX，可选
RDK X3 GND             -> STM32 GND
```

串口参数：

```text
设备：/dev/ttyS3
波特率：115200
格式：8N1
协议：E{error}\n
示例：E-42\n
```

### RDK X3 按键

```text
RDK X3 Pin 28 GPIO -> 按键一端
RDK X3 Pin 30 GND  -> 按键另一端
RDK X3 Pin 17 3.3V -> 10k 上拉电阻 -> Pin 28
```

### ESP32-C3 肌电采集

```text
ESP32-C3 GPIO0 / ADC1_CH0 -> 肌电传感器信号输出
ESP32-C3 3.3V 或 5V       -> 肌电传感器 VCC，按传感器规格选择
ESP32-C3 GND              -> 肌电传感器 GND
```

## 快速开始

### 1. RDK X3 端

将以下文件放到 RDK 的 `/root/`：

```text
/root/button_monitor.py
/root/gimbal_to_stm32.py
/root/rdk_pose_fusion_bridge.py
/root/run_all.sh
```

确保 `run_all.sh` 为 LF 行尾，不要使用 `set -u`，否则可能导致 TROS 环境脚本报 `AMENT_TRACE_SETUP_FILES: unbound variable`。

常用命令：

```bash
chmod +x /root/button_monitor.py /root/gimbal_to_stm32.py /root/rdk_pose_fusion_bridge.py /root/run_all.sh
python3 -m py_compile /root/button_monitor.py /root/gimbal_to_stm32.py /root/rdk_pose_fusion_bridge.py
systemctl restart button_monitor.service
systemctl status button_monitor.service --no-pager
```

按下物理按键后，RDK 会运行：

```text
run_all.sh
  -> start_ai.sh
  -> rdk_pose_fusion_bridge.py --ws-port 8765
  -> gimbal_to_stm32.py
```

查看实时日志：

```bash
tail -f /root/lingjing_run.log
```

### 2. ESP32-C3 肌电固件

工程路径：

```text
肌电检测分支最终版/esp32_emg_firmware/esp32_emg_firmware
```

使用 VS Code + PlatformIO 打开该目录，或在命令行执行：

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

第一块板默认设备名：

```cpp
#define DEVICE_NAME "LS_ARM_BICEPS"
```

第二块板烧录前改成：

```cpp
#define DEVICE_NAME "LS_ARM_TRICEPS"
```

上电后有约 8 秒校准期，校准期间保持肌肉放松。串口 CSV 格式：

```text
raw,acRMS,smooth,baseline,act
```

BLE Notify v3 数据包为 16 字节小端二进制：

```text
偏移  长度  含义
0     2     ASCII "LJ"
2     1     版本号 3
3     1     flags, bit0=1 表示校准中
4     2     seq
6     2     raw ADC
8     2     acRMS * 100
10    2     smooth * 100
12    2     baseline * 100
14    2     activation * 1000
```

### 3. STM32 云台固件

Keil 工程路径：

```text
舵机云台/舵机云台/Project.uvprojx
```

核心逻辑：

- USART1 RX：PA10，接收 RDK 串口协议 `E{error}\n`。
- TIM2_CH1：PA0，输出 50Hz 舵机 PWM。
- OLED 显示 `RX/F/B/PWM/HB` 等诊断信息。

OLED 诊断含义：

```text
F 增加：收到了完整 E...\n 帧
B 增加：PA10 有原始串口字节进入
C=0A：最后收到换行符，帧结尾正常
HB 增加：STM32 主循环正常
```

### 4. Unity 接入

RDK 作为 WebSocket Server：

```text
ws://<RDK_IP>:8765
```

默认示例：

```text
ws://192.168.137.230:8765
```

消息类型为 JSON，主要字段：

```json
{
  "timestamp": 0,
  "frame_id": 0,
  "type": "pose_fusion",
  "skeleton": {
    "keypoints": [],
    "num_points": 17,
    "angles": {
      "left_elbow_flexion_deg": 0,
      "right_elbow_flexion_deg": 0
    }
  },
  "sensors": {
    "channels": {
      "biceps_l": {
        "raw": 0,
        "force_pct": 0,
        "activation": 0,
        "status": "ok"
      }
    }
  },
  "assessment": {
    "upper_arm": {
      "status": "ok"
    }
  },
  "alert": {
    "type": "none"
  }
}
```

完整协议见：

```text
unity通信协议规范.md
```

### 5. 微信小程序

工程路径：

```text
小程序
```

当前小程序默认连接：

```text
ws://192.168.137.230:8765
```

本地调试说明：

- `project.config.json` 中的 `appid` 当前为 `touristappid`，用于避免“登录用户不是该小程序的开发者”导致无法调试。
- 正式上传或真机发布时，需要改回真实 AppID，并让管理员在微信公众平台把开发者微信号加入开发者/体验成员。
- 本地局域网调试 `ws://` 时，开发者工具需要关闭合法域名校验。
- 正式上线通常需要使用 `wss://` 和已配置的合法 socket 域名。

## RDK 日志示例

正常运行时可看到：

```text
HB:fusion frames=... targets=... clients=... keypoints=17 | ARM L=83.9 R=101.1 dA=17.2 | ESP32 L=OK:21.8%/r1096/ac8.2 R=MISS:0.0%/r0/acNA dF=21.8%
TX:E-74 Filtered Error: -74 | ARM L=81.8 R=104.2 dA=22.4 | ESP32 L=OK:27.7%/r1097/ac8.8 R=MISS:0.0%/r0/acNA dF=27.7%
[BLE-DATA] LS_ARM_BICEPS->biceps_l len=16 v=3 flags=0x00 seq=12 raw=1096 ac=8.23 sm=8.14 base=6.50 act1000=273 act=0.273
```

如果肌电仍为 0%，优先看：

- `raw` 是否随传感器输出变化。
- `ac` / `smooth` 是否随肌肉收缩上升。
- `baseline` 是否过高。
- `act1000` 是否长期为 0。

## 常见问题

### 微信开发者工具提示“登录用户不是该小程序的开发者”

本地调试可把 `project.config.json` 中的 `appid` 改为：

```json
"appid": "touristappid"
```

正式使用真实 AppID 时，需要项目管理员在微信公众平台添加你的微信号。

### RDK 启动后反复 watchdog restart

查看日志：

```bash
tail -f /root/lingjing_run.log
```

如果出现：

```text
/opt/tros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

检查 `run_all.sh` 是否误用了 `set -u`。

### ESP32 上传提示 COM 口被占用

关闭 PlatformIO Monitor 或其他串口工具，然后重新上传：

```bash
pio run -t upload
```

### RDK 能收到 ESP32，但肌电长期为 0%

查看 `[BLE-DATA] len=16` 中的真实字段：

```text
raw / ac / sm / base / act1000
```

如果 `raw/ac/smooth` 本身几乎不变，优先检查肌电传感器供电、GND、电极贴片、信号线和 ADC 引脚。
