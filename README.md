# LingJing Guardian (Lean Open-Source Edition)

LingJing Guardian is a multimodal prototype for rehabilitation and fitness monitoring. An RDK X3 runs body-pose inference and data fusion, ESP32-C3 nodes acquire EMG data, and an STM32F103 controls a two-axis servo pan-tilt. The RDK broadcasts fused data to a WeChat Mini Program and Unity through WebSocket.

This release keeps reproducible source code and excludes generated artifacts. The lean repository is about **2 MB**, well below 25 MB. It does not include tool installers, archives, binaries, logs, backups, or machine-specific settings.

[中文说明 / Chinese README](README_cn.md)

## Architecture

```text
USB Camera -> RDK X3 / TROS body detection -> pose fusion bridge -> WebSocket -> Mini Program / Unity
                                      |
                                      +-> UART3 -> STM32F103 -> 2-axis servo pan-tilt

ESP32-C3 EMG node -> BLE Notify -> RDK pose fusion bridge
```

## Repository Layout

Only core source files, main project files, and communication documentation are kept in this repository:

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
│   ├── app.js, app.json, app.wxss
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
└── stm32_gimbal/LingJingGimbal/
    ├── Project.uvprojx
    ├── Start/
    ├── Library/
    ├── Hardware/
    └── User/
```

| Component | Contents | Purpose |
|---|---|---|
| RDK | `rdk_pose_fusion_bridge.py`, `gimbal_to_stm32.py`, `button_monitor.py`, `run_all.sh` | Fusion, gimbal serial bridge, button watchdog, launcher |
| Mini Program | `小程序2`: `app.*`, `project.config.json`, `sitemap.json`, all `pages/`, all `utils/` | Current training and monitoring application source |
| ESP32 | `.gitignore`, `platformio.ini`, `README.md`, `src/main.cpp`, `docs/ble_protocol.md` under `肌电检测分支最终版/esp32_emg_firmware/esp32_emg_firmware` | PlatformIO source and BLE packet definition |
| STM32 | `Project.uvprojx`, `Start/`, `Library/`, `Hardware/`, `User/` under `舵机云台/舵机云台` | Keil project and every source file referenced by it |
| Unity | `Unity人体模型通信协议及使用方法.md` | WebSocket specification, COCO-17 mapping, and a `RdkPoseClient.cs` integration example |

Not included in this repository:

- Tool installers, project archives, firmware images, and other binary release packages.
- ESP32 `.pio/`, `.venv/`, `.vscode/`, `compile_commands.json`, caches, and compiled firmware.
- STM32 `Objects/`, `Listings/`, `DebugConfig/`, `*.uvoptx`, `*.uvguix.*`, object files, build output, backups, and archives.
- `rdk_backup/`, logs, screenshots, videos, images, historical debug records, and device images.
- `project.private.config.json` and `sourcemap.zip` from the Mini Program.

The Mini Program configuration uses `touristappid` as a public example. Set your own AppID locally when needed, and do not commit personal project configuration or private developer settings.

## Quick Start

### ESP32-C3 EMG node

Open `esp32_emg/` in VS Code with PlatformIO:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

- EMG input: `GPIO0 / ADC1_CH0`; status LED: `GPIO8`.
- Keep `DEVICE_NAME` as `LS_ARM_BICEPS` on the first board. Change it to `LS_ARM_TRICEPS` before flashing a second board.
- `upload_port` and `monitor_port` in `platformio.ini` are site-specific. Replace them with the active serial port or remove both entries for automatic detection.
- Keep muscles relaxed during the approximately 8-second power-on calibration. See `docs/ble_protocol.md` for the BLE service, characteristic, and binary packet.

### STM32F103 pan-tilt

Open `stm32_gimbal/LingJingGimbal/Project.uvprojx` in Keil uVision, select the STM32F103C8 target, then build and flash.

- `PA10 / USART1_RX` receives `E{error}\n` frames from the RDK.
- `PA0 / TIM2_CH1` outputs 50 Hz servo PWM.
- The OLED uses software I2C on `PB8/PB9`.
- Power servos from a separate regulated supply and connect its ground to the STM32/RDK ground. Do not power servos from the board's 3.3 V pin.

### RDK X3

The RDK needs a TROS/ROS 2 body-detection setup that publishes `/hobot_mono2d_body_detection`. Copy the four files from `rdk/` to `/root/` on the board:

```text
/root/button_monitor.py
/root/gimbal_to_stm32.py
/root/rdk_pose_fusion_bridge.py
/root/run_all.sh
```

Install the Python packages appropriate for the RDK image:

```bash
python3 -m pip install bleak websockets pyserial
python3 -m py_compile /root/button_monitor.py /root/gimbal_to_stm32.py /root/rdk_pose_fusion_bridge.py
chmod +x /root/run_all.sh
```

The RDK image must also provide `dbus-python`, PyGObject, TROS/ROS 2 `rclpy`, `ai_msgs`, and `Hobot.GPIO` (or a compatible GPIO library).

Before running `run_all.sh`, verify that a matching `/root/start_ai.sh` already exists on the RDK. It launches the camera and `mono2d_body_detection`, and is tightly coupled to the TROS image, model, and camera configuration. It is intentionally not included in this lean repository.

```bash
bash /root/run_all.sh
```

The default bridge connects one EMG board:

```bash
python3 /root/rdk_pose_fusion_bridge.py --ws-port 8765 --emg-device LS_ARM_BICEPS:biceps_l
```

Add `--emg-device LS_ARM_TRICEPS:biceps_r` only when the second ESP32 has been flashed and powered. The bridge broadcasts `pose_fusion` JSON at 20 Hz to `ws://<RDK_IP>:8765`.

### WeChat Mini Program

Import `miniprogram/` into WeChat DevTools.

1. Disable domain validation in DevTools for local `ws://` development.
2. Set the RDK address in the app UI, or edit `utils/rdkSocket.js` to use `ws://<RDK_IP>:8765`.
3. Use your own AppID and add developers in the WeChat Public Platform for physical-device tests.
4. Production deployment normally requires `wss://` and an approved Socket domain; LAN `ws://` is development-only.

### Unity

Unity acts as a WebSocket client:

```text
ws://<RDK_IP>:8765
```

Read [Unity Human Model Communication Protocol and Usage](docs/Unity人体模型通信协议及使用方法.md). It specifies all JSON fields, COCO-17 mapping, null/disconnect handling, and includes a NativeWebSocket + Newtonsoft Json C# client example.

The repository provides the protocol and integration example, but not a runnable Unity scene or C# project. `X Bot@Shoved Reaction With Spin.fbx` is an optional animated model asset; at about 27.86 MiB it exceeds this repository's 25 MB limit and should be distributed separately through a Release, Git LFS, or another download channel. Bone rotations must be calibrated to the imported model's T-pose axes.

## Data and Security Notes

- EMG channels are `biceps_l` and `biceps_r`. On disconnect or after three seconds without an update, `force_pct` and `activation` are `null`; clients must display no data rather than 0% force.
- A monocular 2D pose cannot reliably recover depth. Drive a 3D model with constrained bone rotation/IK instead of assigning 2D coordinates directly to bone positions.
- The current WebSocket service is plaintext LAN-only and has no authentication. Never expose port 8765 directly to the public Internet.
- The current RDK bridge selects the first target with 17 keypoints, so target switching can occur with multiple people or occlusion.

## Recommended `.gitignore`

```gitignore
.pio/
.venv/
.vscode/
compile_commands.json
__pycache__/
*.py[cod]
*.bin
*.elf
*.map
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
project.private.config.json
sourcemap.zip
*.zip
*.exe
```

## License and Reproducibility

No license file is included yet. The project owner should choose and add a license (for example MIT or Apache-2.0) before public release, together with wiring diagrams, RDK/TROS versions, and the model version. Startup scripts, model paths, and system dependencies differ between RDK images, so a clone is not expected to run on any RDK without board-specific deployment work.
