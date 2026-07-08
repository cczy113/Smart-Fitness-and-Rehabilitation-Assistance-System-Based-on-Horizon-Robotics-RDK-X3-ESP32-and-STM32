# LingJing Guardian

A multimodal motion analysis system targeting rehabilitation training and fitness movement monitoring. This project leverages RDK X3 for human pose recognition and data fusion, adopts ESP32-C3 to collect electromyography (EMG) signals, employs STM32F103 to drive pan-tilt servos, and transmits real-time motion and EMG data to Unity and WeChat Mini Programs via WebSocket.

> This project is currently at the competition prototype stage, with codes prioritizing reproducibility, debuggability and demonstration. Hardware wiring configurations, IP addresses, serial port numbers and AppIDs need modification according to actual deployed devices.

## Functional Features
- RDK X3 executes human skeleton key point recognition to extract COCO 17-point pose data.
- Calculates upper limb motion metrics, including flexion angles of left and right elbow joints, as well as the angular difference between the two sides.
- ESP32-C3 transmits raw EMG data packets through BLE Notify, containing fields: `raw`, `acRMS`, `smooth`, `baseline`, `activation`.
- The RDK receives BLE EMG data from ESP32 and aligns timestamps with visual image frames.
- RDK outputs real-time JSON data compatible with Unity and Mini Programs via WebSocket.
- STM32F103 receives visual error data from RDK through serial ports and generates PWM signals to control the 2-axis pan-tilt servos.
- Physical buttons support starting and stopping the full-link program on RDK, equipped with a watchdog for automatic system restart.
- WeChat Mini Program connects to the RDK WebSocket to display left and right upper arm EMG values, left and right elbow angles, motion status and analysis reports.

## System Architecture
```text
USB Camera
   |
   v
RDK X3 / TROS / ROS 2 / mono2d_body_detection
   |                         |
   | UART3 /dev/ttyS3        | WebSocket ws://RDK_IP:8765
   v                         v
STM32F103C8T6            Unity Human Body Model / WeChat Mini Program
   |
   v
2-Axis Servo Pan-Tilt

ESP32-C3 EMG Collection Node
   |
   v
BLE Notify -> RDK X3 Fusion Bridge
```

## Directory Structure
```text
.
├── button_monitor.py              # RDK button trigger for program start/stop and watchdog management
├── gimbal_to_stm32.py             # Serial bridge transmitting pose error data from RDK to STM32
├── rdk_pose_fusion_bridge.py      # Fusion bridge integrating pose data, EMG signals, Unity and Mini Program data streams
├── run_all.sh                     # One-click startup script for RDK
├── Unity Communication Protocol Specification.md           # Documentation of WebSocket data protocol for Unity
├── Project Introduction & Debug Logs.md          # Project details and historical debugging records
├── Servo Pan-Tilt/                     # STM32F103 Keil engineering project
├── Final EMG Detection Branch/            # ESP32-C3 EMG PlatformIO engineering project
├── MiniProgram/                       # WeChat Mini Program engineering project
└── rdk_backup/                   # Backup folder for RDK scripts
```

## Hardware Bill of Materials
- RDK X3 Development Board
- USB Camera
- STM32F103C8T6 Minimum System Board
- Two ESP32-C3 Development Boards (dual-channel demonstration for left and right arms recommended)
- EMG Sensor Modules
- 2-Axis Pan-Tilt Bracket with Servos
- Physical Buttons, 10k Pull-up Resistors, Jumper Wires, External Power Supply for Servos

## Hardware Wiring Connections
### RDK X3 to STM32
```text
RDK X3 Pin 8  UART3_TXD -> STM32 PA10 USART1_RX
RDK X3 Pin 10 UART3_RXD <- STM32 PA9 USART1_TX (optional)
RDK X3 GND             -> STM32 GND
```
Serial Port Parameters:
```text
Device: /dev/ttyS3
Baud Rate: 115200
Format: 8N1
Protocol: E{error}\n
Example: E-42\n
```

### RDK X3 Button Circuit
```text
RDK X3 Pin 28 GPIO -> One terminal of the button
RDK X3 Pin 30 GND  -> The other terminal of the button
RDK X3 Pin 17 3.3V -> 10k Pull-up Resistor -> Pin 28
```

### ESP32-C3 EMG Sensor Wiring
```text
ESP32-C3 GPIO0 / ADC1_CH0 -> Signal Output Pin of EMG Sensor
ESP32-C3 3.3V or 5V       -> VCC Pin of EMG Sensor (select based on sensor specifications)
ESP32-C3 GND              -> GND Pin of EMG Sensor
```

## Quick Start Guide
### 1. RDK X3 Side Deployment
Place the following files under `/root/` directory of RDK:
```text
/root/button_monitor.py
/root/gimbal_to_stm32.py
/root/rdk_pose_fusion_bridge.py
/root/run_all.sh
```
Ensure `run_all.sh` uses LF line endings. Avoid using `set -u`, otherwise the TROS environment script may throw the error `AMENT_TRACE_SETUP_FILES: unbound variable`.

Common Commands:
```bash
chmod +x /root/button_monitor.py /root/gimbal_to_stm32.py /root/rdk_pose_fusion_bridge.py /root/run_all.sh
python3 -m py_compile /root/button_monitor.py /root/gimbal_to_stm32.py /root/rdk_pose_fusion_bridge.py
systemctl restart button_monitor.service
systemctl status button_monitor.service --no-pager
```
After pressing the physical button, the RDK executes the following program chain:
```text
run_all.sh
  -> start_ai.sh
  -> rdk_pose_fusion_bridge.py --ws-port 8765
  -> gimbal_to_stm32.py
```
View real-time running logs:
```bash
tail -f /root/lingjing_run.log
```

### 2. ESP32-C3 EMG Firmware
Project Path:
```text
Final EMG Detection Branch/esp32_emg_firmware/esp32_emg_firmware
```
Open the directory with VS Code + PlatformIO, or run commands in terminal:
```bash
pio run
pio run -t upload
pio device monitor -b 115200
```
Default device name for the first board:
```cpp
#define DEVICE_NAME "LS_ARM_BICEPS"
```
Modify the definition for the second board before flashing firmware:
```cpp
#define DEVICE_NAME "LS_ARM_TRICEPS"
```
An approximate 8-second calibration phase initiates after power-on; keep muscles relaxed during calibration. Serial port CSV output format:
```text
raw,acRMS,smooth,baseline,act
```
BLE Notify v3 data packet is a 16-byte little-endian binary structure:
```text
Offset  Length  Definition
0       2       ASCII header "LJ"
2       1       Version number 3
3       1       Flags; bit0=1 indicates calibration in progress
4       2       Sequence number
6       2       Raw ADC reading
8       2       acRMS multiplied by 100
10      2       smooth value multiplied by 100
12      2       baseline value multiplied by 100
14      2       activation value multiplied by 1000
```

### 3. STM32 Pan-Tilt Firmware
Keil Project Path:
```text
Servo Pan-Tilt/Servo Pan-Tilt/Project.uvprojx
```
Core Logic:
- USART1 RX pin PA10: Receives serial protocol frames `E{error}\n` from RDK
- TIM2_CH1 pin PA0: Outputs 50Hz PWM signals to drive servos
- OLED screen displays diagnostic metrics including `RX/F/B/PWM/HB`

Explanation of OLED diagnostic indicators:
```text
Increment F: Complete frame formatted as E...\n received
Increment B: Raw serial byte input detected on PA10
C=0A: Line feed character received, marking valid frame end
Increment HB: STM32 main loop runs normally
```

### 4. Unity Integration
The RDK acts as a WebSocket Server with address format:
```text
ws://<RDK_IP>:8765
```
Default test address example:
```text
ws://192.168.137.230:8765
```
All transmitted messages are JSON objects with core fields as follows:
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
Refer to the full communication specification file:
```text
Unity Communication Protocol Specification.md
```

### 5. WeChat Mini Program
Project Directory Path:
```text
MiniProgram
```
Default WebSocket connection address preset in the Mini Program:
```text
ws://192.168.137.230:8765
```
Local Debug Instructions:
- The `appid` field in `project.config.json` is set to `touristappid` by default, which avoids the debugging restriction "The logged-in user is not a developer of this Mini Program".
- For official submission or physical device release, replace the placeholder with the official AppID, and the project administrator must add your WeChat ID as a developer/tester on the WeChat Public Platform backend.
- Disable valid domain name verification in the developer tools when debugging local LAN `ws://` connections.
- Official online deployment generally requires `wss://` protocol and pre-configured authorized socket domain names.

## RDK Log Sample Output
Typical normal running log content:
```text
HB:fusion frames=... targets=... clients=... keypoints=17 | ARM L=83.9 R=101.1 dA=17.2 | ESP32 L=OK:21.8%/r1096/ac8.2 R=MISS:0.0%/r0/acNA dF=21.8%
TX:E-74 Filtered Error: -74 | ARM L=81.8 R=104.2 dA=22.4 | ESP32 L=OK:27.7%/r1097/ac8.8 R=MISS:0.0%/r0/acNA dF=27.7%
[BLE-DATA] LS_ARM_BICEPS->biceps_l len=16 v=3 flags=0x00 seq=12 raw=1096 ac=8.23 sm=8.14 base=6.50 act1000=273 act=0.273
```
If EMG activation percentage remains 0%, prioritize troubleshooting these items:
- Whether the `raw` value changes with sensor input signals
- Whether `ac` / `smooth` values rise when muscles contract
- Whether the `baseline` value is excessively high
- Whether `act1000` stays zero persistently

## Common Troubleshooting
### WeChat Developer Tool Error: "The logged-in user is not a developer of this Mini Program"
Modify the `appid` field in `project.config.json` for local debugging:
```json
"appid": "touristappid"
```
When using the official real AppID, contact the project administrator to add your WeChat account on the WeChat Public Platform backend.

### Repeated Watchdog Restarts After RDK Startup
Check runtime logs with this command:
```bash
tail -f /root/lingjing_run.log
```
If the following error log appears:
```text
/opt/tros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```
Verify that the `run_all.sh` script does not contain the `set -u` command.

### ESP32 Firmware Upload Failure: COM Port Occupied
Close PlatformIO Serial Monitor or other serial port tools, then re-execute upload command:
```bash
pio run -t upload
```

### RDK Receives ESP32 BLE Packets But EMG Activation Stays 0%
Inspect raw fields in logs tagged `[BLE-DATA] len=16`:
```text
raw / ac / sm / base / act1000
```
If `raw/ac/smooth` values barely fluctuate, inspect EMG sensor power supply, GND wiring, electrode patches, signal cables and ADC input pins first.
