# 灵境守护 — 通信协议规范

---

## 一、传感器 → RDK（BLE/串口）

### 帧格式

```
| 帧头(2B) | 序列号(2B) | 时间戳(4B) | 通道1(2B) | 通道2(2B) | 通道3(2B) | 通道4(2B) | 校验和(1B) |
| 0xAA55   | seq++      | ms         | 股四头肌  | 腘绳肌    | 腰椎      | 辅助      | XOR校验    |
```

- 帧头：`0xAA 0x55`
- 序列号：uint16，每帧+1，用于检测丢帧
- 时间戳：uint32，毫秒级（ESP32开机计时）
- 通道数据：uint16，ADC原始值（0-4095）
- 校验和：前面所有字节异或

### 心跳包

- 每2秒发送一次，帧格式同上，通道数据全填0xFFFF
- RDK端超过5秒未收到心跳→标记传感器断连

---

## 二、RDK → Unity（WebSocket）

### 连接

- 协议：WebSocket
- 地址：`ws://<RDK_IP>:8765`
- Unity作为Client连接RDK Server

### JSON数据格式

```json
{
  "timestamp": 1717500000123,
  "frame_id": 12345,
  "skeleton": {
    "keypoints": [
      {"id": 0, "name": "nose",           "x": 0.51, "y": 0.22, "conf": 0.95},
      {"id": 1, "name": "left_eye",       "x": 0.49, "y": 0.18, "conf": 0.92},
      {"id": 2, "name": "right_eye",      "x": 0.53, "y": 0.18, "conf": 0.93},
      {"id": 3, "name": "left_ear",       "x": 0.45, "y": 0.20, "conf": 0.88},
      {"id": 4, "name": "right_ear",      "x": 0.57, "y": 0.20, "conf": 0.89},
      {"id": 5, "name": "left_shoulder",  "x": 0.38, "y": 0.40, "conf": 0.96},
      {"id": 6, "name": "right_shoulder", "x": 0.62, "y": 0.40, "conf": 0.97},
      {"id": 7, "name": "left_elbow",     "x": 0.32, "y": 0.55, "conf": 0.94},
      {"id": 8, "name": "right_elbow",    "x": 0.68, "y": 0.55, "conf": 0.95},
      {"id": 9, "name": "left_wrist",     "x": 0.28, "y": 0.68, "conf": 0.91},
      {"id": 10,"name": "right_wrist",    "x": 0.72, "y": 0.68, "conf": 0.92},
      {"id": 11,"name": "left_hip",       "x": 0.42, "y": 0.62, "conf": 0.94},
      {"id": 12,"name": "right_hip",      "x": 0.58, "y": 0.62, "conf": 0.95},
      {"id": 13,"name": "left_knee",      "x": 0.41, "y": 0.78, "conf": 0.93},
      {"id": 14,"name": "right_knee",     "x": 0.59, "y": 0.78, "conf": 0.94},
      {"id": 15,"name": "left_ankle",     "x": 0.40, "y": 0.93, "conf": 0.90},
      {"id": 16,"name": "right_ankle",    "x": 0.60, "y": 0.93, "conf": 0.91}
    ],
    "num_points": 17
  },
  "sensors": {
    "channels": {
      "quadriceps_l": {"raw": 2048, "force_pct": 55.2, "status": "ok"},
      "quadriceps_r": {"raw": 1800, "force_pct": 48.1, "status": "ok"},
      "hamstring_l":  {"raw": 1200, "force_pct": 32.5, "status": "ok"},
      "hamstring_r":  {"raw": 1150, "force_pct": 30.8, "status": "ok"},
      "lumbar":       {"raw": 900,  "force_pct": 24.1, "status": "ok"},
      "aux":          {"raw": 0,    "force_pct": 0.0,  "status": "disconnected"}
    },
    "sensor_status": "partial"
  },
  "alert": {
    "type": "none",
    "message": ""
  }
}
```

### 字段说明

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 毫秒时间戳 |
| frame_id | uint32 | 帧序号 |
| skeleton.keypoints | array | 骨骼关键点，x/y为归一化坐标(0-1)，conf为置信度 |
| sensors.channels | object | 各通道原始值+肌力百分比+状态 |
| alert.type | string | 预警类型：none/bilateral_uneven/lumbar_compensation |
| alert.message | string | 预警语音文本 |

### 预警类型

| alert.type | 触发条件 | 语音播报 |
|------------|----------|----------|
| none | 无异常 | — |
| bilateral_uneven | 左右差值>30% | "检测到双侧发力不均" |
| lumbar_compensation | 腰椎发力超阈值 | "检测到腰部代偿发力，请保持核心收紧" |

---

## 三、骨骼关键点编号（COCO 17点）

| ID | 名称 | ID | 名称 |
|----|------|----|------|
| 0 | nose | 9 | left_wrist |
| 1 | left_eye | 10 | right_wrist |
| 2 | right_eye | 11 | left_hip |
| 3 | left_ear | 12 | right_hip |
| 4 | right_ear | 13 | left_knee |
| 5 | left_shoulder | 14 | right_knee |
| 6 | right_shoulder | 15 | left_ankle |
| 7 | left_elbow | 16 | right_ankle |
| 8 | right_elbow | | |

### 骨骼连接关系（用于点线渲染）

```
0-1, 1-3, 0-2, 2-4,          # 头部
5-6,                           # 肩膀
5-7, 7-9,                      # 左臂
6-8, 8-10,                     # 右臂
5-11, 6-12,                    # 躯干
11-12,                         # 髋部
11-13, 13-15,                  # 左腿
12-14, 14-16                   # 右腿
```
