# -*- coding: utf-8 -*-
import sys
import serial
import time
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ai_msgs.msg import PerceptionTargets

SERIAL_PORT = "/dev/ttyS3"
BAUDRATE = 115200
SEND_INTERVAL = 0.10  # 10Hz is enough for servo tracking and STM32/OLED debug.
HEARTBEAT_INTERVAL = 5.0
TX_LOG_INTERVAL = 1.0
STATUS_CACHE_INTERVAL = 1.0
STATUS_PATH = "/tmp/lingjing_fusion_status.txt"
_status_cache = " | ARM L=NA R=NA dA=NA | ESP32 L=MISS:NA R=MISS:NA dF=NA"
_status_cache_time = 0.0

# 加载 ROS 2 环境
tros_path = '/opt/tros/humble/lib/python3.10/site-packages'
if tros_path not in sys.path:
    sys.path.append(tros_path)

def fusion_status_suffix():
    global _status_cache, _status_cache_time
    now = time.time()
    if now - _status_cache_time < STATUS_CACHE_INTERVAL:
        return _status_cache
    _status_cache_time = now
    try:
        with open(STATUS_PATH, "r", encoding="utf-8") as f:
            text = f.read().strip()
        _status_cache = f" | {text}" if text else ""
    except OSError:
        _status_cache = " | ARM L=NA R=NA dA=NA | ESP32 L=MISS:NA R=MISS:NA dF=NA"
    return _status_cache


def log_line(message):
    print(f"{message}{fusion_status_suffix()}", flush=True)


class RdkToStm32(Node):
    def __init__(self):
        super().__init__('rdk_to_stm32_node')

        # 串口初始化
        self.ser = None
        try:
            self.ser = self.open_serial()
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            sys.exit(1)

        self.center_x = 320.0
        self.last_send_time = 0.0
        self.last_heartbeat_time = 0.0
        self.last_tx_log_time = 0.0
        self.frame_count = 0
        self.target_count = 0

        # --- 滤波配置 ---
        self.error_history = []
        self.filter_size = 3 # 均值滤波窗口大小

        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_mono2d_body_detection',
            self.listener_callback,
            10)
        self.get_logger().info('Filtered Tracking Bridge Started!')

    def open_serial(self):
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1, write_timeout=0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser

    def reopen_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        time.sleep(0.5)
        self.ser = self.open_serial()
        self.get_logger().info("Serial port reopened")

    def listener_callback(self, msg):
        self.frame_count += 1
        now = time.time()
        if now - self.last_heartbeat_time >= HEARTBEAT_INTERVAL:
            log_line(f"HB:bridge frames={self.frame_count} targets={self.target_count}")
            self.last_heartbeat_time = now

        target_roi = None
        if len(msg.targets) > 0:
            for roi in msg.targets[0].rois:
                if roi.type == "body":
                    target_roi = roi
                    break

        if target_roi:
            self.target_count += 1
            # 1. 计算原始坐标偏差
            cx = target_roi.rect.x_offset + (target_roi.rect.width / 2)
            raw_error = int(cx - self.center_x)

            # 2. 均值滤波处理
            self.error_history.append(raw_error)
            if len(self.error_history) > self.filter_size:
                self.error_history.pop(0)

            avg_error = int(sum(self.error_history) / len(self.error_history))

            # 3. 控制发送频率
            # 频率太高会让 STM32 和 OLED 调试显示压力过大，10Hz 对舵机追踪足够。
            if now - self.last_send_time >= SEND_INTERVAL:
                send_msg = f"E{avg_error}\n"
                try:
                    self.ser.write(send_msg.encode("ascii"))
                    self.ser.flush()
                    if now - self.last_tx_log_time >= TX_LOG_INTERVAL:
                        log_line(f"TX:{send_msg.strip()} Filtered Error: {avg_error}")
                        self.last_tx_log_time = now
                except Exception as e:
                    self.get_logger().error(f"Serial Write Error: {e}")
                    try:
                        self.reopen_serial()
                    except Exception as reopen_error:
                        self.get_logger().error(f"Serial Reopen Error: {reopen_error}")
                        raise
                self.last_send_time = now

def main():
    rclpy.init()
    node = RdkToStm32()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if hasattr(node, "ser") and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
