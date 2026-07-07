#!/bin/bash

# Clean old runtime processes before starting the integrated system.
pkill -9 -f mono2d_body_detection 2>/dev/null || true
pkill -9 -f gimbal_to_stm32.py 2>/dev/null || true
pkill -9 -f rdk_pose_fusion_bridge.py 2>/dev/null || true
sleep 1

export PATH=$PATH:/usr/local/bin:/usr/bin
source /opt/tros/humble/setup.bash

chmod 666 /dev/ttyS3 2>/dev/null || true

cleanup() {
    pkill -9 -f mono2d_body_detection 2>/dev/null || true
    pkill -9 -f rdk_pose_fusion_bridge.py 2>/dev/null || true
    pkill -9 -f gimbal_to_stm32.py 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[run_all] starting AI vision brain..."
bash /root/start_ai.sh &
AI_PID=$!

sleep 5

echo "[run_all] starting pose fusion bridge for Unity/EMG/mobile..."
PYTHONUNBUFFERED=1 python3 /root/rdk_pose_fusion_bridge.py --ws-port 8765 &
FUSION_PID=$!

sleep 2

echo "[run_all] starting STM32 gimbal bridge..."
PYTHONUNBUFFERED=1 python3 /root/gimbal_to_stm32.py
