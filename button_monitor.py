#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import signal
import subprocess
import time

try:
    import Hobot.GPIO as GPIO
except ImportError:
    import RPi.GPIO as GPIO

BUTTON_PIN = 28
LOG_PATH = "/root/lingjing_run.log"
RUN_SCRIPT = "/root/run_all.sh"
STATUS_PATH = "/tmp/lingjing_fusion_status.txt"

CHECK_INTERVAL = 2.0
BOOT_GRACE_SECONDS = 90.0
BRIDGE_STALE_SECONDS = 35.0
RESTART_COOLDOWN_SECONDS = 20.0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN)

process = None
log_file = None
started_at = 0.0
last_restart_at = 0.0
log_scan_pos = 0
last_bridge_seen_at = 0.0


def now_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def wall(message):
    os.system(f"wall '{message}'")



def fusion_status_suffix():
    try:
        with open(STATUS_PATH, "r", encoding="utf-8") as f:
            text = f.read().strip()
        return f" | {text}" if text else ""
    except OSError:
        return " | ARM L=NA R=NA dA=NA | ESP32 L=MISS:NA R=MISS:NA dF=NA"


def write_monitor_log(message):
    global log_file
    line = f"[monitor {now_text()}] {message}{fusion_status_suffix()}\n"
    print(line, end="", flush=True)
    if log_file:
        try:
            log_file.write(line)
            log_file.flush()
        except Exception:
            pass


def close_log_file():
    global log_file
    if log_file:
        try:
            log_file.close()
        except Exception:
            pass
        log_file = None


def kill_process_group(p):
    if not p:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except Exception:
        pass
    time.sleep(2.0)
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except Exception:
        pass


def force_cleanup():
    os.system("pkill -9 -f mono2d_body_detection")
    os.system("pkill -9 -f gimbal_to_stm32.py")
    os.system("pkill -9 -f rdk_pose_fusion_bridge.py")
    os.system("pkill -9 -f start_ai.sh")


def start_system(reason):
    global process, log_file, started_at, log_scan_pos, last_bridge_seen_at
    close_log_file()
    log_file = open(LOG_PATH, "a", buffering=1)
    log_file.write(f"\n\n===== LingJing start ({reason}) {now_text()} =====\n")
    log_file.flush()
    process = subprocess.Popen(
        ["/bin/bash", RUN_SCRIPT],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env={**os.environ, "PYTHONUNBUFFERED": "1"},
        preexec_fn=os.setsid,
    )
    started_at = time.time()
    log_scan_pos = os.path.getsize(LOG_PATH)
    last_bridge_seen_at = started_at
    write_monitor_log(f"started pid={process.pid} reason={reason}")


def stop_system(reason):
    global process
    write_monitor_log(f"stopping reason={reason}")
    kill_process_group(process)
    force_cleanup()
    process = None
    close_log_file()


def restart_system(reason):
    global last_restart_at
    current = time.time()
    if current - last_restart_at < RESTART_COOLDOWN_SECONDS:
        return
    last_restart_at = current
    wall("[LingJing] watchdog restarting stuck process")
    write_monitor_log(f"restart requested reason={reason}")
    stop_system(reason)
    start_system(f"watchdog:{reason}")


def is_running():
    return process is not None and process.poll() is None


def scan_bridge_heartbeat():
    global log_scan_pos, last_bridge_seen_at
    try:
        size = os.path.getsize(LOG_PATH)
        if size < log_scan_pos:
            log_scan_pos = 0
        with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
            f.seek(log_scan_pos)
            data = f.read()
            log_scan_pos = f.tell()
        if "HB:bridge" in data or "HB:fusion" in data or "TX:E" in data:
            last_bridge_seen_at = time.time()
    except OSError:
        pass


def health_check():
    if process is None:
        return

    if process.poll() is not None:
        restart_system(f"process exited code={process.returncode}")
        return

    if time.time() - started_at < BOOT_GRACE_SECONDS:
        return

    scan_bridge_heartbeat()
    age = time.time() - last_bridge_seen_at
    if age > BRIDGE_STALE_SECONDS:
        restart_system(f"bridge stale {age:.1f}s")


def handle_button_press():
    if is_running():
        wall("[LingJing] button stop")
        stop_system("button")
    else:
        wall("[LingJing] button start")
        start_system("button")


try:
    last_check = 0.0
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.05)
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                handle_button_press()
                while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                    time.sleep(0.1)

        current = time.time()
        if current - last_check >= CHECK_INTERVAL:
            health_check()
            last_check = current

        time.sleep(0.05)
except Exception as e:
    write_monitor_log(f"fatal monitor error: {e}")
finally:
    GPIO.cleanup()
