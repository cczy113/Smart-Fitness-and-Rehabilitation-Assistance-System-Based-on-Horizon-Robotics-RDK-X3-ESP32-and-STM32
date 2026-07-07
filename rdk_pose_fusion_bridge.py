#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RDK pose fusion bridge for LingJing.

Inputs:
- ROS2 ai_msgs/PerceptionTargets from mono2d_body_detection.
- ESP32 BLE Notify activation values from LS_ARM_BICEPS / LS_ARM_TRICEPS.

Outputs:
- WebSocket JSON server for Unity and other clients at ws://0.0.0.0:8765.
- The JSON keeps the Unity protocol skeleton/sensors/alert shape and adds
  angles/alignment/assessment fields for action correctness checks.
"""

import argparse
import asyncio
import json
import math
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

try:
    import dbus
    import dbus.mainloop.glib
    import dbus.service
    from gi.repository import GLib
except ImportError:
    dbus = None
    GLib = None

TROS_SITE_PACKAGES = "/opt/tros/humble/lib/python3.10/site-packages"
if TROS_SITE_PACKAGES not in sys.path:
    sys.path.append(TROS_SITE_PACKAGES)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from ai_msgs.msg import PerceptionTargets
except ImportError as exc:
    raise SystemExit(f"ROS2/TROS imports failed: {exc}")

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

try:
    import websockets
except ImportError:
    websockets = None

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c3319141"
CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a1"

PHONE_SERVICE_UUID = "7b6a2000-0f1d-4d2a-9f0c-8fd7a8d0f001"
PHONE_NOTIFY_CHAR_UUID = "7b6a2001-0f1d-4d2a-9f0c-8fd7a8d0f001"
PHONE_DEVICE_NAME = "LingJing-RDK"
STATUS_PATH = "/tmp/lingjing_fusion_status.txt"
STATUS_WRITE_INTERVAL = 0.5
_last_status_write_time = 0.0

DEFAULT_WS_HOST = "0.0.0.0"
DEFAULT_WS_PORT = 8765
DEFAULT_IMAGE_WIDTH = 640.0
DEFAULT_IMAGE_HEIGHT = 480.0

COCO_KEYPOINTS = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]

EMG_DEVICES = {
    # Current firmware uses these two names. The first device is treated as the
    # left biceps channel and the second as the right biceps channel for the
    # present two-arm verification demo. Rename here if the firmware names change.
    "LS_ARM_BICEPS": "biceps_l",
    "LS_ARM_TRICEPS": "biceps_r",
}

SENSOR_CHANNELS = ["biceps_l", "biceps_r"]


@dataclass
class EmgSample:
    activation: float = 0.0
    last_log_ms: int = 0
    raw: int = 0
    ac_rms: Optional[float] = None
    smooth: Optional[float] = None
    baseline: Optional[float] = None
    seq: Optional[int] = None
    flags: int = 0
    packet_version: str = ""
    timestamp_ms: int = 0
    connected: bool = False
    device_name: str = ""


@dataclass
class EmgPacket:
    activation: float
    raw: int
    detail: str
    ac_rms: Optional[float] = None
    smooth: Optional[float] = None
    baseline: Optional[float] = None
    seq: Optional[int] = None
    flags: int = 0
    packet_version: str = ""


@dataclass
class FusionState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    frame_id: int = 0
    last_pose_ms: int = 0
    keypoints: List[Dict] = field(default_factory=list)
    angles: Dict[str, Optional[float]] = field(default_factory=dict)
    emg: Dict[str, EmgSample] = field(
        default_factory=lambda: {name: EmgSample() for name in SENSOR_CHANNELS}
    )
    ros_frames: int = 0
    ros_targets: int = 0


state = FusionState()
ws_clients = set()
stop_event = threading.Event()


def now_ms() -> int:
    return int(time.time() * 1000)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def point_to_xy(point) -> Tuple[float, float]:
    return float(point.x), float(point.y)


def normalize_xy(x: float, y: float, width: float, height: float) -> Tuple[float, float]:
    if x > 1.0 or y > 1.0:
        return clamp(x / width, 0.0, 1.0), clamp(y / height, 0.0, 1.0)
    return clamp(x, 0.0, 1.0), clamp(y, 0.0, 1.0)


def angle_at_joint(a: Dict, b: Dict, c: Dict) -> Optional[float]:
    if not a or not b or not c:
        return None
    if min(a.get("conf", 0.0), b.get("conf", 0.0), c.get("conf", 0.0)) <= 0.05:
        return None

    ax, ay = a["x"], a["y"]
    bx, by = b["x"], b["y"]
    cx, cy = c["x"], c["y"]
    v1 = (ax - bx, ay - by)
    v2 = (cx - bx, cy - by)
    n1 = math.hypot(*v1)
    n2 = math.hypot(*v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return None
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    cosv = clamp(dot / (n1 * n2), -1.0, 1.0)
    return round(math.degrees(math.acos(cosv)), 2)


def pick_keypoint_container(target) -> Optional[object]:
    candidates = []
    for points in getattr(target, "points", []):
        pts = getattr(points, "point", [])
        candidates.append((getattr(points, "type", ""), len(pts), points))

    for point_type, count, points in candidates:
        low = point_type.lower()
        if count >= 17 and any(token in low for token in ("body", "kps", "key", "skeleton", "pose")):
            return points
    for _, count, points in candidates:
        if count >= 17:
            return points
    return None


def extract_keypoints(msg: PerceptionTargets, width: float, height: float) -> List[Dict]:
    best_target = None
    for target in msg.targets:
        point_container = pick_keypoint_container(target)
        if point_container is not None:
            best_target = (target, point_container)
            break

    if best_target is None:
        return []

    _, point_container = best_target
    points = list(point_container.point)[:17]
    confs = list(getattr(point_container, "confidence", []))
    keypoints = []
    for idx, point in enumerate(points):
        x, y = point_to_xy(point)
        nx, ny = normalize_xy(x, y, width, height)
        conf = float(confs[idx]) if idx < len(confs) else 1.0
        keypoints.append({
            "id": idx,
            "name": COCO_KEYPOINTS[idx],
            "x": round(nx, 5),
            "y": round(ny, 5),
            "conf": round(clamp(conf, 0.0, 1.0), 4),
        })
    return keypoints


def compute_angles(keypoints: List[Dict]) -> Dict[str, Optional[float]]:
    by_id = {kp["id"]: kp for kp in keypoints}
    left_elbow = angle_at_joint(by_id.get(5), by_id.get(7), by_id.get(9))
    right_elbow = angle_at_joint(by_id.get(6), by_id.get(8), by_id.get(10))
    shoulder_span_angle = None
    if 5 in by_id and 6 in by_id:
        dx = by_id[6]["x"] - by_id[5]["x"]
        dy = by_id[6]["y"] - by_id[5]["y"]
        shoulder_span_angle = round(math.degrees(math.atan2(dy, dx)), 2)
    return {
        "left_elbow_flexion_deg": left_elbow,
        "right_elbow_flexion_deg": right_elbow,
        "shoulder_span_deg": shoulder_span_angle,
    }


def sensor_payload() -> Dict:
    with state.lock:
        samples = {name: sample for name, sample in state.emg.items()}

    channels = {}
    for name in SENSOR_CHANNELS:
        sample = samples[name]
        status = "ok" if sample.connected and now_ms() - sample.timestamp_ms < 3000 else "disconnected"
        channels[name] = {
            "raw": sample.raw,
            "ac_rms": None if sample.ac_rms is None else round(sample.ac_rms, 2),
            "smooth": None if sample.smooth is None else round(sample.smooth, 2),
            "baseline": None if sample.baseline is None else round(sample.baseline, 2),
            "seq": sample.seq,
            "flags": sample.flags,
            "packet_version": sample.packet_version,
            "force_pct": round(sample.activation * 100.0, 1),
            "activation": round(sample.activation, 4),
            "timestamp": sample.timestamp_ms,
            "device": sample.device_name,
            "status": status,
        }
    sensor_status = "ok" if all(v["status"] == "ok" for v in channels.values()) else "partial"
    return {"channels": channels, "sensor_status": sensor_status}


def build_assessment(angles: Dict[str, Optional[float]], sensors: Dict) -> Tuple[Dict, Dict]:
    left = sensors["channels"].get("biceps_l", {})
    right = sensors["channels"].get("biceps_r", {})
    left_force = float(left.get("force_pct", 0.0))
    right_force = float(right.get("force_pct", 0.0))
    diff = abs(left_force - right_force)

    alert = {"type": "none", "message": ""}
    if left.get("status") == "ok" and right.get("status") == "ok" and diff > 30.0:
        alert = {"type": "bilateral_uneven", "message": "detected uneven left/right arm force"}

    left_angle = angles.get("left_elbow_flexion_deg")
    right_angle = angles.get("right_elbow_flexion_deg")
    angle_diff = None
    if left_angle is not None and right_angle is not None:
        angle_diff = round(abs(left_angle - right_angle), 2)
        if angle_diff > 35.0 and alert["type"] == "none":
            alert = {"type": "arm_angle_uneven", "message": "detected uneven left/right arm angle"}

    assessment = {
        "upper_arm": {
            "left_elbow_flexion_deg": left_angle,
            "right_elbow_flexion_deg": right_angle,
            "angle_diff_deg": angle_diff,
            "force_diff_pct": round(diff, 1),
            "status": "ok" if alert["type"] == "none" else "warn",
        }
    }
    return assessment, alert


def build_payload() -> Dict:
    with state.lock:
        frame_id = state.frame_id
        timestamp = state.last_pose_ms or now_ms()
        keypoints = list(state.keypoints)
        angles = dict(state.angles)

    sensors = sensor_payload()
    assessment, alert = build_assessment(angles, sensors)
    alignment = {}
    for name, ch in sensors["channels"].items():
        st = int(ch.get("timestamp") or 0)
        alignment[name] = {
            "emg_timestamp": st,
            "pose_timestamp": timestamp,
            "delta_ms": None if st == 0 else timestamp - st,
        }

    return {
        "timestamp": timestamp,
        "frame_id": frame_id,
        "type": "pose_fusion",
        "skeleton": {
            "keypoints": keypoints,
            "num_points": len(keypoints),
            "angles": angles,
        },
        "sensors": sensors,
        "alignment": alignment,
        "assessment": assessment,
        "alert": alert,
    }


def build_phone_payload() -> Dict:
    payload = build_payload()
    upper = payload.get("assessment", {}).get("upper_arm", {})
    channels = payload.get("sensors", {}).get("channels", {})
    left = channels.get("biceps_l", {})
    right = channels.get("biceps_r", {})
    return {
        "t": payload.get("timestamp"),
        "fid": payload.get("frame_id"),
        "la": upper.get("left_elbow_flexion_deg"),
        "ra": upper.get("right_elbow_flexion_deg"),
        "ad": upper.get("angle_diff_deg"),
        "lf": left.get("force_pct"),
        "rf": right.get("force_pct"),
        "fd": upper.get("force_diff_pct"),
        "st": upper.get("status"),
        "al": payload.get("alert", {}).get("type", "none"),
    }


def format_value(value, suffix=""):
    if value is None:
        return "NA"
    try:
        return f"{float(value):.1f}{suffix}"
    except (TypeError, ValueError):
        return "NA"


def format_status_summary(payload: Optional[Dict] = None) -> str:
    payload = payload or build_payload()
    upper = payload.get("assessment", {}).get("upper_arm", {})
    channels = payload.get("sensors", {}).get("channels", {})
    left = channels.get("biceps_l", {})
    right = channels.get("biceps_r", {})
    left_status = "OK" if left.get("status") == "ok" else "MISS"
    right_status = "OK" if right.get("status") == "ok" else "MISS"
    left_raw = left.get("raw")
    right_raw = right.get("raw")
    left_ac = left.get("ac_rms")
    right_ac = right.get("ac_rms")
    return (
        "ARM "
        f"L={format_value(upper.get('left_elbow_flexion_deg'))} "
        f"R={format_value(upper.get('right_elbow_flexion_deg'))} "
        f"dA={format_value(upper.get('angle_diff_deg'))} | "
        "ESP32 "
        f"L={left_status}:{format_value(left.get('force_pct'), '%')}/r{left_raw if left_raw is not None else 'NA'}/ac{format_value(left_ac)} "
        f"R={right_status}:{format_value(right.get('force_pct'), '%')}/r{right_raw if right_raw is not None else 'NA'}/ac{format_value(right_ac)} "
        f"dF={format_value(upper.get('force_diff_pct'), '%')}"
    )


def write_status_summary(payload: Optional[Dict] = None, force: bool = False):
    global _last_status_write_time
    current = time.time()
    if not force and current - _last_status_write_time < STATUS_WRITE_INTERVAL:
        return
    _last_status_write_time = current
    try:
        with open(STATUS_PATH, "w", encoding="utf-8") as f:
            f.write(format_status_summary(payload))
    except OSError:
        pass


def log_with_status(message: str):
    print(f"{message} | {format_status_summary()}", flush=True)


def encode_phone_payload() -> List[int]:
    data = json.dumps(build_phone_payload(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    # BLE notifications are small. Keep the phone frame compact and truncate as
    # a last resort so Unity still remains the full-fidelity skeleton channel.
    if len(data) > 180:
        data = data[:180]
    return [dbus.Byte(b) for b in data]


class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.freedesktop.DBus.Error.InvalidArgs"


class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotSupported"


class Application(dbus.service.Object):
    PATH_BASE = "/org/lingjing/pose_fusion"

    def __init__(self, bus):
        self.path = self.PATH_BASE
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method("org.freedesktop.DBus.ObjectManager", out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for char in service.characteristics:
                response[char.get_path()] = char.get_properties()
        return response


class Service(dbus.service.Object):
    def __init__(self, bus, index, uuid, primary=True):
        self.path = f"/org/lingjing/pose_fusion/service{index}"
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            "org.bluez.GattService1": {
                "UUID": self.uuid,
                "Primary": self.primary,
                "Characteristics": dbus.Array([c.get_path() for c in self.characteristics], signature="o"),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    @dbus.service.method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != "org.bluez.GattService1":
            raise InvalidArgsException()
        return self.get_properties()["org.bluez.GattService1"]


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + f"/char{index}"
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.notifying = False
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            "org.bluez.GattCharacteristic1": {
                "Service": self.service.get_path(),
                "UUID": self.uuid,
                "Flags": dbus.Array(self.flags, signature="s"),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != "org.bluez.GattCharacteristic1":
            raise InvalidArgsException()
        return self.get_properties()["org.bluez.GattCharacteristic1"]

    @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        return encode_phone_payload()

    @dbus.service.method("org.bluez.GattCharacteristic1")
    def StartNotify(self):
        self.notifying = True

    @dbus.service.method("org.bluez.GattCharacteristic1")
    def StopNotify(self):
        self.notifying = False

    @dbus.service.signal("org.freedesktop.DBus.Properties", signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def notify(self):
        if not self.notifying:
            return True
        self.PropertiesChanged(
            "org.bluez.GattCharacteristic1",
            {"Value": dbus.Array(encode_phone_payload(), signature="y")},
            [],
        )
        return True


class Advertisement(dbus.service.Object):
    PATH_BASE = "/org/lingjing/pose_fusion/advertisement"

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            "org.bluez.LEAdvertisement1": {
                "Type": "peripheral",
                "ServiceUUIDs": dbus.Array([PHONE_SERVICE_UUID], signature="s"),
                "LocalName": PHONE_DEVICE_NAME,
                "Includes": dbus.Array(["tx-power"], signature="s"),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != "org.bluez.LEAdvertisement1":
            raise InvalidArgsException()
        return self.get_properties()["org.bluez.LEAdvertisement1"]

    @dbus.service.method("org.bluez.LEAdvertisement1", in_signature="", out_signature="")
    def Release(self):
        pass


def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
    objects = remote_om.GetManagedObjects()
    for path, interfaces in objects.items():
        if "org.bluez.GattManager1" in interfaces and "org.bluez.LEAdvertisingManager1" in interfaces:
            return path
    return None


def phone_ble_thread_main(rate_hz: float):
    if dbus is None or GLib is None:
        print("[PHONE-BLE] dbus/gi not installed; phone BLE output disabled", flush=True)
        return
    try:
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SystemBus()
        adapter = find_adapter(bus)
        if not adapter:
            print("[PHONE-BLE] no BlueZ GATT/advertising adapter found", flush=True)
            return

        service_manager = dbus.Interface(bus.get_object("org.bluez", adapter), "org.bluez.GattManager1")
        ad_manager = dbus.Interface(bus.get_object("org.bluez", adapter), "org.bluez.LEAdvertisingManager1")
        app = Application(bus)
        service = Service(bus, 0, PHONE_SERVICE_UUID, True)
        char = Characteristic(bus, 0, PHONE_NOTIFY_CHAR_UUID, ["read", "notify"], service)
        service.add_characteristic(char)
        app.add_service(service)
        adv = Advertisement(bus, 0)

        service_manager.RegisterApplication(app.get_path(), {}, reply_handler=lambda: print("[PHONE-BLE] GATT registered", flush=True), error_handler=lambda e: print(f"[PHONE-BLE] GATT register error: {e}", flush=True))
        ad_manager.RegisterAdvertisement(adv.get_path(), {}, reply_handler=lambda: print(f"[PHONE-BLE] advertising as {PHONE_DEVICE_NAME}", flush=True), error_handler=lambda e: print(f"[PHONE-BLE] advertising error: {e}", flush=True))

        interval_ms = int(1000 / max(rate_hz, 1.0))
        GLib.timeout_add(interval_ms, char.notify)
        loop = GLib.MainLoop()
        loop.run()
    except Exception as exc:
        print(f"[PHONE-BLE] disabled due to error: {exc}", flush=True)


class PoseFusionNode(Node):
    def __init__(self, image_width: float, image_height: float):
        super().__init__("rdk_pose_fusion_bridge")
        self.image_width = image_width
        self.image_height = image_height
        self.last_log_ms = 0
        self.subscription = self.create_subscription(
            PerceptionTargets,
            "/hobot_mono2d_body_detection",
            self.listener_callback,
            10,
        )
        self.get_logger().info("Pose fusion bridge subscribed to /hobot_mono2d_body_detection")

    def listener_callback(self, msg: PerceptionTargets):
        keypoints = extract_keypoints(msg, self.image_width, self.image_height)
        angles = compute_angles(keypoints) if keypoints else {}
        ts = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1000000
        if ts <= 0:
            ts = now_ms()
        with state.lock:
            state.frame_id += 1
            state.last_pose_ms = ts
            state.keypoints = keypoints
            state.angles = angles
            state.ros_frames += 1
            if keypoints:
                state.ros_targets += 1

        current = now_ms()
        if current - self.last_log_ms >= 5000:
            self.last_log_ms = current
            write_status_summary(force=True)
            log_with_status(
                f"HB:fusion frames={state.ros_frames} targets={state.ros_targets} "
                f"clients={len(ws_clients)} keypoints={len(keypoints)}"
            )


def ros_thread_main(image_width: float, image_height: float):
    rclpy.init()
    node = PoseFusionNode(image_width, image_height)
    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_emg_packet(data: bytes) -> Optional[EmgPacket]:
    # ESP32-C3 EMG v2.2 sends a 16-byte packet:
    # magic "LJ", version=3, flags, seq, raw, acRMS*100, smooth*100,
    # baseline*100, activation*1000.
    if len(data) == 16 and data[0:2] == b"LJ" and data[2] == 3:
        _, _, version, flags, seq, raw, ac100, sm100, base100, act1000 = struct.unpack(
            "<ccBBHHHHHH", data
        )
        activation = clamp(act1000 / 1000.0, 0.0, 1.0)
        ac_rms = ac100 / 100.0
        smooth = sm100 / 100.0
        baseline = base100 / 100.0
        detail = (
            f"v={version} flags=0x{flags:02x} seq={seq} raw={raw} "
            f"ac={ac_rms:.2f} sm={smooth:.2f} base={baseline:.2f} act1000={act1000}"
        )
        return EmgPacket(
            activation=activation,
            raw=raw,
            detail=detail,
            ac_rms=ac_rms,
            smooth=smooth,
            baseline=baseline,
            seq=seq,
            flags=flags,
            packet_version="v3_full",
        )

    # ESP32-C3 EMG v2.0/v2.1 sends 2-byte little-endian uint16:
    # activation * 1000. This format has no true ADC raw value.
    if len(data) == 2:
        act1000 = data[0] | (data[1] << 8)
        activation = clamp(act1000 / 1000.0, 0.0, 1.0)
        return EmgPacket(
            activation=activation,
            raw=int(round(activation * 4095.0)),
            detail=f"v=legacy2 act1000={act1000} raw_est=1",
            packet_version="legacy2_estimated",
        )

    # Backward compatibility with old firmware that notified ASCII like "0.123".
    try:
        text = data.decode("utf-8", errors="ignore").strip()
        if text:
            activation = clamp(float(text), 0.0, 1.0)
            return EmgPacket(
                activation=activation,
                raw=int(round(activation * 4095.0)),
                detail=f"v=ascii text={text!r} raw_est=1",
                packet_version="ascii_estimated",
            )
    except ValueError:
        pass
    return None


def update_emg(channel_name: str, device_name: str, data: bytes):
    packet = parse_emg_packet(data)
    if packet is None:
        return
    ms = now_ms()
    should_log = False
    with state.lock:
        sample = state.emg[channel_name]
        sample.activation = packet.activation
        sample.raw = packet.raw
        sample.ac_rms = packet.ac_rms
        sample.smooth = packet.smooth
        sample.baseline = packet.baseline
        sample.seq = packet.seq
        sample.flags = packet.flags
        sample.packet_version = packet.packet_version
        sample.timestamp_ms = ms
        sample.connected = True
        sample.device_name = device_name
        if ms - sample.last_log_ms >= 5000:
            sample.last_log_ms = ms
            should_log = True
    if should_log:
        log_with_status(
            f"[BLE-DATA] {device_name}->{channel_name} len={len(data)} "
            f"{packet.detail} act={packet.activation:.3f}"
        )


def mark_emg_disconnected(channel_name: str):
    with state.lock:
        state.emg[channel_name].connected = False


async def connect_emg_device(device_name: str, channel_name: str):
    if BleakScanner is None or BleakClient is None:
        log_with_status("[BLE] bleak is not installed; EMG BLE input disabled")
        return

    while not stop_event.is_set():
        log_with_status(f"[BLE] scanning {device_name} for {channel_name}...")
        try:
            device = await BleakScanner.find_device_by_name(device_name, timeout=10)
        except Exception as exc:
            log_with_status(f"[BLE] scan error {device_name}: {exc}")
            await asyncio.sleep(5)
            continue

        if device is None:
            mark_emg_disconnected(channel_name)
            await asyncio.sleep(3)
            continue

        log_with_status(f"[BLE] connecting {device_name} {device.address}")
        try:
            async with BleakClient(device, timeout=30) as client:
                with state.lock:
                    state.emg[channel_name].connected = True
                    state.emg[channel_name].device_name = device_name
                await client.start_notify(
                    CHAR_UUID,
                    lambda sender, data, ch=channel_name, dn=device_name: update_emg(ch, dn, data),
                )
                while client.is_connected and not stop_event.is_set():
                    await asyncio.sleep(1.0)
        except Exception as exc:
            log_with_status(f"[BLE] {device_name} error: {exc}")

        mark_emg_disconnected(channel_name)
        await asyncio.sleep(2)


async def ws_handler(ws):
    ws_clients.add(ws)
    try:
        async for _ in ws:
            pass
    finally:
        ws_clients.discard(ws)


async def ws_broadcast_loop(rate_hz: float):
    interval = 1.0 / max(rate_hz, 1.0)
    while not stop_event.is_set():
        await asyncio.sleep(interval)
        if not ws_clients:
            continue
        payload = build_payload()
        write_status_summary(payload)
        msg = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        dead = set()
        for ws in list(ws_clients):
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        ws_clients.difference_update(dead)


async def main_async(args):
    if args.enable_phone_ble:
        threading.Thread(target=phone_ble_thread_main, args=(args.phone_ble_rate,), daemon=True).start()
    else:
        log_with_status("[PHONE-BLE] disabled by argument")

    tasks = []
    if args.enable_ble_input:
        for device_name, channel_name in EMG_DEVICES.items():
            tasks.append(asyncio.create_task(connect_emg_device(device_name, channel_name)))
    else:
        log_with_status("[BLE] EMG input disabled by argument")

    if websockets is None:
        log_with_status("[WS] websockets is not installed; Unity output disabled")
    else:
        server = await websockets.serve(ws_handler, args.ws_host, args.ws_port)
        log_with_status(f"[WS] Unity server ws://{args.ws_host}:{args.ws_port}")
        tasks.append(asyncio.create_task(ws_broadcast_loop(args.ws_rate)))
        tasks.append(asyncio.create_task(server.wait_closed()))

    if not tasks:
        while not stop_event.is_set():
            await asyncio.sleep(1)
    else:
        await asyncio.gather(*tasks)


def parse_args(argv: Optional[Iterable[str]] = None):
    parser = argparse.ArgumentParser(description="RDK pose + EMG fusion bridge")
    parser.add_argument("--ws-host", default=DEFAULT_WS_HOST)
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT)
    parser.add_argument("--ws-rate", type=float, default=20.0)
    parser.add_argument("--image-width", type=float, default=DEFAULT_IMAGE_WIDTH)
    parser.add_argument("--image-height", type=float, default=DEFAULT_IMAGE_HEIGHT)
    parser.add_argument("--disable-ble-input", action="store_true")
    parser.add_argument("--disable-phone-ble", action="store_true")
    parser.add_argument("--phone-ble-rate", type=float, default=10.0)
    return parser.parse_args(argv)


def main(argv: Optional[Iterable[str]] = None):
    args = parse_args(argv)
    args.enable_ble_input = not args.disable_ble_input
    args.enable_phone_ble = not args.disable_phone_ble
    ros_thread = threading.Thread(
        target=ros_thread_main,
        args=(args.image_width, args.image_height),
        daemon=True,
    )
    ros_thread.start()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ros_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
