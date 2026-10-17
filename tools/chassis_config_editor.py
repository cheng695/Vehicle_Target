#!/usr/bin/env python3
"""Serve a local, offline editor for config/chassis.yaml."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import sys
from urllib.parse import urlparse

import yaml

from generate_chassis_config import render
from generate_gimbal_config import render as render_gimbal


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "chassis.yaml"
PAGE_PATH = ROOT / "tools" / "chassis_config_editor.html"
GIMBAL_PAGE_PATH = ROOT / "tools" / "gimbal_config_editor.html"
HOME_PAGE_PATH = ROOT / "tools" / "config_editor_home.html"
GEAR_PAGE_PATH = ROOT / "tools" / "gear_explanation.html"
GIMBAL_CONFIG_PATH = ROOT / "config" / "gimbal.yaml"
HOST = "127.0.0.1"
PORT = 8765


def parse_config(text: str) -> dict:
    config = yaml.safe_load(text)
    validate_config(config)
    return config


def validate_config(config: dict) -> None:
    render(config)  # Reuse firmware generator schema and type validation.

    positive_values = {
        "geometry.chassis_radius_m": config["geometry"]["chassis_radius_m"],
        "geometry.wheel_radius_m": config["geometry"]["wheel_radius_m"],
        "control_period_s": config["control_period_s"],
        "patrol.speed_mps": config["patrol"]["speed_mps"],
        "patrol.distance_m": config["patrol"]["distance_m"],
        "patrol.vehicle_turn_radius_m": config["patrol"]["vehicle_turn_radius_m"],
        "patrol.max_wheel_speed_mps": config["patrol"]["max_wheel_speed_mps"],
    }
    for name, value in positive_values.items():
        if value <= 0:
            raise ValueError(f"{name} must be greater than zero")

    remote = config["remote"]
    if not 0 <= remote["deadzone"] < 1:
        raise ValueError("remote.deadzone must be in the range [0, 1)")
    if remote["linear_gain_mps"] < 0 or remote["rotation_gain_radps"] < 0:
        raise ValueError("remote speed gains cannot be negative")

    ramp = config["translation_ramp"]
    if ramp["increase_per_cycle"] < 0 or ramp["decrease_per_cycle"] < 0:
        raise ValueError("translation ramp values cannot be negative")

    patrol = config["patrol"]
    if patrol["stop_linear_threshold_mps"] < 0 or patrol["stop_angular_threshold_radps"] < 0:
        raise ValueError("patrol stop thresholds cannot be negative")
    if config["patrol"]["settle_cycles"] < 1:
        raise ValueError("patrol.settle_cycles must be at least 1")

    motor = config["motor"]
    for key in ("init_can_id", "send_can_id"):
        if not 0 <= motor[key] <= 0x7FF:
            raise ValueError(f"motor.{key} must be a standard CAN ID (0..2047)")
    ids = motor["receive_ids"]
    if any(not 1 <= motor_id <= 8 for motor_id in ids) or len(set(ids)) != len(ids):
        raise ValueError("motor.receive_ids must be unique IDs in the range 1..8")


def format_yaml_value(value) -> str:
    if isinstance(value, bool):
        raise ValueError("Boolean values are not valid chassis parameters")
    if isinstance(value, float):
        return format(value, ".15g")
    if isinstance(value, int):
        return str(value)
    if isinstance(value, list):
        return "[" + ", ".join(format_yaml_value(item) for item in value) + "]"
    raise ValueError(f"Unsupported YAML value type: {type(value).__name__}")


def replace_value(lines: list[str], start: int, end: int, key: str, value, formatted=None) -> None:
    pattern = re.compile(r"^(\s*" + re.escape(key) + r":\s*)([^#]*?)(\s+#.*)?$")
    for index in range(start, end):
        body = lines[index].rstrip("\r\n")
        match = pattern.match(body)
        if match:
            newline = lines[index][len(body):]
            comment = match.group(3) or ""
            replacement = format_yaml_value(value) if formatted is None else formatted
            lines[index] = match.group(1) + replacement + comment + newline
            return
    raise ValueError(f"Could not find YAML key '{key}' to update")


def section_bounds(lines: list[str], section: str) -> tuple[int, int]:
    header = re.compile(r"^" + re.escape(section) + r":\s*(?:#.*)?$")
    start = next((i for i, line in enumerate(lines) if header.match(line.rstrip("\r\n"))), None)
    if start is None:
        raise ValueError(f"Could not find YAML section '{section}'")
    end = len(lines)
    for index in range(start + 1, len(lines)):
        body = lines[index].rstrip("\r\n")
        if body and not body[0].isspace() and not body.startswith("#"):
            end = index
            break
    return start + 1, end


def update_yaml(original: str, config: dict) -> str:
    render(config)  # Validate values before producing or saving YAML.
    lines = original.splitlines(keepends=True)

    replace_value(lines, 0, len(lines), "wheel_count", config["wheel_count"])
    top_control = next((i for i, line in enumerate(lines) if re.match(r"^control_period_s:", line)), None)
    if top_control is None:
        raise ValueError("Could not find YAML key 'control_period_s'")
    replace_value(lines, top_control, top_control + 1, "control_period_s", config["control_period_s"])

    mappings = {
        "geometry": config["geometry"],
        "wheel_layout": config["wheel_layout"],
        "translation_ramp": config["translation_ramp"],
        "remote": config["remote"],
        "patrol": config["patrol"],
        "motor": config["motor"],
    }
    for section, values in mappings.items():
        start, end = section_bounds(lines, section)
        for key, value in values.items():
            formatted = None
            if section == "motor" and key in ("init_can_id", "send_can_id"):
                formatted = f"0x{int(value):X}"
            replace_value(lines, start, end, key, value, formatted)

    start, end = section_bounds(lines, "wheel_pid")
    row_indices = [
        index for index in range(start, end)
        if re.match(r"^\s*-\s*\[", lines[index])
    ]
    rows = config["wheel_pid"]
    if len(row_indices) != len(rows):
        raise ValueError("The YAML wheel_pid rows do not match the configured wheel count")
    for index, row in zip(row_indices, rows):
        body = lines[index].rstrip("\r\n")
        match = re.match(r"^(\s*-\s*)(\[[^]]*\])(\s+#.*)?$", body)
        if not match:
            raise ValueError("Could not parse a wheel_pid row in the project YAML")
        newline = lines[index][len(body):]
        comment = match.group(3) or ""
        lines[index] = match.group(1) + format_yaml_value(row) + comment + newline

    updated = "".join(lines)
    parse_config(updated)
    return updated


def parse_gimbal_config(text: str) -> dict:
    config = yaml.safe_load(text)
    validate_gimbal_config(config)
    return config


def validate_gimbal_config(config: dict) -> None:
    render_gimbal(config)

    yaw = config["yaw"]
    hight = config["hight"]
    if yaw["target_limit_radps"] <= 0 or yaw["constant_speed_limit_radps"] <= 0:
        raise ValueError("Yaw 速度上限必须大于零")
    if hight["travel_deg"] <= 0 or hight["max_speed_radps"] <= 0:
        raise ValueError("云台行程和升降速度上限必须大于零")
    if not 0 <= hight["slowdown_start_ratio"] <= 1:
        raise ValueError("hight.slowdown_start_ratio 必须在 [0, 1] 范围内")
    if hight["min_control_dt_s"] <= 0 or hight["max_control_dt_s"] < hight["min_control_dt_s"]:
        raise ValueError("升降控制周期范围无效")

    limits = config["limit_and_stall"]
    for key in ("hight_lower_limit_active_low", "hight_upper_limit_active_low", "stall_detection_enabled"):
        if not isinstance(limits[key], bool):
            raise ValueError(f"limit_and_stall.{key} 必须为布尔值")

    for section_name in ("hight_motor", "yaw_motor"):
        motor = config[section_name]
        count = motor["count"]
        if count < 1:
            raise ValueError(f"{section_name}.count 必须至少为 1")
        ids_key = "feedback_ids"
        if len(motor[ids_key]) != count:
            raise ValueError(f"{section_name}.feedback_ids 数量必须与 count 一致")
        if any(not 1 <= motor_id <= 8 for motor_id in motor[ids_key]):
            raise ValueError(f"{section_name}.feedback_ids 必须在 1..8 范围内")
        if len(set(motor[ids_key])) != len(motor[ids_key]):
            raise ValueError(f"{section_name}.feedback_ids 不能重复")


def format_gimbal_yaml_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return format_yaml_value(value)


def update_gimbal_yaml(original: str, config: dict) -> str:
    validate_gimbal_config(config)
    lines = original.splitlines(keepends=True)

    section_fields = {
        "yaw": {key: value for key, value in config["yaw"].items() if key != "variable_speed"},
        "hight": config["hight"],
        "pid": {key: value for key, value in config["pid"].items() if isinstance(value, list)},
        "limit_and_stall": config["limit_and_stall"],
        "hight_motor": config["hight_motor"],
        "yaw_motor": config["yaw_motor"],
    }
    for section, values in section_fields.items():
        start, end = section_bounds(lines, section)
        for key, value in values.items():
            formatted = None
            if section in ("hight_motor", "yaw_motor") and key in ("init_can_id", "send_can_id"):
                formatted = f"0x{int(value):X}"
            if section == "yaw_motor" and key == "send_ids":
                formatted = "[" + ", ".join(f"0x{int(item):X}" for item in value) + "]"
            replace_value(lines, start, end, key, value, formatted or format_gimbal_yaml_value(value))

    start, end = section_bounds(lines, "yaw")
    for key, value in config["yaw"]["variable_speed"].items():
        replace_value(lines, start, end, key, value, format_gimbal_yaml_value(value))

    updated = "".join(lines)
    parse_gimbal_config(updated)
    return updated


class EditorHandler(BaseHTTPRequestHandler):
    def _send(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, value) -> None:
        body = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self._send(status, "application/json; charset=utf-8", body)

    def do_GET(self) -> None:
        route = urlparse(self.path).path
        if route == "/":
            self._send(200, "text/html; charset=utf-8", HOME_PAGE_PATH.read_bytes())
            return
        if route == "/chassis":
            self._send(200, "text/html; charset=utf-8", PAGE_PATH.read_bytes())
            return
        if route == "/gimbal":
            self._send(200, "text/html; charset=utf-8", GIMBAL_PAGE_PATH.read_bytes())
            return
        if route == "/gears":
            self._send(200, "text/html; charset=utf-8", GEAR_PAGE_PATH.read_bytes())
            return
        if route == "/api/config":
            try:
                raw = CONFIG_PATH.read_text(encoding="utf-8")
                self._json(200, {"config": parse_config(raw)})
            except (OSError, ValueError, yaml.YAMLError) as exc:
                self._json(500, {"error": str(exc)})
            return
        if route == "/api/gimbal/config":
            try:
                raw = GIMBAL_CONFIG_PATH.read_text(encoding="utf-8")
                self._json(200, {"config": parse_gimbal_config(raw)})
            except (OSError, ValueError, yaml.YAMLError) as exc:
                self._json(500, {"error": str(exc)})
            return
        self._json(404, {"error": "Not found"})

    def do_POST(self) -> None:
        if self.headers.get("Origin") != f"http://{HOST}:{PORT}":
            self._json(403, {"error": "Requests must come from the local editor page"})
            return
        route = urlparse(self.path).path
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length > 100_000:
                self._json(413, {"error": "Request is too large"})
                return
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if route == "/api/export":
                source = CONFIG_PATH.read_text(encoding="utf-8")
                output = update_yaml(source, payload["config"])
                self._send(200, "application/yaml; charset=utf-8", output.encode("utf-8"))
                return
            if route == "/api/gimbal/export":
                source = GIMBAL_CONFIG_PATH.read_text(encoding="utf-8")
                output = update_gimbal_yaml(source, payload["config"])
                self._send(200, "application/yaml; charset=utf-8", output.encode("utf-8"))
                return
            if route == "/api/save":
                source = CONFIG_PATH.read_text(encoding="utf-8")
                output = update_yaml(source, payload["config"])
                temp_path = CONFIG_PATH.with_suffix(".yaml.tmp")
                with temp_path.open("w", encoding="utf-8", newline="\n") as config_file:
                    config_file.write(output)
                temp_path.replace(CONFIG_PATH)
                self._json(200, {"message": "配置已保存到 config/chassis.yaml"})
                return
            if route == "/api/gimbal/save":
                source = GIMBAL_CONFIG_PATH.read_text(encoding="utf-8")
                output = update_gimbal_yaml(source, payload["config"])
                temp_path = GIMBAL_CONFIG_PATH.with_suffix(".yaml.tmp")
                with temp_path.open("w", encoding="utf-8", newline="\n") as config_file:
                    config_file.write(output)
                temp_path.replace(GIMBAL_CONFIG_PATH)
                self._json(200, {"message": "配置已保存到 config/gimbal.yaml"})
                return
            self._json(404, {"error": "Not found"})
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError, json.JSONDecodeError) as exc:
            self._json(400, {"error": str(exc)})

    def log_message(self, fmt: str, *args) -> None:
        print("[config-editor] " + fmt % args)


def main() -> int:
    try:
        server = ThreadingHTTPServer((HOST, PORT), EditorHandler)
    except OSError as exc:
        print(f"Could not start editor at http://{HOST}:{PORT}: {exc}", file=sys.stderr)
        return 1

    print(f"Offline config editor: http://{HOST}:{PORT}")
    print("首页：/    底盘配置：/chassis    云台配置：/gimbal")
    print("Press Ctrl+C in this terminal to stop the editor.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping chassis editor.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
