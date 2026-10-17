#!/usr/bin/env python3
"""Generate User/Module/Gimbal/GimbalConfig.hpp from config/gimbal.yaml."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

try:
    import yaml
except ImportError as exc:
    raise SystemExit(
        "PyYAML is required. Install it with: "
        "python -m pip install -r tools/requirements.txt"
    ) from exc


def number(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def integer(value, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def sequence(value, length: int, label: str) -> list:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label} must contain exactly {length} items")
    return value


def cpp_float(value, label: str) -> str:
    result = f"{number(value, label):.12g}"
    if "." not in result and "e" not in result.lower():
        result += ".0"
    return result + "f"


def cpp_double(value, label: str) -> str:
    return f"{number(value, label):.15g}"


def render(config: dict) -> str:
    if not isinstance(config, dict):
        raise ValueError("The YAML root must be a mapping")

    yaw = config["yaw"]
    variable = yaw["variable_speed"]
    hight = config["hight"]
    pid = config["pid"]
    limit = config["limit_and_stall"]
    hight_motor = config["hight_motor"]
    yaw_motor = config["yaw_motor"]
    docs = config["documentation"]

    def description(key: str) -> str:
        value = docs[key]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"documentation.{key} must be a non-empty string")
        return value.strip()

    lines = [
        "#pragma once", "", "#include <cstdint>", "",
        "/**", " * @file GimbalConfig.hpp",
        " * @brief GENERATED FILE. Edit config/gimbal.yaml, then regenerate this file.",
        " * @details Rebuild and flash firmware after changing the YAML configuration.",
        " */", "namespace GimbalConfig", "{",
        "/** @brief PID parameters: Kp, Ki, Kd, output limit, integral limit and separation threshold. */",
        "struct PIDParameters { float kp; float ki; float kd; float output_limit; float integral_limit; float integral_separation_threshold; };",
        "/** @brief DM motor MIT position, velocity, torque and gain ranges. */",
        "struct DMMotorParameters { float position_min_rad; float position_max_rad; float velocity_min_radps; float velocity_max_radps; float torque_min_nm; float torque_max_nm; float kp_min; float kp_max; float kd_min; float kd_max; };",
        "/** @brief DJI motor feedback and current unit conversion parameters. */",
        "struct DJIMotorParameters { double reduction_ratio; double torque_constant_nm_per_a; double feedback_current_max; double motor_current_max_a; double encoder_resolution; };",
        "",
        "// ===== Yaw axis speed planning =====",
    ]

    def emit_float(name: str, value, doc_key: str, label: str):
        lines.append(
            f"inline constexpr float {name} = {cpp_float(value, label)}; "
            f"///< {description(doc_key)}"
        )

    def emit_integer(cpp_type: str, name: str, value, doc_key: str, label: str):
        lines.append(
            f"inline constexpr {cpp_type} {name} = {integer(value, label)}; "
            f"///< {description(doc_key)}"
        )

    def emit_float_array(name: str, values, count: int, doc_key: str, label: str):
        items = [cpp_float(v, label) for v in sequence(values, count, label)]
        lines.append(
            f"inline constexpr float {name}[{count}] = {{{', '.join(items)}}}; "
            f"///< {description(doc_key)}"
        )

    def emit_uint_array(cpp_type: str, name: str, values, count: int, doc_key: str, label: str):
        items = [str(integer(v, label)) for v in sequence(values, count, label)]
        lines.append(
            f"inline constexpr {cpp_type} {name}[{count}] = {{{', '.join(items)}}}; "
            f"///< {description(doc_key)}"
        )

    emit_float("YAW_TARGET_LIMIT_RADPS", yaw["target_limit_radps"], "yaw_target_limit_radps", "yaw.target_limit_radps")
    emit_float("YAW_CONSTANT_SPEED_STEP_RADPS", yaw["constant_speed_step_radps"], "yaw_constant_speed_step_radps", "yaw.constant_speed_step_radps")
    emit_float("YAW_CONSTANT_SPEED_LIMIT_RADPS", yaw["constant_speed_limit_radps"], "yaw_constant_speed_limit_radps", "yaw.constant_speed_limit_radps")
    emit_float("YAW_SCROLL_THRESHOLD", yaw["scroll_threshold"], "yaw_scroll_threshold", "yaw.scroll_threshold")
    emit_float("YAW_VARIABLE_BASE_SPEED_RADPS", variable["base_radps"], "yaw_variable_base_speed_radps", "yaw.variable_speed.base_radps")
    emit_float("YAW_VARIABLE_AMPLITUDE_1_RADPS", variable["amplitude_1_radps"], "yaw_variable_amplitude_1_radps", "yaw.variable_speed.amplitude_1_radps")
    emit_float("YAW_VARIABLE_AMPLITUDE_2_RADPS", variable["amplitude_2_radps"], "yaw_variable_amplitude_2_radps", "yaw.variable_speed.amplitude_2_radps")
    emit_float("YAW_VARIABLE_FREQUENCY_1_HZ", variable["frequency_1_hz"], "yaw_variable_frequency_1_hz", "yaw.variable_speed.frequency_1_hz")
    emit_float("YAW_VARIABLE_FREQUENCY_2_HZ", variable["frequency_2_hz"], "yaw_variable_frequency_2_hz", "yaw.variable_speed.frequency_2_hz")
    emit_float("YAW_VARIABLE_PHASE_1_RAD", variable["phase_1_rad"], "yaw_variable_phase_1_rad", "yaw.variable_speed.phase_1_rad")
    emit_float("YAW_VARIABLE_PHASE_2_RAD", variable["phase_2_rad"], "yaw_variable_phase_2_rad", "yaw.variable_speed.phase_2_rad")

    lines += ["", "// ===== Hight mechanism speed planning ====="]
    hight_names = [
        ("HIGHT_TRAVEL_DEG", "travel_deg", "hight_travel_deg"),
        ("HIGHT_UP_SPEED_RADPS", "up_speed_radps", "hight_up_speed_radps"),
        ("HIGHT_DOWN_SPEED_RADPS", "down_speed_radps", "hight_down_speed_radps"),
        ("HIGHT_HOME_SPEED_RADPS", "home_speed_radps", "hight_home_speed_radps"),
        ("HIGHT_ACCELERATION_RADPS2", "acceleration_radps2", "hight_acceleration_radps2"),
        ("HIGHT_MAX_SPEED_RADPS", "max_speed_radps", "hight_max_speed_radps"),
        ("HIGHT_SLOWDOWN_START_RATIO", "slowdown_start_ratio", "hight_slowdown_start_ratio"),
        ("HIGHT_LIMIT_SPEED_RADPS", "limit_speed_radps", "hight_limit_speed_radps"),
        ("HIGHT_MIN_CONTROL_DT_S", "min_control_dt_s", "hight_min_control_dt_s"),
        ("HIGHT_MAX_CONTROL_DT_S", "max_control_dt_s", "hight_max_control_dt_s"),
    ]
    for cpp_name, yaml_name, doc_key in hight_names:
        emit_float(cpp_name, hight[yaml_name], doc_key, f"hight.{yaml_name}")

    lines += ["", "// ===== PID and current limits ====="]
    emit_float("HIGHT_CURRENT_MAX", pid["hight_current_max"], "hight_current_max", "pid.hight_current_max")
    emit_float("HIGHT_GRAVITY_FEEDFORWARD", pid["hight_gravity_feedforward"], "hight_gravity_feedforward", "pid.hight_gravity_feedforward")
    for axis, doc_key in (("yaw", "yaw_pid"), ("hight", "hight_pid")):
        values = [cpp_float(v, f"pid.{axis}") for v in sequence(pid[axis], 6, f"pid.{axis}")]
        lines.append(
            f"inline constexpr PIDParameters {('YAW_PID' if axis == 'yaw' else 'HIGHT_PID')} = "
            f"{{{', '.join(values)}}}; ///< {description(doc_key)}"
        )

    lines += ["", "// ===== Limit switches and stall detection ====="]
    if (not isinstance(limit["hight_lower_limit_active_low"], bool) or
            not isinstance(limit["hight_upper_limit_active_low"], bool) or
            not isinstance(limit["stall_detection_enabled"], bool)):
        raise ValueError("limit_and_stall limit polarity and stall detection settings must be booleans")
    lines.append(f"inline constexpr bool HIGHT_LOWER_LIMIT_ACTIVE_LOW = {'true' if limit['hight_lower_limit_active_low'] else 'false'}; ///< {description('hight_lower_limit_active_low')}")
    lines.append(f"inline constexpr bool HIGHT_UPPER_LIMIT_ACTIVE_LOW = {'true' if limit['hight_upper_limit_active_low'] else 'false'}; ///< {description('hight_upper_limit_active_low')}")
    lines.append(f"inline constexpr bool HIGHT_STALL_DETECTION_ENABLED = {'true' if limit['stall_detection_enabled'] else 'false'}; ///< {description('hight_stall_detection_enabled')}")
    emit_float("HIGHT_STALL_MIN_TARGET_RADPS", limit["stall_min_target_radps"], "hight_stall_min_target_radps", "limit_and_stall.stall_min_target_radps")
    emit_float("HIGHT_STALL_MAX_ACTUAL_RADPS", limit["stall_max_actual_radps"], "hight_stall_max_actual_radps", "limit_and_stall.stall_max_actual_radps")
    emit_float("HIGHT_STALL_MIN_CURRENT", limit["stall_min_current"], "hight_stall_min_current", "limit_and_stall.stall_min_current")
    emit_integer("std::uint32_t", "HIGHT_STALL_DURATION_MS", limit["stall_duration_ms"], "hight_stall_duration_ms", "limit_and_stall.stall_duration_ms")

    lines += ["", "// ===== Hight GM3508 motor and CAN configuration ====="]
    hight_count = integer(hight_motor["count"], "hight_motor.count")
    emit_integer("std::uint8_t", "HIGHT_MOTOR_COUNT", hight_count, "hight_motor_count", "hight_motor.count")
    emit_integer("std::uint16_t", "HIGHT_MOTOR_INIT_CAN_ID", hight_motor["init_can_id"], "hight_motor_init_can_id", "hight_motor.init_can_id")
    emit_uint_array("std::uint8_t", "HIGHT_MOTOR_FEEDBACK_IDS", hight_motor["feedback_ids"], hight_count, "hight_motor_feedback_ids", "hight_motor.feedback_ids")
    emit_integer("std::uint32_t", "HIGHT_MOTOR_SEND_CAN_ID", hight_motor["send_can_id"], "hight_motor_send_can_id", "hight_motor.send_can_id")
    emit_integer("std::uint8_t", "HIGHT_MOTOR_CHANNEL", hight_motor["channel"], "hight_motor_channel", "hight_motor.channel")
    emit_integer("std::uint8_t", "HIGHT_MOTOR_RING_ID", hight_motor["ring_id"], "hight_motor_ring_id", "hight_motor.ring_id")
    dji_params = [
        cpp_double(hight_motor[key], f"hight_motor.{key}")
        for key in ("reduction_ratio", "torque_constant_nm_per_a", "feedback_current_max", "motor_current_max_a", "encoder_resolution")
    ]
    lines.append(f"inline constexpr DJIMotorParameters HIGHT_MOTOR_PARAMETERS = {{{', '.join(dji_params)}}}; ///< {description('hight_motor_parameters')}")

    lines += ["", "// ===== Yaw J4310 motor and CAN configuration ====="]
    yaw_count = integer(yaw_motor["count"], "yaw_motor.count")
    emit_integer("std::uint8_t", "YAW_MOTOR_COUNT", yaw_count, "yaw_motor_count", "yaw_motor.count")
    emit_integer("std::uint16_t", "YAW_MOTOR_INIT_CAN_ID", yaw_motor["init_can_id"], "yaw_motor_init_can_id", "yaw_motor.init_can_id")
    emit_uint_array("std::uint8_t", "YAW_MOTOR_FEEDBACK_IDS", yaw_motor["feedback_ids"], yaw_count, "yaw_motor_feedback_ids", "yaw_motor.feedback_ids")
    emit_uint_array("std::uint32_t", "YAW_MOTOR_SEND_IDS", yaw_motor["send_ids"], yaw_count, "yaw_motor_send_ids", "yaw_motor.send_ids")
    emit_integer("std::uint8_t", "YAW_MOTOR_CHANNEL", yaw_motor["channel"], "yaw_motor_channel", "yaw_motor.channel")
    dm_params = [cpp_float(v, "yaw_motor.mit_parameters") for v in sequence(yaw_motor["mit_parameters"], 10, "yaw_motor.mit_parameters")]
    lines.append(f"inline constexpr DMMotorParameters YAW_MOTOR_PARAMETERS = {{{', '.join(dm_params)}}}; ///< {description('yaw_motor_parameters')}")
    emit_integer("std::uint32_t", "YAW_MOTOR_COMMAND_INTERVAL_MS", yaw_motor["command_interval_ms"], "yaw_motor_command_interval_ms", "yaw_motor.command_interval_ms")
    emit_integer("std::uint32_t", "YAW_MOTOR_FEEDBACK_TIMEOUT_MS", yaw_motor["feedback_timeout_ms"], "yaw_motor_feedback_timeout_ms", "yaw_motor.feedback_timeout_ms")
    lines += ["} // namespace GimbalConfig", ""]
    return "\n".join(lines)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=root / "config/gimbal.yaml")
    parser.add_argument("--output", type=Path, default=root / "User/Module/Gimbal/GimbalConfig.hpp")
    args = parser.parse_args()

    try:
        config = yaml.safe_load(args.input.read_text(encoding="utf-8"))
        generated = render(config)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"Gimbal config generation failed: {exc}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="\n") as output_file:
        output_file.write(generated)
    print(f"Generated {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
