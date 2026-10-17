#!/usr/bin/env python3
"""Generate the firmware's ChassisConfig.hpp from config/chassis.yaml."""

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


def require_number(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def require_int(value, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def require_list(value, length: int, label: str) -> list:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label} must contain exactly {length} items")
    return value


def cpp_float(value, label: str) -> str:
    rendered = f"{require_number(value, label):.10g}"
    if "." not in rendered and "e" not in rendered.lower():
        rendered += ".0"
    return f"{rendered}f"


def cpp_double(value, label: str) -> str:
    return f"{require_number(value, label):.15g}"


def render(config: dict) -> str:
    if not isinstance(config, dict):
        raise ValueError("The YAML root must be a mapping")

    wheel_count = require_int(config["wheel_count"], "wheel_count")
    # Current OmniCalculation implementation has fixed four-wheel equations.
    if wheel_count != 4:
        raise ValueError("wheel_count must be 4 until OmniCalculation supports other layouts")

    geometry = config["geometry"]
    layout = config["wheel_layout"]
    ramp = config["translation_ramp"]
    remote = config["remote"]
    patrol = config["patrol"]
    motor = config["motor"]
    descriptions = config["documentation"]

    def description(key: str) -> str:
        text = descriptions[key]
        if not isinstance(text, str) or not text.strip():
            raise ValueError(f"documentation.{key} must be a non-empty string")
        return text.strip()

    azimuth = [cpp_float(v, "wheel_layout.azimuth_rad") for v in require_list(layout["azimuth_rad"], wheel_count, "wheel_layout.azimuth_rad")]
    direction = [cpp_float(v, "wheel_layout.direction_rad") for v in require_list(layout["direction_rad"], wheel_count, "wheel_layout.direction_rad")]
    motor_ids = [str(require_int(v, "motor.receive_ids")) for v in require_list(motor["receive_ids"], wheel_count, "motor.receive_ids")]
    pid_rows = require_list(config["wheel_pid"], wheel_count, "wheel_pid")
    pids = []
    for index, row in enumerate(pid_rows):
        row = require_list(row, 6, f"wheel_pid[{index}]")
        pids.append("    {" + ", ".join(cpp_float(v, f"wheel_pid[{index}]") for v in row) + "}")
    output_limits = [require_number(row[3], f"wheel_pid[{index}].output_limit") for index, row in enumerate(pid_rows)]
    if any(limit != output_limits[0] for limit in output_limits[1:]):
        raise ValueError("all wheel_pid output limits must match the shared MOTOR_CURRENT_LIMIT")

    lines = [
        "#pragma once",
        "",
        "#include <cstdint>",
        "",
        "/**",
        " * @file ChassisConfig.hpp",
        " * @brief GENERATED FILE. Edit config/chassis.yaml, then regenerate this file.",
        " * @details Rebuild and flash firmware after changing the YAML configuration.",
        " */",
        "namespace ChassisConfig",
        "{",
        "// ===== Wheel count and geometry (FK and IK must use matching values) =====",
        f"inline constexpr std::uint8_t WHEEL_COUNT = {wheel_count}; ///< {description('wheel_count')}",
        'static_assert(WHEEL_COUNT == 4, "Current OmniCalculation implementation requires exactly four wheels.");',
        f"inline constexpr float CHASSIS_RADIUS_M = {cpp_float(geometry['chassis_radius_m'], 'geometry.chassis_radius_m')}; ///< {description('chassis_radius_m')}",
        f"inline constexpr float WHEEL_RADIUS_M = {cpp_float(geometry['wheel_radius_m'], 'geometry.wheel_radius_m')}; ///< {description('wheel_radius_m')}",
        "",
        f"inline constexpr float WHEEL_AZIMUTH_RAD[WHEEL_COUNT] = {{{', '.join(azimuth)}}}; ///< {description('wheel_azimuth_rad')}",
        f"inline constexpr float WHEEL_DIRECTION_RAD[WHEEL_COUNT] = {{{', '.join(direction)}}}; ///< {description('wheel_direction_rad')}",
        "",
        f"/** @brief {description('wheel_pid')} */",
        "struct WheelPid { float kp; float ki; float kd; float output_limit; float integral_limit; float integral_separation_threshold; };",
        f"inline constexpr float MOTOR_CURRENT_LIMIT = {cpp_float(pid_rows[0][3], 'wheel_pid[0].output_limit')}; ///< {description('motor_current_limit')}",
        "inline constexpr WheelPid WHEEL_PID[WHEEL_COUNT] = {",
        ",\n".join(pids),
        "};",
        "",
        f"inline constexpr float TRANSLATION_RAMP_INCREASE = {cpp_float(ramp['increase_per_cycle'], 'translation_ramp.increase_per_cycle')}; ///< {description('translation_ramp_increase')}",
        f"inline constexpr float TRANSLATION_RAMP_DECREASE = {cpp_float(ramp['decrease_per_cycle'], 'translation_ramp.decrease_per_cycle')}; ///< {description('translation_ramp_decrease')}",
        "",
        f"inline constexpr float CONTROL_PERIOD_S = {cpp_float(config['control_period_s'], 'control_period_s')}; ///< {description('control_period_s')}",
        f"inline constexpr float REMOTE_DEADZONE = {cpp_float(remote['deadzone'], 'remote.deadzone')}; ///< {description('remote_deadzone')}",
        f"inline constexpr float REMOTE_LINEAR_GAIN = {cpp_float(remote['linear_gain_mps'], 'remote.linear_gain_mps')}; ///< {description('remote_linear_gain_mps')}",
        f"inline constexpr float REMOTE_ROTATION_GAIN = {cpp_float(remote['rotation_gain_radps'], 'remote.rotation_gain_radps')}; ///< {description('remote_rotation_gain_radps')}",
        f"inline constexpr float PATROL_SPEED_MPS = {cpp_float(patrol['speed_mps'], 'patrol.speed_mps')}; ///< {description('patrol_speed_mps')}",
        f"inline constexpr float PATROL_STOP_LINEAR_MPS = {cpp_float(patrol['stop_linear_threshold_mps'], 'patrol.stop_linear_threshold_mps')}; ///< {description('patrol_stop_linear_threshold_mps')}",
        f"inline constexpr float PATROL_STOP_ANGULAR_RADPS = {cpp_float(patrol['stop_angular_threshold_radps'], 'patrol.stop_angular_threshold_radps')}; ///< {description('patrol_stop_angular_threshold_radps')}",
        f"inline constexpr unsigned PATROL_SETTLE_CYCLES = {require_int(patrol['settle_cycles'], 'patrol.settle_cycles')}; ///< {description('patrol_settle_cycles')}",
        f"inline constexpr float PATROL_DISTANCE_M = {cpp_float(patrol['distance_m'], 'patrol.distance_m')}; ///< {description('patrol_distance_m')}",
        f"inline constexpr float VEHICLE_TURN_RADIUS_M = {cpp_float(patrol['vehicle_turn_radius_m'], 'patrol.vehicle_turn_radius_m')}; ///< {description('vehicle_turn_radius_m')}",
        f"inline constexpr float MAX_WHEEL_SPEED_MPS = {cpp_float(patrol['max_wheel_speed_mps'], 'patrol.max_wheel_speed_mps')}; ///< {description('max_wheel_speed_mps')}",
        "",
        f"inline constexpr std::uint16_t MOTOR_INIT_CAN_ID = {require_int(motor['init_can_id'], 'motor.init_can_id')}; ///< {description('motor_init_can_id')}",
        f"inline constexpr std::uint8_t MOTOR_RECEIVE_IDS[WHEEL_COUNT] = {{{', '.join(motor_ids)}}}; ///< {description('motor_receive_ids')}",
        f"inline constexpr std::uint32_t MOTOR_SEND_CAN_ID = {require_int(motor['send_can_id'], 'motor.send_can_id')}; ///< {description('motor_send_can_id')}",
        "",
        f"inline constexpr double MOTOR_REDUCTION_RATIO = {cpp_double(motor['reduction_ratio'], 'motor.reduction_ratio')}; ///< {description('motor_reduction_ratio')}",
        f"inline constexpr double MOTOR_TORQUE_CONSTANT = {cpp_double(motor['torque_constant_nm_per_a'], 'motor.torque_constant_nm_per_a')}; ///< {description('motor_torque_constant_nm_per_a')}",
        f"inline constexpr double MOTOR_FEEDBACK_CURRENT_MAX = {cpp_double(motor['feedback_current_max'], 'motor.feedback_current_max')}; ///< {description('motor_feedback_current_max')}",
        f"inline constexpr double MOTOR_MODEL_CURRENT_MAX = {cpp_double(motor['model_current_max_a'], 'motor.model_current_max_a')}; ///< {description('motor_model_current_max_a')}",
        f"inline constexpr double MOTOR_ENCODER_RESOLUTION = {cpp_double(motor['encoder_resolution'], 'motor.encoder_resolution')}; ///< {description('motor_encoder_resolution')}",
        "} // namespace ChassisConfig",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=root / "config/chassis.yaml")
    parser.add_argument("--output", type=Path, default=root / "build/generated/ChassisConfig.hpp")
    args = parser.parse_args()

    try:
        config = yaml.safe_load(args.input.read_text(encoding="utf-8"))
        output = render(config)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"Chassis config generation failed: {exc}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="\n") as generated_file:
        generated_file.write(output)
    print(f"Generated {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
