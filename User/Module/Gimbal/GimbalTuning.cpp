#include "GimbalTuning.hpp"

#include "GimbalConfig.hpp"

namespace GimbalTuning
{
volatile float YAW_TARGET_LIMIT_RADPS = GimbalConfig::YAW_TARGET_LIMIT_RADPS;
volatile float YAW_CONSTANT_SPEED_STEP_RADPS = GimbalConfig::YAW_CONSTANT_SPEED_STEP_RADPS;
volatile float YAW_CONSTANT_SPEED_LIMIT_RADPS = GimbalConfig::YAW_CONSTANT_SPEED_LIMIT_RADPS;
volatile float YAW_SCROLL_THRESHOLD = GimbalConfig::YAW_SCROLL_THRESHOLD;
volatile float YAW_VARIABLE_BASE_SPEED_RADPS = GimbalConfig::YAW_VARIABLE_BASE_SPEED_RADPS;
volatile float YAW_VARIABLE_AMPLITUDE_1_RADPS = GimbalConfig::YAW_VARIABLE_AMPLITUDE_1_RADPS;
volatile float YAW_VARIABLE_AMPLITUDE_2_RADPS = GimbalConfig::YAW_VARIABLE_AMPLITUDE_2_RADPS;
volatile float YAW_VARIABLE_FREQUENCY_1_HZ = GimbalConfig::YAW_VARIABLE_FREQUENCY_1_HZ;
volatile float YAW_VARIABLE_FREQUENCY_2_HZ = GimbalConfig::YAW_VARIABLE_FREQUENCY_2_HZ;
volatile float YAW_VARIABLE_PHASE_1_RAD = GimbalConfig::YAW_VARIABLE_PHASE_1_RAD;
volatile float YAW_VARIABLE_PHASE_2_RAD = GimbalConfig::YAW_VARIABLE_PHASE_2_RAD;

volatile float HIGHT_TRAVEL_DEG = GimbalConfig::HIGHT_TRAVEL_DEG;
volatile float HIGHT_UP_SPEED_RADPS = GimbalConfig::HIGHT_UP_SPEED_RADPS;
volatile float HIGHT_DOWN_SPEED_RADPS = GimbalConfig::HIGHT_DOWN_SPEED_RADPS;
volatile float HIGHT_HOME_SPEED_RADPS = GimbalConfig::HIGHT_HOME_SPEED_RADPS;
volatile float HIGHT_ACCELERATION_RADPS2 = GimbalConfig::HIGHT_ACCELERATION_RADPS2;
volatile float HIGHT_MAX_SPEED_RADPS = GimbalConfig::HIGHT_MAX_SPEED_RADPS;
volatile float HIGHT_SLOWDOWN_START_RATIO = GimbalConfig::HIGHT_SLOWDOWN_START_RATIO;
volatile float HIGHT_LIMIT_SPEED_RADPS = GimbalConfig::HIGHT_LIMIT_SPEED_RADPS;
volatile float HIGHT_MIN_CONTROL_DT_S = GimbalConfig::HIGHT_MIN_CONTROL_DT_S;
volatile float HIGHT_MAX_CONTROL_DT_S = GimbalConfig::HIGHT_MAX_CONTROL_DT_S;

volatile float HIGHT_CURRENT_MAX = GimbalConfig::HIGHT_CURRENT_MAX;
volatile float HIGHT_GRAVITY_FEEDFORWARD = GimbalConfig::HIGHT_GRAVITY_FEEDFORWARD;

volatile float YAW_PID_KP = GimbalConfig::YAW_PID.kp;
volatile float YAW_PID_KI = GimbalConfig::YAW_PID.ki;
volatile float YAW_PID_KD = GimbalConfig::YAW_PID.kd;
volatile float YAW_PID_OUTPUT_LIMIT = GimbalConfig::YAW_PID.output_limit;
volatile float YAW_PID_INTEGRAL_LIMIT = GimbalConfig::YAW_PID.integral_limit;
volatile float YAW_PID_INTEGRAL_SEPARATION = GimbalConfig::YAW_PID.integral_separation_threshold;
volatile float HIGHT_PID_KP = GimbalConfig::HIGHT_PID.kp;
volatile float HIGHT_PID_KI = GimbalConfig::HIGHT_PID.ki;
volatile float HIGHT_PID_KD = GimbalConfig::HIGHT_PID.kd;
volatile float HIGHT_PID_OUTPUT_LIMIT = GimbalConfig::HIGHT_PID.output_limit;
volatile float HIGHT_PID_INTEGRAL_LIMIT = GimbalConfig::HIGHT_PID.integral_limit;
volatile float HIGHT_PID_INTEGRAL_SEPARATION = GimbalConfig::HIGHT_PID.integral_separation_threshold;

} // namespace GimbalTuning
