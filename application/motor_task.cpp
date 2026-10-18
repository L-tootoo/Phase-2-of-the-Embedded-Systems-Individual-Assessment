#include <cmath>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"

// 使用 uart_task.cpp 中已经定义的遥控器对象。
extern sp::DBus remote;

namespace
{

enum MotorIndex
{
  FRONT_LEFT = 0,   // ID 1：左前
  REAR_LEFT = 1,    // ID 2：左后
  REAR_RIGHT = 2,   // ID 3：右后
  FRONT_RIGHT = 3,  // ID 4：右前
  MOTOR_COUNT = 4
};

constexpr float REDUCTION_RATIO = 13.7f;
constexpr float WHEEL_RADIUS = 0.077f;  // 轮半径，m
constexpr float MAX_SPEED = 0.5f;       // 最大平移速度，m/s，初期低速调试
constexpr float STICK_DEADZONE = 0.05f;

// 当前 can_task 使用 osDelay(2)，这里暂按名义周期 2 ms 设置。
constexpr float CONTROL_DT = 0.002f;

// 待整定：先调 P，再逐步加入 I，初期 D 可保持为零。
constexpr float SPEED_KP = 0.8f;
constexpr float SPEED_KI = 0.1f;
constexpr float SPEED_KD = 0.0f;

// 待配置：PID 输出是输出轴力矩，单位 N·m。
// 设为零时不会产生驱动力矩。
constexpr float MAX_TORQUE = 1.0f;
constexpr float MAX_I_TORQUE = 0.8f;

// 轮组纵向间距 330 mm，横向间距 370 mm。
// 旋转解算系数：半轴距 + 半轮距，单位 m。
constexpr float ROTATION_LEVER = (0.330f + 0.370f) / 2.0f;

// 底盘目标旋转角速度，单位 rad/s。
constexpr float ROTATION_SPEED = 1.0f;

// 拨轮中心死区。
constexpr float DIAL_DEADZONE = 0.05f;

sp::RM_Motor motors[MOTOR_COUNT] = {
  {1, sp::RM_Motors::M3508, REDUCTION_RATIO},
  {2, sp::RM_Motors::M3508, REDUCTION_RATIO},
  {3, sp::RM_Motors::M3508, REDUCTION_RATIO},
  {4, sp::RM_Motors::M3508, REDUCTION_RATIO}};

// 待实测确认，并非已确定的安装方向。
// 电机正转驱动车体向前：填 +1；驱动车体向后：填 -1。
constexpr float MOTOR_DIRECTION[MOTOR_COUNT] = {
  -1.0f,  // ID 1：左前
  +1.0f,  // ID 2：左后
  +1.0f,  // ID 3：右后
  -1.0f   // ID 4：右前
};

// 每个大括号创建一个独立 PID。
// 默认使用同一组参数，也可以把某组参数换成数值，单独调整该电机。
sp::PID speed_pid[MOTOR_COUNT] = {
  {
    // speed_pid[0]：ID 1，左前
    CONTROL_DT,    // dt：控制周期，s
    SPEED_KP,      // kp：比例系数
    SPEED_KI,      // ki：积分系数
    SPEED_KD,      // kd：微分系数
    MAX_TORQUE,    // max_out：最大输出力矩，N·m
    MAX_I_TORQUE,  // max_iout：最大积分输出，N·m
    1.0f,          // alpha：微分滤波系数，1 表示不滤波
    false,         // angular：速度环，不启用角度环绕处理
    false          // dynamic：不启用动态积分
  },
  {
    // speed_pid[1]：ID 2，左后
    CONTROL_DT,    // dt：控制周期，s
    SPEED_KP,      // kp：比例系数
    SPEED_KI,      // ki：积分系数
    SPEED_KD,      // kd：微分系数
    MAX_TORQUE,    // max_out：最大输出力矩，N·m
    MAX_I_TORQUE,  // max_iout：最大积分输出，N·m
    1.0f,          // alpha：微分滤波系数
    false,         // angular：不启用角度环绕处理
    false          // dynamic：不启用动态积分
  },
  {
    // speed_pid[2]：ID 3，右后
    CONTROL_DT,    // dt：控制周期，s
    SPEED_KP,      // kp：比例系数
    SPEED_KI,      // ki：积分系数
    SPEED_KD,      // kd：微分系数
    MAX_TORQUE,    // max_out：最大输出力矩，N·m
    MAX_I_TORQUE,  // max_iout：最大积分输出，N·m
    1.0f,          // alpha：微分滤波系数
    false,         // angular：不启用角度环绕处理
    false          // dynamic：不启用动态积分
  },
  {
    // speed_pid[3]：ID 4，右前
    CONTROL_DT,    // dt：控制周期，s
    SPEED_KP,      // kp：比例系数
    SPEED_KI,      // ki：积分系数
    SPEED_KD,      // kd：微分系数
    MAX_TORQUE,    // max_out：最大输出力矩，N·m
    MAX_I_TORQUE,  // max_iout：最大积分输出，N·m
    1.0f,          // alpha：微分滤波系数
    false,         // angular：不启用角度环绕处理
    false          // dynamic：不启用动态积分
  }};
constexpr float k1 = 3.066786f;
constexpr float k2 = 0.01515196f;
constexpr float a = 3.996536f;

// 输入：四轮力矩指令 N·m，四轮实际角速度 rad/s。
// 返回：四轮预测总输入功率 W。
float calculate_power(const float torque[MOTOR_COUNT], const float speed[MOTOR_COUNT])
{
  float mechanical_power = 0.0f;
  float torque_squared_sum = 0.0f;
  float speed_squared_sum = 0.0f;

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    mechanical_power += torque[i] * speed[i];
    torque_squared_sum += torque[i] * torque[i];
    speed_squared_sum += speed[i] * speed[i];
  }

  return mechanical_power + k1 * torque_squared_sum + k2 * speed_squared_sum + a;
}
// 死区内输出零；死区外连续映射到 [-1, 1]。
float apply_deadzone(float value)
{
  if (value > 1.0f) value = 1.0f;
  if (value < -1.0f) value = -1.0f;

  if (std::abs(value) <= STICK_DEADZONE) {
    return 0.0f;
  }

  if (value > 0.0f) {
    return (value - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
  }

  return (value + STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
}
// ---------- 自动前进的位置外环 ----------

constexpr float AUTO_DISTANCE = 3.0f;        // 目标距离，m
constexpr float AUTO_MAX_SPEED = 1.2f;       // 自动移动最大速度，m/s
constexpr float AUTO_DISTANCE_TOL = 0.02f;   // 到达距离容差，m
constexpr float AUTO_STOP_SPEED = 0.03f;     // 判定停稳的轮缘速度，m/s
constexpr uint32_t AUTO_SETTLE_MS = 300;     // 连续满足到达条件的时间
constexpr uint32_t AUTO_TIMEOUT_MS = 30000;  // 自动任务超时，ms

// 输出是底盘速度 m/s，不是力矩！
// 第一版先使用 P 控制，参数需要实测调整。
sp::PID position_pid(
  CONTROL_DT,      // dt：控制周期，s
  1.2f,            // kp：位置误差 m → 速度 m/s
  0.0f,            // ki：初期不加积分
  0.0f,            // kd：初期不加微分
  AUTO_MAX_SPEED,  // max_out：最大前进/后退速度，m/s
  0.0f,            // max_iout：积分输出限幅
  1.0f,            // alpha
  false,           // angular：距离不做角度环绕
  false            // dynamic
);

enum class AutoState
{
  IDLE,
  RUNNING,
  DONE,
  FAULT
};

AutoState auto_state = AutoState::IDLE;
bool auto_armed = false;

float auto_start_angle[MOTOR_COUNT] = {};
float auto_distance = 0.0f;

uint32_t auto_start_ms = 0;
uint32_t auto_settle_start_ms = 0;
bool auto_settling = false;

// 取消自动任务，且必须重新准备后才能启动。
void auto_reset()
{
  auto_state = AutoState::IDLE;
  auto_armed = false;
  auto_settling = false;
  auto_distance = 0.0f;
  position_pid.clear();
}

// 返回 false：本周期必须零力矩输出。
// 返回 true：继续使用下面的四轮速度控制。
// 手动模式下不修改 vx、vy、wz。
bool auto_update(float & vx, float & vy, float & wz)
{
  const uint32_t now_ms = osKernelSysTick();
  const auto left_switch = remote.sw_l;

  if (left_switch != sp::DBusSwitchMode::UP) {
    // 离开正在运行、已完成或故障的自动模式：
    // 清除状态，下一次必须重新经过左下档准备。
    if (auto_state != AutoState::IDLE) {
      position_pid.clear();
      auto_armed = false;

      for (int i = 0; i < MOTOR_COUNT; ++i) {
        speed_pid[i].clear();
      }
    }

    auto_state = AutoState::IDLE;
    auto_settling = false;

    // 左下档置位；经过中档时保留准备标志。
    if (left_switch == sp::DBusSwitchMode::DOWN) {
      auto_armed = true;
    }

    return true;
  }

  // 左上档属于自动模式，不接受摇杆或拨轮的运动指令。
  vx = 0.0f;
  vy = 0.0f;
  wz = 0.0f;

  // 上电时就处于左上档，不能直接起跑。
  if (auto_state == AutoState::IDLE && !auto_armed) {
    return false;
  }

  // 读取多圈角度及轮速快照。
  float angle[MOTOR_COUNT];
  float wheel_speed[MOTOR_COUNT];
  bool feedback_ok = true;

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    angle[i] = motors[i].angle;
    wheel_speed[i] = motors[i].speed;

    if (!motors[i].is_alive(now_ms)) {
      feedback_ok = false;
    }
  }

  // 自动测距不能使用过期的电机反馈。
  if (!feedback_ok) {
    auto_state = AutoState::FAULT;
    auto_armed = false;
    position_pid.clear();
    return false;
  }

  if (auto_state == AutoState::FAULT) {
    return false;
  }

  // 第一次进入自动模式：记录当前位置为起点。
  if (auto_state == AutoState::IDLE) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      auto_start_angle[i] = angle[i];
      speed_pid[i].clear();
    }

    position_pid.clear();
    auto_distance = 0.0f;
    auto_start_ms = now_ms;
    auto_settling = false;
    auto_armed = false;
    auto_state = AutoState::RUNNING;
  }

  // 四轮向前行驶距离的平均值。
  // motors.angle 已经除过减速比，不能再次除以 13.7。
  auto_distance = 0.0f;
  float max_wheel_speed = 0.0f;

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    auto_distance +=
      MOTOR_DIRECTION[i] * (angle[i] - auto_start_angle[i]) * WHEEL_RADIUS / MOTOR_COUNT;

    const float speed_abs = std::abs(wheel_speed[i] * WHEEL_RADIUS);

    if (speed_abs > max_wheel_speed) {
      max_wheel_speed = speed_abs;
    }
  }

  // 完成后维持零速闭环，不重复启动。
  if (auto_state == AutoState::DONE) {
    return true;
  }

  if (now_ms - auto_start_ms >= AUTO_TIMEOUT_MS) {
    auto_state = AutoState::FAULT;
    position_pid.clear();
    return false;
  }

  const float error = AUTO_DISTANCE - auto_distance;

  if (std::abs(error) <= AUTO_DISTANCE_TOL) {
    // 进入距离容差后，先让速度环制动至零速。
    vx = 0.0f;
    position_pid.clear();

    if (max_wheel_speed <= AUTO_STOP_SPEED) {
      if (!auto_settling) {
        auto_settling = true;
        auto_settle_start_ms = now_ms;
      }

      if (now_ms - auto_settle_start_ms >= AUTO_SETTLE_MS) {
        auto_state = AutoState::DONE;
      }
    }
    else {
      auto_settling = false;
    }
  }
  else {
    auto_settling = false;

    position_pid.calc(AUTO_DISTANCE, auto_distance);
    vx = position_pid.out;
  }

  return true;
}
// ---------- 功率限制 ----------

// 首先用较低上限验证限制功能。
// 后续改为：有效的 PM02 功率上限 - 经过验证的误差裕量。
constexpr float DEBUG_POWER_LIMIT = 80.0f;

// 调试数据：限制前预测、限制后预测、缩放系数、有效上限。
// status：0=正常，1=正在限制，2=模型无可行解或数据异常。
float power_debug[5] = {};

// 模型诊断快照：指令3项、反馈3项、时间s、状态、缩放、上限、有效标志。
float power_terms_data[11] = {};
uint32_t power_terms_origin_ms = 0;
bool power_terms_clock_started = false;

void publish_power_terms(
  const float command[3], const float feedback[3], uint32_t stamp_ms, float status, float scale,
  float limit, bool valid)
{
  taskENTER_CRITICAL();
  if (!power_terms_clock_started) {
    power_terms_origin_ms = stamp_ms;
    power_terms_clock_started = true;
  }
  for (int i = 0; i < 3; ++i) {
    power_terms_data[i] = command[i];
    power_terms_data[i + 3] = feedback[i];
  }
  power_terms_data[6] = static_cast<float>(stamp_ms - power_terms_origin_ms) * 0.001f;
  power_terms_data[7] = status;
  power_terms_data[8] = scale;
  power_terms_data[9] = limit;
  power_terms_data[10] = valid ? 1.0f : 0.0f;
  taskEXIT_CRITICAL();
}

// 未乘 k1、k2 的原始特征，供后续拟合。
void calculate_power_terms(
  const float torque[MOTOR_COUNT], const float speed[MOTOR_COUNT], float terms[3])
{
  terms[0] = terms[1] = terms[2] = 0.0f;
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    terms[0] += torque[i] * speed[i];
    terms[1] += torque[i] * torque[i];
    terms[2] += speed[i] * speed[i];
  }
}

// 功率对比诊断：[0]最终指令模型功率，[1]反馈转矩模型功率，单位 W。
// [2]为有效标志；失能、自动任务未启动、反馈过期或计算异常时为0。
float power_compare_data[3] = {};

void publish_power_compare(float command_power, float feedback_power, bool valid)
{
  taskENTER_CRITICAL();
  power_compare_data[0] = command_power;
  power_compare_data[1] = feedback_power;
  power_compare_data[2] = valid ? 1.0f : 0.0f;
  taskEXIT_CRITICAL();
}

// 按模型求最大的可行缩放系数 s。
// P(s) = A*s*s + B*s + C，要求 0 <= s <= 1。
// 返回 false 表示没有找到满足上限的有效解。
bool find_power_scale(
  const float torque[MOTOR_COUNT], const float speed[MOTOR_COUNT], float limit, float & scale)
{
  scale = 0.0f;

  if (!std::isfinite(limit) || limit <= 0.0f) {
    return false;
  }

  double A = 0.0;
  double B = 0.0;
  double C = a;

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    if (!std::isfinite(torque[i]) || !std::isfinite(speed[i])) {
      return false;
    }

    const double t = torque[i];
    const double w = speed[i];

    A += k1 * t * t;
    B += t * w;
    C += k2 * w * w;
  }

  // 原始指令没有超限，不缩小。
  if (A + B + C <= limit) {
    scale = 1.0f;
    return true;
  }

  const double D = C - limit;

  // A 很小时退化成一次不等式。
  if (A < 1e-12) {
    if (B > 1e-12 && D <= 0.0) {
      const double s = -D / B;
      scale = static_cast<float>(s < 0.0 ? 0.0 : (s > 1.0 ? 1.0 : s));
      return true;
    }

    return false;
  }

  const double discriminant = B * B - 4.0 * A * D;

  if (!std::isfinite(discriminant) || discriminant < 0.0) {
    return false;
  }

  const double root = std::sqrt(discriminant);
  const double lower_root = (-B - root) / (2.0 * A);
  const double upper_root = (-B + root) / (2.0 * A);

  // 对 A > 0，可行区间在两根之间。
  const double lower = lower_root > 0.0 ? lower_root : 0.0;
  const double upper = upper_root < 1.0 ? upper_root : 1.0;

  if (lower > upper || upper < 0.0 || lower > 1.0) {
    return false;
  }

  scale = static_cast<float>(upper);
  return true;
}

// 发布一组完整的调试数据。
void publish_power_debug(float before, float after, float scale, float limit, float status)
{
  taskENTER_CRITICAL();

  power_debug[0] = before;
  power_debug[1] = after;
  power_debug[2] = scale;
  power_debug[3] = limit;
  power_debug[4] = status;

  taskEXIT_CRITICAL();
}
}  // namespace

// 由 can_task 调用，生成控制数据，不负责发送 CAN。
void motor_make_frame(uint8_t * data)
{
  bool enabled;
  float stick_forward;
  float stick_horizontal;

  enabled = remote.is_alive(osKernelSysTick()) && remote.sw_r == sp::DBusSwitchMode::MID;

  stick_forward = remote.ch_lv;
  stick_horizontal = remote.ch_lh;

  // 清空整帧。
  for (int i = 0; i < 8; ++i) {
    data[i] = 0;
  }

  // 下档、上档或遥控器掉线：直接零力矩，不运行零速闭环。
  if (!enabled) {
    auto_reset();  // 新增：取消自动任务，清除位置环状态
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      speed_pid[i].clear();
      motors[i].cmd(0.0f);
      motors[i].write(data);
    }
    // 清除上一周期的控制调试值。
    // 这里的零是“未执行预测”的标记，不代表实际耗电为零。
    publish_power_debug(0.0f, 0.0f, 0.0f, DEBUG_POWER_LIMIT, 0.0f);
    publish_power_compare(0.0f, 0.0f, false);
    const float empty_terms[3] = {};
    publish_power_terms(
      empty_terms, empty_terms, osKernelSysTick(), 0.0f, 0.0f, DEBUG_POWER_LIMIT, false);
    return;
  }

  // 车体坐标：向前为 vx 正方向，向左为 vy 正方向。
  float vx = apply_deadzone(stick_forward) * MAX_SPEED;
  float vy = -apply_deadzone(stick_horizontal) * MAX_SPEED;

  // 限制合速度，避免斜推摇杆时速度增大。
  const float speed = std::sqrt(vx * vx + vy * vy);
  if (speed > MAX_SPEED) {
    const float scale = MAX_SPEED / speed;
    vx *= scale;
    vy *= scale;
  }

  // 拨轮控制旋转。
  // 从车顶俯视：逆时针为正，顺时针为负。
  // remote.ch_lu：拨轮向左为正，向右为负。
  const float dial = remote.ch_lu;
  float wz = 0.0f;

  if (dial > DIAL_DEADZONE) {
    wz = ROTATION_SPEED;  // 向左拨：逆时针，目标 1 rad/s
  }
  else if (dial < -DIAL_DEADZONE) {
    wz = -ROTATION_SPEED;  // 向右拨：顺时针，目标 1 rad/s
  }

  // 自动模式用位置外环生成 vx，同时将 vy、wz 设为零。
  // 手动模式保持原来的摇杆控制。
  if (!auto_update(vx, vy, wz)) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      speed_pid[i].clear();
      motors[i].cmd(0.0f);
      motors[i].write(data);
    }
    // 清除上一周期的控制调试值。
    // 这里的零是“未执行预测”的标记，不代表实际耗电为零。
    publish_power_debug(0.0f, 0.0f, 0.0f, DEBUG_POWER_LIMIT, 0.0f);
    publish_power_compare(0.0f, 0.0f, false);
    const float empty_terms[3] = {};
    publish_power_terms(
      empty_terms, empty_terms, osKernelSysTick(), 0.0f, 0.0f, DEBUG_POWER_LIMIT, false);

    return;
  }
  // 旋转产生的轮速分量，单位 m/s。
  const float rotation = ROTATION_LEVER * wz;

  // 麦轮解算：平移 + 旋转。
  // 此处每轮仍以“驱动车体向前”为正。
  // 后面的 MOTOR_DIRECTION 会转换到电机实际正方向。
  float target_speed[MOTOR_COUNT] = {};

  // vx：前后速度；vy：横向速度。
  // wz：底盘旋转角速度，逆时针为正。
  // 左侧 1、2 减去旋转项，右侧 3、4 加上旋转项。
  target_speed[FRONT_LEFT] = (vx + vy + ROTATION_LEVER * wz) / WHEEL_RADIUS;  // ID 1

  target_speed[REAR_LEFT] = (vx - vy - ROTATION_LEVER * wz) / WHEEL_RADIUS;  // ID 2

  target_speed[REAR_RIGHT] = (vx + vy - ROTATION_LEVER * wz) / WHEEL_RADIUS;  // ID 3

  target_speed[FRONT_RIGHT] = (vx - vy + ROTATION_LEVER * wz) / WHEEL_RADIUS;  // ID 4

  // 读取四轮输出轴角速度，单位 rad/s。
  float measured_speed[MOTOR_COUNT];

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    measured_speed[i] = motors[i].speed;
  }

  float target_torque[MOTOR_COUNT] = {};

  // 1. 先算完四个速度 PID，不立即发送。
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    target_speed[i] *= MOTOR_DIRECTION[i];

    speed_pid[i].calc(target_speed[i], measured_speed[i]);
    target_torque[i] = speed_pid[i].out;
  }

  // 2. 使用原始力矩指令预测总功率。
  // 使用未滤波的模型结果，显示滤波不参与这里的控制。
  const float power_before = calculate_power(target_torque, measured_speed);

  const float power_limit = DEBUG_POWER_LIMIT;

  float power_scale = 1.0f;

  const bool feasible = find_power_scale(target_torque, measured_speed, power_limit, power_scale);

  float status = 0.0f;

  if (!feasible) {
    // 异常或模型找不到可行解：撤掉驱动力矩。
    // 这不代表实际功率已经满足上限，必须通过状态和功率计观察。
    power_scale = 0.0f;
    status = 2.0f;
  }
  else if (power_scale < 1.0f) {
    status = 1.0f;
  }

  // 3. 统一缩放四轮力矩，保留原有力矩比例。
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    target_torque[i] *= power_scale;
  }

  // 4. 重新预测限制后的功率。
  const float power_after = calculate_power(target_torque, measured_speed);

  // 5. 第一版采用简单的积分处理：
  // 限制介入时清除 PID 状态，防止积分继续积累。
  // 本周期使用的是上面已经保存、缩放后的力矩。
  //
  // 你当前 KD=0，适合先用这种保守方法验证。
  // 后续可改成更平滑的条件积分或反算抗饱和。
  if (status != 0.0f) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      speed_pid[i].clear();
    }
  }

  // 6. 发布数据。
  publish_power_debug(power_before, power_after, power_scale, power_limit, status);

  // 比较同一份反馈速度下，最终力矩指令与反馈转矩的模型功率。
  // 使用 target_torque，避免 PID clear() 后读到零，也避免使用限幅前指令。
  float feedback_torque[MOTOR_COUNT];
  float feedback_speed[MOTOR_COUNT];
  bool feedback_alive = true;

  taskENTER_CRITICAL();
  const uint32_t compare_now_ms = osKernelSysTick();
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    feedback_torque[i] = motors[i].torque;
    feedback_speed[i] = motors[i].speed;
    if (!motors[i].is_alive(compare_now_ms)) {
      feedback_alive = false;
    }
  }
  taskEXIT_CRITICAL();

  const float command_model_power = calculate_power(target_torque, feedback_speed);
  const float feedback_model_power = calculate_power(feedback_torque, feedback_speed);
  const bool compare_valid = feasible && feedback_alive && std::isfinite(command_model_power) &&
                             std::isfinite(feedback_model_power);

  publish_power_compare(command_model_power, feedback_model_power, compare_valid);

  float command_terms[3];
  float feedback_terms[3];
  calculate_power_terms(target_torque, feedback_speed, command_terms);
  calculate_power_terms(feedback_torque, feedback_speed, feedback_terms);
  publish_power_terms(
    command_terms, feedback_terms, compare_now_ms, status, power_scale, power_limit, compare_valid);

  // 7. 最后才发送限制后的力矩。
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    motors[i].cmd(target_torque[i]);
    motors[i].write(data);
  }
}

// 由 CAN 接收回调调用，更新电机反馈。
void motor_receive(uint32_t rx_id, uint8_t * data, uint32_t stamp_ms)
{
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    if (rx_id == motors[i].rx_id) {
      motors[i].read(data, stamp_ms);
      return;
    }
  }
}
// 获取四个电机的实际角速度，顺序为 ID 1、2、3、4。
// 保留电机反馈原始正负号，单位 rad/s。
void motor_get_speed(float speed[4])
{
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    speed[i] = motors[i].speed;
  }
}

// motor_id：电调 ID，取值 1～4。
// data：目标速度、反馈速度、输出力矩、积分输出。
void motor_get_pid_debug(int motor_id, float data[4])
{
  for (int i = 0; i < 4; ++i) {
    data[i] = 0.0f;
  }

  if (motor_id < 1 || motor_id > MOTOR_COUNT) {
    return;
  }

  const int index = motor_id - 1;

  // 只在复制数据时进入临界区，不在临界区发送串口。

  data[0] = speed_pid[index].data.set;   // 目标速度，rad/s
  data[1] = motors[index].speed;         // 实际速度，rad/s
  data[2] = speed_pid[index].out;        // PID 输出，N·m
  data[3] = speed_pid[index].data.iout;  // 积分输出，N·m
}
// 顺序：ID1转矩、ID1速度、ID2转矩、ID2速度……
// 转矩：N·m；角速度：rad/s。
void motor_get_axis_data(float data[8])
{
  taskENTER_CRITICAL();

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    data[2 * i] = motors[i].torque;
    data[2 * i + 1] = motors[i].speed;
  }

  taskEXIT_CRITICAL();
}
// 兼容旧 Plotter：返回最近一次控制周期的最终指令模型功率。
// 失能时返回0作为未计算标记；有效性请使用 motor_get_power_compare 查询。
float motor_get_predicted_power()
{
  taskENTER_CRITICAL();
  const float power = power_compare_data[0];
  taskEXIT_CRITICAL();
  return power;
}

void motor_get_power_debug(float data[5])
{
  taskENTER_CRITICAL();

  for (int i = 0; i < 5; ++i) {
    data[i] = power_debug[i];
  }

  taskEXIT_CRITICAL();
}
// 与5通道 Plotter 配套：两种模型功率和有效标志。
void motor_get_power_compare(float data[3])
{
  taskENTER_CRITICAL();
  for (int i = 0; i < 3; ++i) {
    data[i] = power_compare_data[i];
  }
  taskEXIT_CRITICAL();
}

// 从任务中调用；返回同一控制周期的完整快照。
void motor_get_power_terms(float data[11])
{
  taskENTER_CRITICAL();
  for (int i = 0; i < 11; ++i) {
    data[i] = power_terms_data[i];
  }
  taskEXIT_CRITICAL();
}
