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
constexpr float SPEED_KI = 0.0f;
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
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      speed_pid[i].clear();
      motors[i].cmd(0.0f);
      motors[i].write(data);
    }
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

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    // 转换到电机反馈使用的正负方向。
    target_speed[i] *= MOTOR_DIRECTION[i];

    // 速度闭环：rad/s → 输出轴力矩 N·m。
    speed_pid[i].calc(target_speed[i], measured_speed[i]);

    motors[i].cmd(speed_pid[i].out);
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