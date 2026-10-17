#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 函数定义在 motor_task.cpp。
void motor_get_speed(float speed[4]);
void motor_get_pid_debug(int motor_id, float data[4]);

// 使用 UART1 向上位机发送数据。
sp::Plotter plotter(&huart1);

namespace
{
// 修改这里，选择要观察的电调 ID：1、2、3、4。
// 这里只切换观察对象，不会让其他电机失能。
constexpr int DEBUG_MOTOR_ID = 1;
}  // namespace

extern "C" void plotter_task(void const * argument)
{
  float data[4] = {};

  while (true) {
    motor_get_pid_debug(DEBUG_MOTOR_ID, data);

    plotter.plot(
      data[0],  // 曲线1：目标速度，rad/s
      data[1],  // 曲线2：实际速度，rad/s
      data[2],  // 曲线3：PID 输出力矩，N·m
      data[3]   // 曲线4：积分输出力矩，N·m
    );

    osDelay(10);
  }
}