#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

void motor_get_axis_data(float data[8]);
bool power_meter_get(float data[3]);
float motor_get_predicted_power();

sp::Plotter plotter(&huart1);

extern "C" void plotter_task(void const * argument)
{
  float axis_data[8] = {};
  float power_data[3] = {};

  while (true) {
    motor_get_axis_data(axis_data);
    const bool power_alive = power_meter_get(power_data);
    const float predicted_power = motor_get_predicted_power();

    plotter.plot(
      /*axis_data[0],   // 通道1：ID 1 反馈转矩，N·m
      axis_data[1],   // 通道2：ID 1 实际角速度，rad/s
      axis_data[2],   // 通道3：ID 2 反馈转矩，N·m
      axis_data[3],   // 通道4：ID 2 实际角速度，rad/s
      axis_data[4],   // 通道5：ID 3 反馈转矩，N·m
      axis_data[5],   // 通道6：ID 3 实际角速度，rad/s*/
      //axis_data[6],   // 通道7：ID 4 反馈转矩，N·m
      //axis_data[7],   // 通道8：ID 4 实际角速度，rad/s
      power_data[2],  // 通道9：功率计实际功率，W
      //power_alive ? 1.0f : 0.0f, // 通道10：功率计在线状态
      predicted_power  // 通道11：预测功率，W
    );

    osDelay(10);
  }
}