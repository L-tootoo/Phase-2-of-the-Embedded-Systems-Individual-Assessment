#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 定义在 can_task.cpp。
bool power_meter_get(float data[3]);

sp::Plotter plotter(&huart1);

extern "C" void plotter_task(void const * argument)
{
  float data[3] = {};

  while (true) {
    const bool alive = power_meter_get(data);

    plotter.plot(
      data[0],             // 曲线1：电流，A
      data[1],             // 曲线2：电压，V
      data[2],             // 曲线3：实际功率，W
      alive ? 1.0f : 0.0f  // 曲线4：在线为1，未收到或超时为0
    );

    osDelay(10);
  }
}