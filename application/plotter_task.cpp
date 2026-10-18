#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

void motor_get_power_terms(float data[11]);
bool power_meter_get(float data[3]);

sp::Plotter plotter(&huart1);

extern "C" void plotter_task(void const * argument)
{
  float terms[11] = {};
  float meter[3] = {};

  while (true) {
    motor_get_power_terms(terms);
    const bool alive = power_meter_get(meter);

    // 不滤波。时间为电机诊断快照时刻；功率计是最近收到的测量值，
    // 并非与电机反馈硬件同步采样，分析时仍需考虑测量延迟。
    plotter.plot(
      terms[0],            // 1：最终指令 Σ(torque * speed)，W
      terms[1],            // 2：最终指令 Σ(torque^2)，(N*m)^2
      terms[2],            // 3：实际 Σ(speed^2)，(rad/s)^2
      terms[3],            // 4：反馈 Σ(torque * speed)，W
      terms[4],            // 5：反馈 Σ(torque^2)，(N*m)^2
      terms[5],            // 6：同一实际 Σ(speed^2)，与通道3一致
      meter[2],            // 7：功率计实际功率，W
      terms[6],            // 8：相对首次诊断快照的时间，s
      terms[7],            // 9：限制状态，0正常/1限制/2异常
      terms[8],            // 10：缩放系数，0~1
      terms[9],            // 11：控制功率上限，W
      terms[10],           // 12：诊断有效标志
      alive ? 1.0f : 0.0f  // 13：功率计在线标志
    );

    osDelay(10);
  }
}
