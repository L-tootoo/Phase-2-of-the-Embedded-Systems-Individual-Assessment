#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const *argument)
{
  led.start();

  while (true) {
    // 红 → 绿
    for (float x = 0; x <= 1.0f; x += 0.01f) {
      led.set(1.0f - x, x, 0);
      osDelay(10);
    }

    // 绿 → 蓝
    for (float x = 0; x <= 1.0f; x += 0.01f) {
      led.set(0, 1.0f - x, x);
      osDelay(10);
    }

    // 蓝 → 红
    for (float x = 0; x <= 1.0f; x += 0.01f) {
      led.set(x, 0, 1.0f - x);
      osDelay(10);
    }
  }
}