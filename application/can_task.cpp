#include "cmsis_os.h"
#include "io/can/can.hpp"

// 函数定义在 motor_task.cpp。
// 这里只声明，让本文件能够调用，不需要新建 .hpp。
void motor_make_frame(uint8_t *data);

void motor_receive(
    uint32_t rx_id,
    uint8_t *data,
    uint32_t stamp_ms);

namespace
{
sp::CAN chassis_can(&hcan1);
}

extern "C" void can_task(void const *argument)
{
    chassis_can.config();
    chassis_can.start();

    while (true) {
        // 电机部分决定发送什么。
        motor_make_frame(chassis_can.tx_data);

        // CAN 部分负责把数据发出去。
        // 0x200 对应 C620 的 ID 1～4 控制指令。
        chassis_can.send(0x200);

        osDelay(2);
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(
               hcan, CAN_RX_FIFO0) > 0) {
        chassis_can.recv();

        // 电机反馈使用标准帧。
        if (!chassis_can.frame_type) {
            motor_receive(
                chassis_can.rx_id,
                chassis_can.rx_data,
                osKernelSysTick());
        }
    }
}