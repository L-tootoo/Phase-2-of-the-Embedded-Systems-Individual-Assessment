#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "task.h"

// 定义在 motor_task.cpp。
void motor_make_frame(uint8_t * data);

void motor_receive(uint32_t rx_id, uint8_t * data, uint32_t stamp_ms);

namespace
{
sp::CAN chassis_can(&hcan1);

// 功率计测量数据。
struct PowerMeterData
{
  float current = 0.0f;  // A
  float voltage = 0.0f;  // V
  float power = 0.0f;    // W

  uint32_t last_update_ms = 0;
  bool received = false;
  uint32_t sequence = 0;  // 每收到一帧有效功率计报文加一。
};

PowerMeterData power_meter;

// 暂定 500 ms 未收到数据视为掉线。
// 后续根据功率计实际发送周期调整。
constexpr uint32_t POWER_METER_TIMEOUT_MS = 500;

// 高字节在前，将两个字节合并为 uint16_t。
uint16_t read_u16_be(const uint8_t * data)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
}

// 解析功率计数据。
// 按截图中的格式，暂按无符号 16 位数解析。
void power_meter_receive(const uint8_t * data, uint32_t now_ms)
{
  power_meter.current = read_u16_be(&data[0]) / 100.0f;
  power_meter.voltage = read_u16_be(&data[2]) / 100.0f;
  power_meter.power = read_u16_be(&data[4]) / 100.0f;

  // data[6]、data[7] 为保留字节，不参与计算。
  power_meter.last_update_ms = now_ms;
  power_meter.received = true;
  ++power_meter.sequence;
}
}  // namespace

// 供 plotter_task 等任务读取。
// data[0]：电流，data[1]：电压，data[2]：功率。
// 返回 true 表示数据已收到且尚未超时。
bool power_meter_get(float data[3])
{
  bool alive;

  taskENTER_CRITICAL();

  const uint32_t now_ms = osKernelSysTick();

  data[0] = power_meter.current;
  data[1] = power_meter.voltage;
  data[2] = power_meter.power;

  alive = power_meter.received && (now_ms - power_meter.last_update_ms < POWER_METER_TIMEOUT_MS);

  taskEXIT_CRITICAL();

  return alive;
}

// 周期发送四个电机的控制指令。
extern "C" void can_task(void const * argument)
{
  chassis_can.config();
  chassis_can.start();

  while (true) {
    // motor_task 生成 ID 1～4 的电机控制数据。
    motor_make_frame(chassis_can.tx_data);

    // C620 的 ID 1～4 共用 0x200 控制帧。
    chassis_can.send(0x200);

    osDelay(2);
  }
}

// CAN1 接收中断：分发电机反馈和功率计反馈。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};

    // 直接用 HAL 获取完整帧头，便于检查帧类型和长度。
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      break;
    }

    // 这里只处理 8 字节的标准数据帧。
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) {
      continue;
    }

    const uint32_t now_ms = osKernelSysTick();

    if (header.StdId == 0x301) {
      // 功率计反馈。
      power_meter_receive(data, now_ms);
    }
    else if (header.StdId >= 0x201 && header.StdId <= 0x204) {
      // ID 1～4 电机反馈。
      motor_receive(header.StdId, data, now_ms);
    }
  }
}
// 接收时间不是功率计内部采样时间，不能据此消除其内部延迟。
bool power_meter_get_fit_snapshot(float data[3], uint32_t & stamp_ms, uint32_t & sequence)
{
  taskENTER_CRITICAL();
  data[0] = power_meter.current;
  data[1] = power_meter.voltage;
  data[2] = power_meter.power;
  stamp_ms = power_meter.last_update_ms;
  sequence = power_meter.sequence;
  const bool alive =
    power_meter.received && (osKernelSysTick() - stamp_ms < POWER_METER_TIMEOUT_MS);
  taskEXIT_CRITICAL();
  return alive;
}
