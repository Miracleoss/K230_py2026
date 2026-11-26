#ifndef TASK_BRIDGE_H
#define TASK_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 任务桥接层
 *
 * 提供 C 风格的对外接口，供主程序调用以初始化与运行飞控任务。
 * 通过 extern "C" 保证 C/C++ 混合编译时的符号兼容性。
 */

/// 初始化飞控任务及其闭环运行状态；成功后由主循环按固定节拍驱动。
void Task_Init(void);

/**
 * @brief 飞控任务主循环（由主程序周期调用）。
 *
 * 每个毫秒刻度至多执行一次：轮询并锁存最新主机过载指令，按 5 ms 采集 IMU，
 * 仅以新 IMU 样本推进闭环；主机指令与 IMU 分别在 kHostCommandTimeoutMs、
 * kImuStaleTimeoutMs 规定的时限内新鲜且设备就绪时才解算并输出。超时或解算
 * 失败会复位闭环并输出空指令，使舵机进入回中路径。
 */
void Task_Loop(void);

#ifdef __cplusplus
}

#include "Common/common_types.h"
#include "Common/control_command.h"
#include "Common/dart_light_status.h"
#include "Common/host_overload_command.h"
#include "Common/imu_data.h"

static const uint8_t kTaskImuTelemetryHostProtocol = 0U;
static const uint8_t kTaskImuTelemetryVofaJustFloat = 1U;

/// 0 = existing Host frame; 1 = VOFA+ JustFloat. Change in Watch for a live selection.
extern volatile uint8_t imu_telemetry_mode;

/// 取走最新一条从 USART1 解码出的有效物理过载指令。
ResultCode Task_TakeLatestHostOverload(HostOverloadCommand& command);

/// 编码一份有效的六轴采样并启动一次异步 USART1 DMA 发送。
ResultCode Task_SendImuData(const ImuData& imu, const BodyControlCommand& command,
                            float overload_yaw_rate_radps, float overload_pitch_rate_radps);

/// Encode and send one valid IMU sample using the VOFA+ JustFloat test format.
ResultCode Task_SendVofaImuData(const ImuData& imu);

/// 飞镖发光装置对外控制与状态接口
ResultCode Task_DartLightSetTeam(DartLightTeam team);
ResultCode Task_DartLightRequestStatus();
const DartLightStatus& Task_DartLightGetStatus();

void output_test();

#endif

#endif /* TASK_BRIDGE_H */
