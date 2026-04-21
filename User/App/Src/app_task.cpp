#include "app_task.h"
#include "drv_uart.h"

#include "Config.h"

#include "app_flight.h"
#include "tim.h"

volatile uint32_t t_ms = 0U;
volatile bool task_10ms_flag = false;
volatile bool task_20ms_flag = false;
volatile uint32_t task_overload_count = 0U;

namespace {

/**
 * @brief 20ms 分频计数器。
 * @details TIM2 100Hz 每次中断 10ms，累计 2 次触发 20ms 任务。
 */
volatile uint8_t g_div20_count = 0U;

namespace AppFlight {

void ControlLoop()
{
	App_Flight_Task();
}

} // namespace AppFlight

namespace DevK230 {

void Update()
{
	// 当前 K230 数据链路由 DMA+回调驱动，此处保留周期接口用于后续扩展。
}

} // namespace DevK230

} // namespace

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if ((htim == nullptr) || (htim->Instance != TIM2)) {
		return;
	}

	// TIM2 调度周期来自统一配置。
	t_ms += HwConfig::Task::kTickMs;

	// 10ms 任务每次中断触发一次。
	task_10ms_flag = true;

	// 20ms 任务每 2 次中断触发一次。
	++g_div20_count;
	if (g_div20_count >= HwConfig::Task::kTask20MsDivider) {
		g_div20_count = 0U;
		task_20ms_flag = true;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  DrvUART_RxEventIRQHandler(huart, Size);
}

extern "C" bool App_Task_Init(void)
{
	if (!App_Flight_Init()) {
		return false;
	}

	// 启动 TIM2 基础定时中断（100Hz），驱动前后台任务标志调度。
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		return false;
	}

	return true;
}

extern "C" void Task_Loop(void)
{
	bool run_10ms = false;
	bool run_20ms = false;

	// 性能保护：若两个任务标志同时置位，记录一次拥塞事件。
	__disable_irq();
	if (task_10ms_flag && task_20ms_flag) {
		++task_overload_count;
	}

	if (task_10ms_flag) {
		task_10ms_flag = false;
		run_10ms = true;
	}

	if (task_20ms_flag) {
		task_20ms_flag = false;
		run_20ms = true;
	}
	__enable_irq();

	// 前后台调度：先执行控制闭环，再执行低频任务。
	if (run_10ms) {
		AppFlight::ControlLoop();
	}

	if (run_20ms) {
		DevK230::Update();
	}
}

