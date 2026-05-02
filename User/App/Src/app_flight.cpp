#include "app_flight.h"

#include "Config.h"

#include "alg_control.h"
#include "dev_bmi088.h"
#include "dev_k230.h"
#include "dev_servo.h"
#include "dev_temp_ctrl.h"
#include "drv_uart.h"
#include "vofa.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

// 设备实例化（置于匿名 namespace 外部，便于 Watch 窗口查看）
Bmi088 g_bmi088(&hspi1);
K230 g_k230;
Control g_control;
Servo g_servo(&htim1);
TempCtrl g_temp_ctrl(&htim3, PinDef::TempCtrl::HEATER_PWM_CHANNEL);

namespace {

constexpr uint32_t kFlightPeriodMs = HwConfig::App::kFlightPeriodMs;
constexpr uint32_t kK230TimeoutMs = HwConfig::App::kK230TimeoutMs;
constexpr float kCmdLimitG = FcParam::Control::kGuidanceCmdLimitG;
constexpr uint16_t kK230RxBufferSize = HwConfig::App::kK230RxBufferSize;
// BMI088 目标工作温度（℃），减小温漂。
constexpr float kTargetTempC = 40.0f;

bool g_app_inited = false;
uint32_t g_last_run_ms = 0U;
uint32_t g_last_k230_rx_ms = 0U;

uint8_t g_k230_rx_buffer[kK230RxBufferSize] = {0U};

float Clamp(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}

	if (value > max_value) {
		return max_value;
	}

	return value;
}

bool IsFiniteFloat(float value)
{
	return (value == value) && (value > -1.0e6f) && (value < 1.0e6f);
}

namespace DevBmi088 {

bool Read(Vector3f& gyro, Vector3f& accel)
{
	Bmi088::Data data = {};
	if (!g_bmi088.ReadSensor(&data)) {
		return false;
	}

	gyro.x = data.gyro_rads[0];
	gyro.y = data.gyro_rads[1];
	gyro.z = data.gyro_rads[2];

	accel.x = data.accel_mps2[0];
	accel.y = data.accel_mps2[1];
	accel.z = data.accel_mps2[2];

	return true;
}

} // namespace DevBmi088

namespace DevK230 {

GuidanceCmd GetCommand()
{
	return g_k230.GetGuidanceCommand();
}

bool IsOnline(uint32_t now_ms)
{
	if (!g_k230.HasValidCommand()) {
		return false;
	}

	return ((now_ms - g_last_k230_rx_ms) <= kK230TimeoutMs);
}

} // namespace DevK230

namespace AlgControl {

RpyCommand Update(float ny_cmd, float nz_cmd, const Vector3f& gyro, const Vector3f& accel)
{
	return g_control.Update(ny_cmd, nz_cmd, gyro, accel);
}

void Mixer(float R, float P, float Y, float* delta_out)
{
	g_control.Mixer(R, P, Y, delta_out);
}

} // namespace AlgControl

namespace DevServo {

void SetFinAngles(float* delta)
{
	g_servo.SetFinAngles(delta);
}

} // namespace DevServo

void K230RxCallback(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t length, void* userContext)
{
	(void)userContext;
	if ((huart != &huart1) || (buffer == nullptr) || (length == 0U)) {
		return;
	}

	g_k230.ParseProtocol(buffer, length);
	if (g_k230.HasValidCommand()) {
		g_last_k230_rx_ms = HAL_GetTick();
	}
}

bool InitK230Rx()
{
	// 配置 UART 接收事件处理项 绑定相关信息
	DrvUART::RxEventHandler handler = {};
	handler.huart = &huart1;
	handler.buffer = g_k230_rx_buffer;
	handler.bufferSize = kK230RxBufferSize;
	handler.callback = &K230RxCallback;
	handler.userContext = nullptr;

	// 注册 UART 的g_rx_handlers[]表
	if (!DrvUART::RegisterRxEventHandler(handler)) {
		return false;
	}
	// 启动 UART DMA 空闲接收
	DrvUART::StartReceive_DMA_Idle(&huart1, g_k230_rx_buffer, kK230RxBufferSize);
	return true;
}

} // namespace

extern "C" bool App_Flight_Init(void)
{
	Vofa::Init(&huart1);

	bool ok = true;

	ok = ok && g_bmi088.Init();
	ok = ok && g_servo.Init();
	ok = ok && InitK230Rx();
	ok = ok && g_temp_ctrl.Init();
	g_temp_ctrl.SetTarget(kTargetTempC);

	// 与舵机设备层限幅保持一致。
	g_control.SetFinLimit(FcParam::Servo::kAngleLimitDeg);

	g_app_inited = ok;
	g_last_run_ms = HAL_GetTick();
	return ok;
}

extern "C" void App_Flight_Task(void)
{
	// 初始化未完成时，每隔 1 秒重试一次（避免频繁阻塞）。
	if (!g_app_inited) {
		static uint32_t g_retry_after_ms = 0U;
		uint32_t now_ms = HAL_GetTick();
		if (now_ms < g_retry_after_ms) {
			return;
		}
		if (App_Flight_Init()) {
			g_retry_after_ms = 0U;
		} else {
			g_retry_after_ms = now_ms + 1000U;
			return;
		}
	}

	uint32_t now_ms = HAL_GetTick();
	if ((now_ms - g_last_run_ms) < kFlightPeriodMs) {
		return;
	}
	g_last_run_ms = now_ms;

	// Step1: 获取传感器数据 + 温度闭环
	Vector3f gyro = {};
	Vector3f accel = {};
	if (!DevBmi088::Read(gyro, accel)) {
		float safe_delta[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		DevServo::SetFinAngles(safe_delta);
		return;
	}

	float temp_c = 0.0f;
	g_bmi088.ReadTemperature(&temp_c);
	g_temp_ctrl.Update(temp_c);

	// Step2: 获取制导指令（包含超时保护）
	float ny_cmd = 0.0f;
	float nz_cmd = 0.0f;
	if (DevK230::IsOnline(now_ms)) {
		GuidanceCmd cmd = DevK230::GetCommand();
		if (IsFiniteFloat(cmd.ny_c) && IsFiniteFloat(cmd.nz_c)) {
			ny_cmd = Clamp(cmd.ny_c, -kCmdLimitG, kCmdLimitG);
			nz_cmd = Clamp(cmd.nz_c, -kCmdLimitG, kCmdLimitG);
		}
	}

	// Step3: 控制律解算
	RpyCommand rpy = AlgControl::Update(ny_cmd, nz_cmd, gyro, accel);

	// Step4: 舵面分配
	float delta[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	AlgControl::Mixer(rpy.R, rpy.P, rpy.Y, delta);

	// Step5: 硬件执行
	DevServo::SetFinAngles(delta);

	// Vofa+ 输出调试数据（10Hz）
	// ch 0-2: filtered accel    ch 3-5: filtered gyro
	// ch 6: target_temperature (℃)    ch 7: now_temperature (℃)
	static uint8_t vofa_div = 0U;
	if (++vofa_div >= 10U) {
		vofa_div = 0U;
		const Bmi088::Data& flt = g_bmi088.filtered_data_;
		float dbg[8] = {
			flt.accel_mps2[0], flt.accel_mps2[1], flt.accel_mps2[2],
			flt.gyro_rads[0],  flt.gyro_rads[1],  flt.gyro_rads[2],
			g_temp_ctrl.current_temp_,
			static_cast<float>(g_temp_ctrl.pwm_duty_)
		};
		Vofa::JustFloat(dbg, 8);
	}
}

