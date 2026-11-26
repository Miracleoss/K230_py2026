#include <stdint.h>
#include <stdio.h>
#include <limits>

#include "Application/app_dart_system.h"
#include "Config/servo_config.h"
#include "Driver/drv_pwm_channel.h"

extern "C" HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t)
{
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*, uint32_t)
{
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef*,
    uint8_t*,
    uint8_t*,
    uint16_t,
    uint32_t)
{
    return HAL_OK;
}

extern "C" void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, GPIO_PinState)
{
}

extern "C" GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t)
{
    return GPIO_PIN_RESET;
}

void TestHalSetCompare(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t compare)
{
    timer->compare[channel / 4U] = compare;
}

static bool Expect(bool condition, const char* message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        return false;
    }
    return true;
}

static void NoDelay(uint32_t)
{
}

static bool BindControlOutputs(
    Class_DartSystem& system,
    TIM_HandleTypeDef& timer,
    Class_PwmChannel (&pwm)[kSurfaceCount])
{
    const uint32_t channels[kSurfaceCount] = {
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        TIM_CHANNEL_3,
        TIM_CHANNEL_4
    };

    for (uint32_t index = 0U; index < kSurfaceCount; ++index) {
        if (pwm[index].Bind(&timer, channels[index], 1000000U) != ResultCode::Ok) {
            return false;
        }
    }

    // Deliberately bind out of order: logical placement must come from SurfaceId.
    return system.BindServo(&pwm[2], &kServoConfigs[2]) == ResultCode::Ok &&
           system.BindServo(&pwm[0], &kServoConfigs[0]) == ResultCode::Ok &&
           system.BindServo(&pwm[3], &kServoConfigs[3]) == ResultCode::Ok &&
           system.BindServo(&pwm[1], &kServoConfigs[1]) == ResultCode::Ok;
}

int main()
{
    bool passed = true;

    Class_DartSystem unbound_system;
    Bmi088RawSample raw = {};
    passed &= Expect(
        unbound_system.InitializeBmi088(NoDelay) == ResultCode::NotReady,
        "unbound BMI088 initialization must report NotReady");
    passed &= Expect(
        unbound_system.ReadBmi088RawSample(raw) == ResultCode::NotReady,
        "unbound BMI088 sampling must report NotReady");
    Bmi088ProbeTrace trace = {};
    uint8_t traced_acc_chip_id = 0U;
    uint8_t traced_gyro_chip_id = 0U;
    passed &= Expect(
        unbound_system.ProbeBmi088WithTrace(traced_acc_chip_id, traced_gyro_chip_id, trace) ==
            ResultCode::NotReady,
        "unbound traced BMI088 probe must report NotReady");
    passed &= Expect(
        !unbound_system.IsBmi088Initialized(),
        "unbound BMI088 must not report initialized");
    BodyControlCommand valid_command;
    valid_command.valid = true;
    passed &= Expect(
        unbound_system.ApplyBodyControlCommand(valid_command) == ResultCode::NotReady,
        "a control command must be rejected until all four servos are bound");

    ImuData neutral_imu;
    neutral_imu.valid = true;
    neutral_imu.sample_timestamp_ms = 5U;
    neutral_imu.data_ready_seq = 1U;
    BodyControlCommand neutral_closed_loop_command;
    neutral_closed_loop_command.roll = 1.0f;
    neutral_closed_loop_command.pitch = -1.0f;
    neutral_closed_loop_command.yaw = 1.0f;
    passed &= Expect(
        unbound_system.UpdateClosedLoop(0.0f, 0.0f, neutral_imu, 0.005f, neutral_closed_loop_command) ==
            ResultCode::Ok,
        "a valid IMU sample must execute the closed-loop controller");
    passed &= Expect(
        neutral_closed_loop_command.valid &&
            neutral_closed_loop_command.roll == 0.0f &&
            neutral_closed_loop_command.pitch == 0.0f &&
            neutral_closed_loop_command.yaw == 0.0f &&
            neutral_closed_loop_command.timestamp_ms == 5U,
        "neutral controller outputs must map to a zero body command");

    neutral_imu.valid = false;
    passed &= Expect(
        unbound_system.UpdateClosedLoop(0.0f, 0.0f, neutral_imu, 0.005f, neutral_closed_loop_command) ==
            ResultCode::InvalidData,
        "an invalid IMU sample must reject the closed-loop update");
    passed &= Expect(
        !neutral_closed_loop_command.valid &&
            neutral_closed_loop_command.roll == 0.0f &&
            neutral_closed_loop_command.pitch == 0.0f &&
            neutral_closed_loop_command.yaw == 0.0f,
        "an invalid closed-loop update must return a neutral invalid body command");

    TIM_HandleTypeDef partial_timer = {};
    partial_timer.Init.Period = 5000U;
    Class_PwmChannel partial_pwm;
    Class_DartSystem partial_system;
    passed &= Expect(
        partial_pwm.Bind(&partial_timer, TIM_CHANNEL_1, 1000000U) == ResultCode::Ok &&
            partial_system.BindServo(&partial_pwm, &kServoConfigs[0]) == ResultCode::Ok &&
            partial_pwm.SetPulseWidthUs(1000U) == ResultCode::Ok,
        "partial binding setup must succeed");
    BodyControlCommand invalid_partial_command;
    passed &= Expect(
        partial_system.ApplyBodyControlCommand(invalid_partial_command) == ResultCode::InvalidData,
        "an invalid command must take precedence over incomplete readiness");
    passed &= Expect(
        partial_timer.compare[0] == 1470U,
        "an invalid command must neutralize every already-bound servo");

    passed &= Expect(
        partial_pwm.SetPulseWidthUs(1000U) == ResultCode::Ok,
        "partial valid-command setup must move the bound servo away from neutral");
    BodyControlCommand valid_partial_command;
    valid_partial_command.valid = true;
    passed &= Expect(
        partial_system.ApplyBodyControlCommand(valid_partial_command) == ResultCode::NotReady,
        "a valid command must report NotReady while any servo is unbound");
    passed &= Expect(
        partial_timer.compare[0] == 1470U,
        "a NotReady control path must neutralize every already-bound servo");

    TIM_HandleTypeDef timer = {};
    timer.Init.Period = 5000U;
    Class_PwmChannel pwm[kSurfaceCount];
    Class_DartSystem system;
    passed &= Expect(
        BindControlOutputs(system, timer, pwm),
        "all four logical servos must bind through Class_DartSystem");

    BodyControlCommand command;
    command.roll = 0.25f;
    command.pitch = 0.125f;
    command.yaw = -0.125f;
    command.timestamp_ms = 42U;
    command.valid = true;

    passed &= Expect(
        system.ApplyBodyControlCommand(command) == ResultCode::Ok,
        "a valid body command must reach all four servo outputs");
    passed &= Expect(timer.compare[0] == 1345U, "Servo1 pulse must match mixed delta1=0.25");
    passed &= Expect(timer.compare[1] == 1470U, "Servo2 pulse must match mixed delta2=0.0");
    passed &= Expect(timer.compare[2] == 1345U, "Servo3 pulse must match mixed delta3=0.25");
    passed &= Expect(timer.compare[3] == 1320U, "Servo4 pulse must match mixed delta4=0.5");

    command.valid = false;
    passed &= Expect(
        system.ApplyBodyControlCommand(command) == ResultCode::InvalidData,
        "an invalid body command must be rejected");
    passed &= Expect(timer.compare[0] == 1470U, "invalid command must neutralize Servo1");
    passed &= Expect(timer.compare[1] == 1470U, "invalid command must neutralize Servo2");
    passed &= Expect(timer.compare[2] == 1470U, "invalid command must neutralize Servo3");
    passed &= Expect(timer.compare[3] == 1570U, "invalid command must neutralize Servo4");

    command.valid = true;
    command.roll = 1.25f;
    command.pitch = 0.0f;
    command.yaw = 0.0f;
    passed &= Expect(
        system.ApplyBodyControlCommand(command) == ResultCode::InvalidData,
        "an out-of-range body axis must be rejected");
    passed &= Expect(
        timer.compare[0] == 1470U && timer.compare[1] == 1470U &&
            timer.compare[2] == 1470U && timer.compare[3] == 1570U,
        "an out-of-range body command must keep every servo neutral");

    command.roll = std::numeric_limits<float>::quiet_NaN();
    passed &= Expect(
        system.ApplyBodyControlCommand(command) == ResultCode::InvalidData,
        "a non-finite body axis must be rejected");
    passed &= Expect(
        timer.compare[0] == 1470U && timer.compare[1] == 1470U &&
            timer.compare[2] == 1470U && timer.compare[3] == 1570U,
        "a non-finite body command must keep every servo neutral");

    TIM_HandleTypeDef limited_timer = {};
    limited_timer.Init.Period = 1500U;
    Class_PwmChannel limited_pwm;
    passed &= Expect(
        limited_pwm.Bind(&limited_timer, TIM_CHANNEL_3, 1000000U) == ResultCode::Ok &&
            system.BindServo(&limited_pwm, &kServoConfigs[2]) == ResultCode::Ok,
        "output-failure setup must replace Servo3 with a limited PWM channel");
    command.roll = -0.25f;
    passed &= Expect(
        system.ApplyBodyControlCommand(command) == ResultCode::InvalidData,
        "a later servo output failure must be reported");
    passed &= Expect(
        timer.compare[0] == 1470U && timer.compare[1] == 1470U &&
            limited_timer.compare[2] == 1470U && timer.compare[3] == 1570U,
        "a later servo output failure must neutralize all four logical servos");

    if (!passed) {
        return 1;
    }

    printf("DartSystem control-output behavior checks passed.\n");
    return 0;
}
