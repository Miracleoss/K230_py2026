#include <limits>
#include <stdio.h>

#include "Algorithm/alg_closed_loop_controller.h"

static bool Expect(bool condition, const char* message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        return false;
    }
    return true;
}

static bool ExpectNear(float actual, float expected, float tolerance, const char* message)
{
    const float difference = actual >= expected ? actual - expected : expected - actual;
    if (difference > tolerance) {
        fprintf(
            stderr,
            "FAIL: %s (actual=%.6f expected=%.6f)\n",
            message,
            static_cast<double>(actual),
            static_cast<double>(expected));
        return false;
    }
    return true;
}

static ImuData MakeValidImu(uint32_t timestamp_ms, uint32_t sequence)
{
    ImuData imu;
    imu.sample_timestamp_ms = timestamp_ms;
    imu.data_ready_seq = sequence;
    imu.valid = true;
    return imu;
}

static ClosedLoopControllerConfig MakeTestConfig()
{
    ClosedLoopControllerConfig config = MakeDefaultClosedLoopControllerConfig();
    config.n_y_c_min = -10.0f;
    config.n_y_c_max = 10.0f;
    config.n_z_c_min = -10.0f;
    config.n_z_c_max = 10.0f;
    config.overload_y_pi.kp = 2.0f;
    config.overload_y_pi.ki = 1.0f;
    config.overload_y_pi.integral_min = -5.0f;
    config.overload_y_pi.integral_max = 5.0f;
    config.overload_y_pi.output_min = -5.0f;
    config.overload_y_pi.output_max = 5.0f;
    config.overload_z_pi = config.overload_y_pi;
    config.rate_r_pid.kp = 0.25f;
    config.rate_r_pid.ki = 0.0f;
    config.rate_r_pid.kd = 0.0f;
    config.rate_r_pid.integral_min = -2.0f;
    config.rate_r_pid.integral_max = 2.0f;
    config.rate_r_pid.output_min = -1.0f;
    config.rate_r_pid.output_max = 1.0f;
    config.rate_p_pid = config.rate_r_pid;
    config.rate_y_pid = config.rate_r_pid;
    config.dt_min_s = 0.001f;
    config.dt_max_s = 0.020f;
    return config;
}

static bool TestZeroInputProducesNeutralOutput()
{
    Class_ClosedLoopController controller;
    ClosedLoopControlOutput output;
    const ImuData imu = MakeValidImu(100U, 1U);

    bool passed = true;
    passed &= Expect(
        controller.Update(0.0f, 0.0f, imu, 0.005f, output) == ResultCode::Ok,
        "zero input with a valid new IMU sample must be accepted");
    passed &= Expect(output.valid, "accepted controller output must be valid");
    passed &= ExpectNear(output.n_y_meas, 0.0f, 0.000001f, "zero f_y must produce zero n_y_meas");
    passed &= ExpectNear(output.n_z_meas, 0.0f, 0.000001f, "zero f_z must produce zero n_z_meas");
    passed &= ExpectNear(output.omega_x_c, 0.0f, 0.000001f, "omega_x_c must remain zero");
    passed &= ExpectNear(output.omega_y_c, 0.0f, 0.000001f, "zero-gain Y PI must produce zero rate command");
    passed &= ExpectNear(output.omega_z_c, 0.0f, 0.000001f, "zero-gain Z PI must produce zero rate command");
    passed &= ExpectNear(output.ctrl_R, 0.5f, 0.000001f, "zero roll effort must map to ctrl_R midpoint");
    passed &= ExpectNear(output.ctrl_P, 0.5f, 0.000001f, "zero pitch effort must map to ctrl_P midpoint");
    passed &= ExpectNear(output.ctrl_Y, 0.5f, 0.000001f, "zero yaw effort must map to ctrl_Y midpoint");
    return passed;
}

static bool TestPositiveOverloadProducesPositivePitchRateCommand()
{
    Class_ClosedLoopController controller(MakeTestConfig());
    ClosedLoopControlOutput output;
    const ImuData imu = MakeValidImu(200U, 2U);

    bool passed = true;
    passed &= Expect(
        controller.Update(1.0f, 0.0f, imu, 0.005f, output) == ResultCode::Ok,
        "positive n_y_c must be accepted");
    passed &= Expect(output.omega_z_c > 0.0f, "positive upward overload error must produce positive omega_z_c");
    passed &= ExpectNear(output.omega_x_c, 0.0f, 0.000001f, "overload PI must not command roll rate");
    passed &= ExpectNear(output.omega_y_c, 0.0f, 0.000001f, "pure pitch overload must not command yaw rate");
    passed &= Expect(output.ctrl_P > 0.5f, "positive pitch-rate tracking error must produce ctrl_P above midpoint");
    return passed;
}

static bool TestYawOverloadMapsWithKinematicSignToYawRateAndControl()
{
    Class_ClosedLoopController right_controller(MakeTestConfig());
    ClosedLoopControlOutput right_output;
    const ImuData right_imu = MakeValidImu(210U, 20U);

    bool passed = true;
    passed &= Expect(
        right_controller.Update(0.0f, 1.0f, right_imu, 0.005f, right_output) == ResultCode::Ok,
        "positive n_z_c (+Z rightward overload) must be accepted");
    passed &= Expect(
        right_output.omega_y_c < 0.0f,
        "positive +Z overload error must command negative omega_y_c (nose-right yaw)");
    passed &= ExpectNear(right_output.omega_x_c, 0.0f, 0.000001f, "yaw overload PI must not command roll rate");
    passed &= ExpectNear(right_output.omega_z_c, 0.0f, 0.000001f, "pure yaw overload must not command pitch rate");
    passed &= Expect(
        right_output.ctrl_Y < 0.5f,
        "negative yaw-rate command with zero gyro_y must drive ctrl_Y below midpoint");

    Class_ClosedLoopController left_controller(MakeTestConfig());
    ClosedLoopControlOutput left_output;
    const ImuData left_imu = MakeValidImu(220U, 21U);
    passed &= Expect(
        left_controller.Update(0.0f, -1.0f, left_imu, 0.005f, left_output) == ResultCode::Ok,
        "negative n_z_c (-Z leftward overload) must be accepted");
    passed &= Expect(
        left_output.omega_y_c > 0.0f,
        "negative -Z overload error must command positive omega_y_c (nose-left yaw)");
    passed &= Expect(
        left_output.ctrl_Y > 0.5f,
        "positive yaw-rate command with zero gyro_y must drive ctrl_Y above midpoint");
    return passed;
}

static bool TestPositiveRateErrorAndSaturation()
{
    ClosedLoopControllerConfig config = MakeTestConfig();
    config.overload_y_pi.kp = 0.0f;
    config.overload_y_pi.ki = 0.0f;
    config.rate_p_pid.kp = 0.5f;

    Class_ClosedLoopController rate_controller(config);
    ClosedLoopControlOutput rate_output;
    ImuData rate_imu = MakeValidImu(300U, 3U);
    rate_imu.gyro_z_radps = -1.0f;

    bool passed = true;
    passed &= Expect(
        rate_controller.Update(0.0f, 0.0f, rate_imu, 0.005f, rate_output) == ResultCode::Ok,
        "positive pitch-rate error sample must be accepted");
    passed &= ExpectNear(rate_output.ctrl_P, 0.75f, 0.000001f, "positive unit rate error must map to configured pitch effort");

    config.overload_y_pi.kp = 5.0f;
    config.rate_p_pid.kp = 5.0f;
    Class_ClosedLoopController saturation_controller(config);
    ClosedLoopControlOutput saturation_output;
    const ImuData saturation_imu = MakeValidImu(301U, 4U);
    passed &= Expect(
        saturation_controller.Update(10.0f, 0.0f, saturation_imu, 0.005f, saturation_output) == ResultCode::Ok,
        "large overload command must still produce a bounded valid output");
    passed &= ExpectNear(saturation_output.ctrl_P, 1.0f, 0.000001f, "positive pitch effort must saturate at 1.0");
    passed &= Expect(
        saturation_output.ctrl_R >= 0.0f && saturation_output.ctrl_R <= 1.0f &&
            saturation_output.ctrl_P >= 0.0f && saturation_output.ctrl_P <= 1.0f &&
            saturation_output.ctrl_Y >= 0.0f && saturation_output.ctrl_Y <= 1.0f,
        "all control outputs must remain in the unit interval");
    return passed;
}

static bool TestInvalidInputsResetToNeutral()
{
    const ClosedLoopControllerConfig config = MakeTestConfig();
    bool passed = true;

    Class_ClosedLoopController invalid_imu_controller(config);
    ClosedLoopControlOutput invalid_imu_output;
    ImuData invalid_imu = MakeValidImu(400U, 5U);
    invalid_imu.valid = false;
    passed &= Expect(
        invalid_imu_controller.Update(1.0f, 0.0f, invalid_imu, 0.005f, invalid_imu_output) ==
            ResultCode::InvalidData,
        "invalid IMU data must be rejected");
    passed &= Expect(!invalid_imu_output.valid, "rejected IMU data must produce invalid output");
    passed &= ExpectNear(invalid_imu_output.ctrl_R, 0.5f, 0.000001f, "invalid IMU must neutralize ctrl_R");
    passed &= ExpectNear(invalid_imu_output.ctrl_P, 0.5f, 0.000001f, "invalid IMU must neutralize ctrl_P");
    passed &= ExpectNear(invalid_imu_output.ctrl_Y, 0.5f, 0.000001f, "invalid IMU must neutralize ctrl_Y");

    Class_ClosedLoopController zero_dt_controller(config);
    ClosedLoopControlOutput zero_dt_output;
    const ImuData zero_dt_imu = MakeValidImu(401U, 6U);
    passed &= Expect(
        zero_dt_controller.Update(0.0f, 0.0f, zero_dt_imu, 0.0f, zero_dt_output) ==
            ResultCode::InvalidData,
        "zero dt must be rejected");
    passed &= Expect(!zero_dt_output.valid, "zero dt must produce invalid output");
    passed &= ExpectNear(zero_dt_output.ctrl_P, 0.5f, 0.000001f, "zero dt must produce neutral output");

    Class_ClosedLoopController large_dt_controller(config);
    ClosedLoopControlOutput large_dt_output;
    const ImuData large_dt_imu = MakeValidImu(402U, 7U);
    passed &= Expect(
        large_dt_controller.Update(0.0f, 0.0f, large_dt_imu, config.dt_max_s + 0.001f, large_dt_output) ==
            ResultCode::InvalidData,
        "dt above configured maximum must be rejected");
    passed &= Expect(!large_dt_output.valid, "oversized dt must produce invalid output");
    passed &= ExpectNear(large_dt_output.ctrl_P, 0.5f, 0.000001f, "oversized dt must produce neutral output");

    Class_ClosedLoopController nan_controller(config);
    ClosedLoopControlOutput nan_output;
    const ImuData nan_imu = MakeValidImu(403U, 8U);
    passed &= Expect(
        nan_controller.Update(
            std::numeric_limits<float>::quiet_NaN(),
            0.0f,
            nan_imu,
            0.005f,
            nan_output) == ResultCode::InvalidData,
        "non-finite command input must be rejected");
    passed &= ExpectNear(nan_output.ctrl_P, 0.5f, 0.000001f, "non-finite command must produce neutral output");
    return passed;
}

static bool TestRepeatedTimestampDoesNotAdvanceIntegral()
{
    ClosedLoopControllerConfig config = MakeTestConfig();
    config.overload_y_pi.kp = 0.0f;
    config.overload_y_pi.ki = 1.0f;
    config.rate_p_pid.kp = 0.0f;

    Class_ClosedLoopController controller(config);
    ClosedLoopControlOutput first_output;
    const ImuData first_imu = MakeValidImu(500U, 9U);
    bool passed = true;
    passed &= Expect(
        controller.Update(1.0f, 0.0f, first_imu, 0.010f, first_output) == ResultCode::Ok,
        "first timestamp must be accepted");
    passed &= ExpectNear(first_output.omega_z_c, 0.010f, 0.000001f, "first sample must advance PI integral once");

    ClosedLoopControlOutput repeated_output;
    const ImuData repeated_timestamp_imu = MakeValidImu(500U, 9U);
    passed &= Expect(
        controller.Update(1.0f, 0.0f, repeated_timestamp_imu, 0.010f, repeated_output) ==
            ResultCode::InvalidData,
        "repeated timestamp and sequence must be rejected");
    passed &= Expect(!repeated_output.valid, "repeated timestamp must invalidate output");
    passed &= ExpectNear(repeated_output.ctrl_P, 0.5f, 0.000001f, "repeated timestamp must neutralize output");

    ClosedLoopControlOutput after_reset_output;
    const ImuData after_reset_imu = MakeValidImu(510U, 11U);
    passed &= Expect(
        controller.Update(1.0f, 0.0f, after_reset_imu, 0.010f, after_reset_output) == ResultCode::Ok,
        "new sample after repeated timestamp reset must be accepted");
    passed &= ExpectNear(
        after_reset_output.omega_z_c,
        0.010f,
        0.000001f,
        "rejected duplicate sample must not retain or advance PI integral");
    return passed;
}

static bool TestNegativeGainAntiWindupUnwindsCorrectly()
{
    ClosedLoopControllerConfig config = MakeTestConfig();
    config.overload_z_pi.kp = 0.0f;
    config.overload_z_pi.ki = 0.0f;
    config.rate_y_pid.kp = 0.0f;
    config.rate_y_pid.ki = -1.0f;
    config.rate_y_pid.kd = 0.0f;
    config.rate_y_pid.integral_min = -1.0f;
    config.rate_y_pid.integral_max = 1.0f;
    config.rate_y_pid.output_min = -0.125f;
    config.rate_y_pid.output_max = 0.125f;

    Class_ClosedLoopController controller(config);
    ClosedLoopControlOutput output;
    bool passed = true;
    const float dt_s = 0.015625f;

    for (uint32_t step = 0U; step < 20U; ++step) {
        ImuData imu = MakeValidImu(600U + step, 30U + step);
        imu.gyro_y_radps = 1.0f;
        passed &= Expect(
            controller.Update(0.0f, 0.0f, imu, dt_s, output) == ResultCode::Ok,
            "negative-Ki saturation step must succeed");
    }
    passed &= ExpectNear(
        output.ctrl_Y,
        0.5625f,
        0.000001f,
        "negative Ki with negative error must clamp at upper output bound");

    ImuData unwind_imu = MakeValidImu(630U, 60U);
    unwind_imu.gyro_y_radps = -1.0f;
    passed &= Expect(
        controller.Update(0.0f, 0.0f, unwind_imu, dt_s, output) == ResultCode::Ok,
        "negative-Ki unwind step must succeed");
    passed &= ExpectNear(
        output.ctrl_Y,
        0.5546875f,
        0.000001f,
        "sign-aware anti-windup must freeze at saturation boundary and unwind immediately");
    return passed;
}

int main()
{
    bool passed = true;
    passed &= TestZeroInputProducesNeutralOutput();
    passed &= TestPositiveOverloadProducesPositivePitchRateCommand();
    passed &= TestYawOverloadMapsWithKinematicSignToYawRateAndControl();
    passed &= TestPositiveRateErrorAndSaturation();
    passed &= TestInvalidInputsResetToNeutral();
    passed &= TestNegativeGainAntiWindupUnwindsCorrectly();
    passed &= TestRepeatedTimestampDoesNotAdvanceIntegral();
    {
        Class_ClosedLoopController default_controller;
        ClosedLoopControlOutput default_output;
        ImuData first_imu = MakeValidImu(10U, 100U);
        first_imu.sample_timestamp_us = 10000ULL;
        ImuData jitter_imu = MakeValidImu(11U, 101U);
        jitter_imu.sample_timestamp_us = 10995ULL;
        ImuData min_bound_imu = MakeValidImu(11U, 102U);
        min_bound_imu.sample_timestamp_us = 11495ULL;
        ImuData too_short_imu = MakeValidImu(11U, 103U);
        too_short_imu.sample_timestamp_us = 11985ULL;

        passed &= Expect(
            default_controller.Update(0.0f, 0.0f, first_imu, 0.001f, default_output) == ResultCode::Ok,
            "default config must accept 1 ms nominal initial dt");
        passed &= Expect(
            default_controller.Update(0.0f, 0.0f, jitter_imu, 0.000995f, default_output) == ResultCode::Ok,
            "default config must accept sub-millisecond 1 kHz polling jitter (995 us)");
        passed &= Expect(
            default_controller.Update(0.0f, 0.0f, min_bound_imu, 0.0005f, default_output) == ResultCode::Ok,
            "default config must accept the 500 us lower dt bound");
        passed &= Expect(
            default_controller.Update(0.0f, 0.0f, too_short_imu, 0.00049f, default_output) == ResultCode::InvalidData,
            "default config must reject dt below 500 us");
    }

    if (!passed) {
        return 1;
    }

    printf("Closed-loop controller behavior checks passed.\n");
    return 0;
}
