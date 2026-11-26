param(
    [string]$ProjectRoot = (Resolve-Path "$PSScriptRoot/..").Path
)

$ErrorActionPreference = "Stop"

function Assert-True {
    param(
        [bool]$Condition,
        [string]$Message
    )

    if (-not $Condition) {
        throw $Message
    }
}

function Read-Text {
    param([string]$Path)

    return Get-Content -Raw -Encoding UTF8 $Path
}

$repoRoot = (Resolve-Path "$ProjectRoot/..").Path
$algorithmRoot = Join-Path $repoRoot "User/Algorithm"
$keilProject = Join-Path $ProjectRoot "A133P_Dart.uvprojx"
$eideProject = Join-Path $ProjectRoot ".eide/eide.yml"

$headers = @(
    "alg_xtail_mixer.h",
    "alg_attitude_estimator.h"
)

$sources = @(
    "alg_xtail_mixer.cpp",
    "alg_attitude_estimator.cpp"
)

Assert-True (Test-Path $algorithmRoot) "Missing User/Algorithm directory"

foreach ($file in ($headers + $sources)) {
    Assert-True (Test-Path (Join-Path $algorithmRoot $file)) "Missing User/Algorithm/$file"
}

$contracts = @{
    "alg_xtail_mixer.h" = @(
        "class Class_XTailMixer",
        "Class_XTailMixer()",
        "ResultCode Mix(const BodyControlCommand& input, SurfaceCommand& output) const"
    )
    "alg_attitude_estimator.h" = @(
        "class Class_AttitudeEstimator",
        "Class_AttitudeEstimator()",
        "ResultCode Reset()",
        "ResultCode Update(const ImuData& imu, AttitudeState& attitude)"
    )
}

foreach ($entry in $contracts.GetEnumerator()) {
    $content = Read-Text (Join-Path $algorithmRoot $entry.Key)
    foreach ($needle in $entry.Value) {
        Assert-True ($content.Contains($needle)) "Missing contract '$needle' in $($entry.Key)"
    }
}

$forbiddenAlgorithmTerms = @(
    "HAL_", "SPI_HandleTypeDef", "TIM_HandleTypeDef", "UART_HandleTypeDef", "GPIO_TypeDef",
    "Driver/", "Device/", "Application/", "Task/",
    "Class_SpiBus", "Class_PwmChannel", "Class_GpioPin",
    "Class_Bmi088", "Class_Servo", "Class_ImuHeater",
    "Bind(", "IsBound(",
    "ReadRegister", "WriteRegister", "BMI088_REG", "CHIP_ID",
    "TIM_CHANNEL", "PWM", "pulse", "1500", "2500", "500U",
    "Mahony", "Madgwick", "EKF", "Complementary", "Kalman",
    "quaternion", "magnetometer", "gyro bias",
    "PID", "Filter", "FlightController", "HostCommunication", "Protocol",
    "new ", "delete ", "std::", "namespace ", "virtual "
)

foreach ($file in ($headers + $sources)) {
    $content = Read-Text (Join-Path $algorithmRoot $file)
    foreach ($needle in $forbiddenAlgorithmTerms) {
        Assert-True (-not $content.Contains($needle)) "Forbidden Algorithm content '$needle' in $file"
    }
}

$xtailSource = Read-Text (Join-Path $algorithmRoot "alg_xtail_mixer.cpp")
$estimatorSource = Read-Text (Join-Path $algorithmRoot "alg_attitude_estimator.cpp")

$requiredXtailTerms = @(
    "ResultCode Class_XTailMixer::Mix(",
    "output.delta1 = input.roll + input.yaw + input.pitch;",
    "output.delta2 = input.roll + input.yaw - input.pitch;",
    "output.delta3 = input.roll - input.yaw - input.pitch;",
    "output.delta4 = input.roll - input.yaw + input.pitch;",
    "if (max_abs > 1.0f)",
    "output.delta1 /= max_abs;",
    "output.delta2 /= max_abs;",
    "output.delta3 /= max_abs;",
    "output.delta4 /= max_abs;"
)

foreach ($needle in $requiredXtailTerms) {
    Assert-True ($xtailSource.Contains($needle)) "Missing Stage 3D.1 XTail Mixer term '$needle'"
}

Assert-True (-not $xtailSource.Contains("clamp")) "XTail Mixer must use uniform scaling, not individual clamp"
Assert-True (-not $xtailSource.Contains("Clamp")) "XTail Mixer must use uniform scaling, not individual Clamp"
Assert-True (-not $estimatorSource.Contains("Reset(")) "Class_AttitudeEstimator::Reset must not be implemented in Stage 2.4"
Assert-True (-not $estimatorSource.Contains("Update(")) "Class_AttitudeEstimator::Update must not be implemented in Stage 2.4"

$prematureFiles = @(
    "class_pid.h", "class_pid.cpp",
    "class_filter.h", "class_filter.cpp",
    "class_flight_controller.h", "class_flight_controller.cpp",
    "class_quaternion.h", "class_quaternion.cpp",
    "class_matrix.h", "class_matrix.cpp"
)

foreach ($file in $prematureFiles) {
    Assert-True (-not (Test-Path (Join-Path $algorithmRoot $file))) "Premature Algorithm file exists: $file"
}

$keilText = Read-Text $keilProject
$eideText = Read-Text $eideProject

Assert-True ($keilText.Contains("../User/Algorithm")) "Keil include path missing ../User/Algorithm"
Assert-True ($keilText.Contains("<GroupName>Application/User/Algorithm</GroupName>")) "Keil Algorithm group missing"

foreach ($file in ($headers + $sources)) {
    Assert-True ($keilText.Contains("../User/Algorithm/$file")) "Keil project missing $file"
    Assert-True ($eideText.Contains("../User/Algorithm/$file")) "EIDE project missing $file"
}

Assert-True ($eideText.Contains("../User/Algorithm")) "EIDE include path missing ../User/Algorithm"

Write-Host "Algorithm contract checks passed."

