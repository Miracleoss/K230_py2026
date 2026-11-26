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
$deviceRoot = Join-Path $repoRoot "User/Device"
$keilProject = Join-Path $ProjectRoot "A133P_Dart.uvprojx"
$eideProject = Join-Path $ProjectRoot ".eide/eide.yml"

$headers = @(
    "dev_bmi088.h",
    "dev_servo.h",
    "dev_imu_heater.h"
)

$sources = @(
    "dev_bmi088.cpp",
    "dev_servo.cpp",
    "dev_imu_heater.cpp"
)

Assert-True (Test-Path $deviceRoot) "Missing User/Device directory"

foreach ($file in ($headers + $sources)) {
    Assert-True (Test-Path (Join-Path $deviceRoot $file)) "Missing User/Device/$file"
}

$contracts = @{
    "dev_bmi088.h" = @(
        "class Class_Bmi088",
        "struct Bmi088RawSample",
        "struct Bmi088ProbeTrace",
        "typedef void (*Bmi088DelayMsFunction)(uint32_t delay_ms)",
        "ResultCode Bind(Class_SpiBus* spi_bus, Class_GpioPin* acc_cs, Class_GpioPin* gyro_cs)",
        "ResultCode Probe(uint8_t& acc_chip_id, uint8_t& gyro_chip_id)",
        "ResultCode Initialize(Bmi088DelayMsFunction delay_ms)",
        "ResultCode ReadRawSample(Bmi088RawSample& sample)",
        "ResultCode ProbeWithTrace(uint8_t& acc_chip_id, uint8_t& gyro_chip_id, Bmi088ProbeTrace& trace)",
        "bool IsBound() const",
        "bool IsInitialized() const"
    )
    "dev_servo.h" = @(
        "class Class_Servo",
        "ResultCode Bind(Class_PwmChannel* pwm, const ServoConfig* config)",
        "bool IsBound() const",
        "ResultCode SetDelta(float delta)",
        "Class_PwmChannel* pwm_",
        "const ServoConfig* config_"
    )
    "dev_imu_heater.h" = @(
        "class Class_ImuHeater",
        "ResultCode Bind(Class_PwmChannel* pwm)",
        "bool IsBound() const",
        "Class_PwmChannel* pwm_"
    )
}

foreach ($entry in $contracts.GetEnumerator()) {
    $content = Read-Text (Join-Path $deviceRoot $entry.Key)
    foreach ($needle in $entry.Value) {
        Assert-True ($content.Contains($needle)) "Missing contract '$needle' in $($entry.Key)"
    }
}

$forbiddenDeviceTerms = @(
    "SPI_HandleTypeDef", "TIM_HandleTypeDef", "UART_HandleTypeDef", "GPIO_TypeDef",
    "HAL_SPI_", "HAL_TIM_", "HAL_GPIO_", "HAL_UART_",
    "BMI088_REG",
    "SetDeflection", "SetAngle", "SetNeutral", "Enable(", "Disable(", "Start(",
    "1500", "2500", "500U",
    "PID", "Kp", "Ki", "Kd", "integrator",
    "HostCommunication", "Protocol", "FlightController", "XTailMixer",
    "new ", "delete ", "std::", "namespace ", "virtual "
)

foreach ($file in ($headers + $sources)) {
    $content = Read-Text (Join-Path $deviceRoot $file)
    foreach ($needle in $forbiddenDeviceTerms) {
        Assert-True (-not $content.Contains($needle)) "Forbidden Device content '$needle' in $file"
    }
}

$servoSource = Read-Text (Join-Path $deviceRoot "dev_servo.cpp")
Assert-True ($servoSource.Contains("pwm_->SetPulseWidthUs(")) "Servo must output through Class_PwmChannel::SetPulseWidthUs"
Assert-True (-not $servoSource.Contains("HAL_")) "Servo Device must not call HAL"

Assert-True (-not (Test-Path (Join-Path $deviceRoot "class_host_communication.h"))) "HostCommunication header must not be created in Stage 2.3"
Assert-True (-not (Test-Path (Join-Path $deviceRoot "class_host_communication.cpp"))) "HostCommunication source must not be created in Stage 2.3"

$keilText = Read-Text $keilProject
$eideText = Read-Text $eideProject

Assert-True ($keilText.Contains("../User/Device")) "Keil include path missing ../User/Device"
Assert-True ($keilText.Contains("<GroupName>Application/User/Device</GroupName>")) "Keil Device group missing"

foreach ($file in ($headers + $sources)) {
    Assert-True ($keilText.Contains("../User/Device/$file")) "Keil project missing $file"
    Assert-True ($eideText.Contains("../User/Device/$file")) "EIDE project missing $file"
}

Assert-True ($eideText.Contains("../User/Device")) "EIDE include path missing ../User/Device"

Write-Host "Device contract checks passed."

$gxx = Get-Command g++.exe -ErrorAction Stop
$testSupport = Join-Path $ProjectRoot "Tools/TestSupport"
$testSource = Join-Path $ProjectRoot "Tools/test_bmi088_device.cpp"
$testBinary = Join-Path ([System.IO.Path]::GetTempPath()) "a133p_bmi088_device.exe"

try {
    & $gxx.Source `
        -std=c++11 -Wall -Wextra -Werror `
        -I $testSupport `
        -I (Join-Path $repoRoot "User") `
        $testSource `
        (Join-Path $deviceRoot "dev_bmi088.cpp") `
        (Join-Path $repoRoot "User/Driver/drv_gpio_pin.cpp") `
        (Join-Path $repoRoot "User/Driver/drv_spi_bus.cpp") `
        -o $testBinary
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    & $testBinary
    exit $LASTEXITCODE
}
finally {
    Remove-Item -LiteralPath $testBinary -Force -ErrorAction SilentlyContinue
}

