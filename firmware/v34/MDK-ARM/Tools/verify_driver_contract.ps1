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
$driverRoot = Join-Path $repoRoot "User/Driver"
$keilProject = Join-Path $ProjectRoot "A133P_Dart.uvprojx"
$eideProject = Join-Path $ProjectRoot ".eide/eide.yml"

$headers = @(
    "drv_gpio_pin.h",
    "drv_spi_bus.h",
    "drv_pwm_channel.h",
    "drv_timebase.h",
    "drv_uart_dma.h"
)

$sources = @(
    "drv_gpio_pin.cpp",
    "drv_spi_bus.cpp",
    "drv_pwm_channel.cpp",
    "drv_timebase.cpp",
    "drv_uart_dma.cpp"
)

Assert-True (Test-Path $driverRoot) "Missing User/Driver directory"

foreach ($file in ($headers + $sources)) {
    Assert-True (Test-Path (Join-Path $driverRoot $file)) "Missing User/Driver/$file"
}

$contracts = @{
    "drv_gpio_pin.h" = @(
        "class Class_GpioPin",
        "ResultCode Bind(GPIO_TypeDef* port, uint16_t pin)",
        "bool IsBound() const",
        "ResultCode SetHigh()",
        "ResultCode SetLow()",
        "ResultCode Write(bool high)",
        "ResultCode Read(bool& high) const"
    )
    "drv_spi_bus.h" = @(
        "class Class_SpiBus",
        "ResultCode Bind(SPI_HandleTypeDef* handle)",
        "ResultCode Transfer(const uint8_t* tx, uint8_t* rx, uint16_t length, uint32_t timeout_ms)"
    )
    "drv_pwm_channel.h" = @(
        "class Class_PwmChannel",
        "ResultCode Bind(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t counter_frequency_hz)",
        "ResultCode Start()",
        "ResultCode Stop()",
        "ResultCode SetCompareTicks(uint32_t ticks)",
        "ResultCode SetDutyCycle(float duty_0_to_1)",
        "ResultCode SetPulseWidthUs(uint32_t pulse_width_us)"
    )
    "drv_timebase.h" = @(
        "class Class_Timebase",
        "ResultCode Bind(TIM_HandleTypeDef* timer)",
        "void OnTickIsr()",
        "uint32_t NowMs() const"
    )
    "drv_uart_dma.h" = @(
        "class Class_UartDma",
        "ResultCode Bind(UART_HandleTypeDef* uart)",
        "ResultCode StartReceiveCircular(uint8_t* buffer, uint16_t capacity)",
        "ResultCode ServiceReceive()",
        "ResultCode SendAsync(uint8_t* data, uint16_t length)",
        "void OnRxErrorIsr(uint32_t error_code)",
        "uint32_t Available() const",
        "ResultCode ReadByte(uint8_t& byte)",
        "uint32_t GetRxOverflowCount() const",
        "uint32_t GetRxErrorCount() const",
        "uint32_t GetRxRestartCount() const",
        "uint32_t GetLastRxError() const",
        "uint16_t rx_read_position_",
        "volatile uint32_t rx_overflow_count_",
        "volatile uint32_t rx_error_count_",
        "uint32_t rx_restart_count_",
        "volatile uint32_t rx_last_error_",
        "volatile bool rx_restart_pending_",
        "volatile bool rx_started_"
    )
}

foreach ($entry in $contracts.GetEnumerator()) {
    $content = Read-Text (Join-Path $driverRoot $entry.Key)
    foreach ($needle in $entry.Value) {
        Assert-True ($content.Contains($needle)) "Missing contract '$needle' in $($entry.Key)"
    }
}

$forbiddenDriverTerms = @(
    "hspi1", "htim1", "htim3", "htim5", "huart1",
    "spi.h", "tim.h", "usart.h",
    "SERVO_NEUTRAL_US", "BMI088_REG_CHIP_ID", "BMI088_ACC_CS",
    "HEATER_TARGET_TEMP", "A133P_HEADER", "XTail",
    "Roll", "Pitch", "Yaw",
    "new ", "delete ", "std::vector", "iostream", "dynamic_cast", "typeid", "virtual "
)

foreach ($file in ($headers + $sources)) {
    $content = Read-Text (Join-Path $driverRoot $file)
    foreach ($needle in $forbiddenDriverTerms) {
        Assert-True (-not $content.Contains($needle)) "Forbidden Driver content '$needle' in $file"
    }
}

$uartHeader = Read-Text (Join-Path $driverRoot "drv_uart_dma.h")
$uartSource = Read-Text (Join-Path $driverRoot "drv_uart_dma.cpp")

Assert-True (-not $uartHeader.Contains("GetLastRxSize")) "Class_UartDma must not expose GetLastRxSize as the RX event consumption API"
Assert-True (-not $uartSource.Contains("GetLastRxSize")) "Class_UartDma must not implement GetLastRxSize as the RX event consumption API"
Assert-True (-not $uartHeader.Contains("void OnRxEvent(uint16_t size)")) "Class_UartDma must expose ISR-specific RX event naming"
Assert-True (-not $uartHeader.Contains("StartReceiveToIdle")) "Class_UartDma must retire the Normal-DMA StartReceiveToIdle API"
Assert-True (-not $uartSource.Contains("StartReceiveToIdle")) "Class_UartDma must retire the Normal-DMA StartReceiveToIdle implementation"
Assert-True (-not $uartHeader.Contains("TakeRxEvent")) "Class_UartDma must retire the event-oriented TakeRxEvent API"
Assert-True (-not $uartSource.Contains("TakeRxEvent")) "Class_UartDma must retire the event-oriented TakeRxEvent implementation"
Assert-True (-not $uartHeader.Contains("OnRxEventIsr")) "NDTR-polled circular RX must not expose an RX event callback API"
Assert-True (-not $uartSource.Contains("OnRxEventIsr")) "NDTR-polled circular RX must not implement an RX event callback path"
Assert-True (-not $uartSource.Contains("HAL_UARTEx_ReceiveToIdle_DMA")) "Circular RX must use standard DMA receive without IDLE callbacks"
Assert-True ($uartSource.Contains("HAL_UART_Receive_DMA(uart_, rx_buffer_, rx_capacity_)")) "Circular RX must start through HAL_UART_Receive_DMA"
Assert-True ($uartSource.Contains("__HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT | DMA_IT_TC)")) "NDTR polling must disable DMA HT and TC interrupts"
Assert-True ($uartSource.Contains("__HAL_DMA_GET_COUNTER(uart_->hdmarx)")) "Available must derive the DMA write position from NDTR"
Assert-True (-not $uartHeader.Contains("std::atomic")) "Class_UartDma must use Cortex-M4 aligned volatile counters, not std::atomic"

$forbiddenCircularRxTerms = @(
    "ReadAvailable",
    "RefreshRxProgress",
    "UpdateRxPosition",
    "__get_PRIMASK",
    "__disable_irq",
    "__enable_irq",
    "memcpy(",
    "std::atomic",
    "mutex"
)

foreach ($needle in $forbiddenCircularRxTerms) {
    Assert-True (-not $uartHeader.Contains($needle)) "Forbidden simplified circular RX header term '$needle'"
    Assert-True (-not $uartSource.Contains($needle)) "Forbidden simplified circular RX source term '$needle'"
}

$serviceStart = $uartSource.IndexOf("ResultCode Class_UartDma::ServiceReceive()")
$onErrorStart = $uartSource.IndexOf("void Class_UartDma::OnRxErrorIsr(uint32_t error_code)")
$availableStart = $uartSource.IndexOf("uint32_t Class_UartDma::Available() const")
$readByteStart = $uartSource.IndexOf("ResultCode Class_UartDma::ReadByte(uint8_t& byte)")
$overflowGetterStart = $uartSource.IndexOf("uint32_t Class_UartDma::GetRxOverflowCount() const")
Assert-True ($serviceStart -ge 0) "Missing main-context RX recovery service"
Assert-True (($onErrorStart -ge 0) -and ($onErrorStart -lt $availableStart)) "OnRxErrorIsr must precede Available"
Assert-True (($availableStart -ge 0) -and ($availableStart -lt $readByteStart)) "Available must precede ReadByte"
Assert-True (($readByteStart -ge 0) -and ($readByteStart -lt $overflowGetterStart)) "ReadByte must precede GetRxOverflowCount"

$serviceEnd = $uartSource.IndexOf("ResultCode Class_UartDma::SendAsync", $serviceStart)
$serviceText = $uartSource.Substring($serviceStart, $serviceEnd - $serviceStart)
$onErrorText = $uartSource.Substring($onErrorStart, $availableStart - $onErrorStart)
$availableText = $uartSource.Substring($availableStart, $readByteStart - $availableStart)
$readByteText = $uartSource.Substring($readByteStart, $overflowGetterStart - $readByteStart)

Assert-True ($serviceText.Contains("HAL_UART_AbortReceive(uart_)")) "RX recovery must abort the faulted HAL receive in main context"
Assert-True ($serviceText.Contains("StartHalReceive()")) "RX recovery must restart circular DMA in main context"
Assert-True ($serviceText.Contains("rx_restart_pending_ = true")) "Failed RX recovery must remain pending"
Assert-True ($serviceText.Contains("++rx_restart_count_")) "Successful RX recovery must be counted"
Assert-True ($serviceText.Contains("return ResultCode::InvalidData")) "Successful recovery must report the stream discontinuity"

Assert-True ($onErrorText.Contains("rx_last_error_ = error_code")) "UART error ISR must retain the HAL error flags"
Assert-True ($onErrorText.Contains("++rx_error_count_")) "UART error ISR must count errors"
Assert-True ($onErrorText.Contains("HAL_UART_ERROR_ORE")) "UART error ISR must recognize overrun"
Assert-True ($onErrorText.Contains("rx_restart_pending_ = true")) "UART error ISR must request main-context recovery"
Assert-True (-not $onErrorText.Contains("HAL_UART_AbortReceive")) "UART error ISR must not abort receive"
Assert-True (-not $onErrorText.Contains("StartHalReceive")) "UART error ISR must not restart DMA"

Assert-True ($availableText.Contains("GetDmaWritePosition()")) "Available must poll the DMA write position"
Assert-True ($availableText.Contains("write_position - rx_read_position_")) "Available must compute the forward modular distance"
Assert-True ($availableText.Contains("rx_capacity_ - rx_read_position_ + write_position")) "Available must handle DMA ring wrap"

Assert-True ($readByteText.Contains("byte = rx_buffer_[rx_read_position_]")) "ReadByte must consume from the current ring position"
Assert-True ($readByteText.Contains("++rx_read_position_")) "ReadByte must advance the ring read position"
Assert-True ($readByteText.Contains("rx_read_position_ = 0U")) "ReadByte must wrap the ring read position"

$gxx = Get-Command g++.exe -ErrorAction SilentlyContinue
Assert-True ($null -ne $gxx) "g++.exe is required for the Class_UartDma circular RX behavior test"

$testSource = Join-Path $PSScriptRoot "test_uart_dma_circular.cpp"
$testSupport = Join-Path $PSScriptRoot "TestSupport"
$testBinary = Join-Path ([System.IO.Path]::GetTempPath()) "a133p_uart_dma_circular_test.exe"

Assert-True (Test-Path $testSource) "Missing UART circular RX behavior test"
Assert-True (Test-Path (Join-Path $testSupport "stm32f4xx_hal.h")) "Missing UART circular RX HAL test double"

& $gxx.Source `
    -std=c++11 -Wall -Wextra -Werror `
    -I $testSupport `
    -I (Join-Path $repoRoot "User") `
    $testSource `
    (Join-Path $driverRoot "drv_uart_dma.cpp") `
    -o $testBinary
Assert-True ($LASTEXITCODE -eq 0) "Failed to compile Class_UartDma circular RX behavior test"

& $testBinary
Assert-True ($LASTEXITCODE -eq 0) "Class_UartDma circular RX behavior test failed"
Remove-Item -LiteralPath $testBinary -Force -ErrorAction SilentlyContinue

$keilText = Read-Text $keilProject
$eideText = Read-Text $eideProject

Assert-True ($keilText.Contains("../User/Driver")) "Keil include path missing ../User/Driver"
Assert-True ($keilText.Contains("<GroupName>Application/User/Driver</GroupName>")) "Keil Driver group missing"

foreach ($file in ($headers + $sources)) {
    Assert-True ($keilText.Contains("../User/Driver/$file")) "Keil project missing $file"
    Assert-True ($eideText.Contains("../User/Driver/$file")) "EIDE project missing $file"
}

Assert-True ($eideText.Contains("../User/Driver")) "EIDE include path missing ../User/Driver"

Write-Host "Driver contract checks passed."

