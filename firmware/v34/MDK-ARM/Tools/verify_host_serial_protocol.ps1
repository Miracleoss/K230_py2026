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

$repoRoot = (Resolve-Path "$ProjectRoot/..").Path
$userRoot = Join-Path $repoRoot "User"
$protocolHeader = Join-Path $userRoot "Protocol/host_serial_protocol.h"
$protocolSource = Join-Path $userRoot "Protocol/host_serial_protocol.cpp"
$deviceHeader = Join-Path $userRoot "Device/dev_host_serial.h"
$deviceSource = Join-Path $userRoot "Device/dev_host_serial.cpp"
$taskHeader = Join-Path $userRoot "Task/task_bridge.h"
$taskSource = Join-Path $userRoot "Task/task_bridge.cpp"
$projectFile = Join-Path $ProjectRoot "A133P_Dart.uvprojx"
$eideFile = Join-Path $ProjectRoot ".eide/eide.yml"

foreach ($path in @($protocolHeader, $protocolSource, $deviceHeader, $deviceSource, $taskHeader, $taskSource, $projectFile, $eideFile)) {
    Assert-True (Test-Path $path) "Missing protocol integration file: $path"
}

$taskHeaderText = Get-Content -Raw -Encoding UTF8 $taskHeader
$taskSourceText = Get-Content -Raw -Encoding UTF8 $taskSource
$protocolHeaderText = Get-Content -Raw -Encoding UTF8 $protocolHeader
$protocolSourceText = Get-Content -Raw -Encoding UTF8 $protocolSource
$deviceHeaderText = Get-Content -Raw -Encoding UTF8 $deviceHeader
$deviceSourceText = Get-Content -Raw -Encoding UTF8 $deviceSource
Assert-True ($taskHeaderText.Contains("Task_TakeLatestHostOverload")) "Task bridge must expose the latest physical overload command"
Assert-True ($taskHeaderText.Contains("Task_SendImuData")) "Task bridge must expose IMU transmission"
Assert-True ($taskHeaderText.Contains("Task_SendVofaImuData")) "Task bridge must expose the VOFA JustFloat test entry"
Assert-True ($taskSourceText.Contains("imu_telemetry_mode = kTaskImuTelemetryHostProtocol")) "Host protocol must remain the default telemetry mode"
Assert-True ($taskSourceText.Contains("imu_telemetry_mode == kTaskImuTelemetryVofaJustFloat")) "Task must select VOFA mode through the Watch-visible mode"
Assert-True ($taskSourceText.Contains("static Class_HostSerial host_serial")) "Task bridge must own the host serial device"
Assert-True ($taskSourceText.Contains("host_serial.Bind(&host_uart)")) "Task init must bind the host serial device"
Assert-True ($taskSourceText.Contains("host_serial.Poll()")) "Task loop must poll protocol bytes outside interrupt context"
Assert-True ($deviceHeaderText.Contains("SendVofaJustFloatData")) "Host serial device must expose VOFA JustFloat sending"
Assert-True ($deviceHeaderText.Contains("vofa_tx_buffers_[2][kVofaJustFloatFrameSize]")) "VOFA sending must have two dedicated TX buffers"
Assert-True ($deviceSourceText.Contains("VofaProtocolEncodeJustFloatFrame")) "VOFA sending must use the protocol encoder"
Assert-True ($deviceSourceText.Contains("uart_->SendAsync(frame, kVofaJustFloatFrameSize)")) "VOFA sending must remain on UART DMA"
Assert-True ($protocolHeaderText.Contains("kVofaJustFloatFrameSize")) "Protocol header must define the fixed JustFloat frame size"
Assert-True ($protocolHeaderText.Contains("VofaProtocolEncodeJustFloatFrame")) "Protocol header must expose the JustFloat encoder"
Assert-True ($protocolSourceText.Contains("kVofaJustFloatTail3 = 0x7FU")) "Protocol source must retain the JustFloat sync tail"

$callbackMatch = [regex]::Match(
    $taskSourceText,
    '(?s)extern "C" void HAL_UART_ErrorCallback\s*\([^)]*\)\s*\{(?<body>.*?)\r?\n\}'
)
Assert-True $callbackMatch.Success "Missing UART error callback"
$callbackBody = $callbackMatch.Groups["body"].Value
Assert-True ($callbackBody.Contains("host_uart.OnRxErrorIsr(huart->ErrorCode)")) "UART error callback must notify the DMA wrapper"
Assert-True (-not $callbackBody.Contains("host_serial")) "UART error callback must not parse protocol data"
Assert-True (-not $callbackBody.Contains("HAL_UART_AbortReceive")) "UART error callback must defer HAL recovery to the main loop"
Assert-True (-not $taskSourceText.Contains("HAL_UARTEx_RxEventCallback")) "Task must not use an RX event callback with NDTR polling"

[xml]$projectXml = Get-Content -Raw -Encoding UTF8 $projectFile
$projectPaths = @($projectXml.SelectNodes("//FilePath") | ForEach-Object { $_.'#text'.Replace("\", "/") })
foreach ($requiredPath in @(
    "../User/Protocol/host_serial_protocol.h",
    "../User/Protocol/host_serial_protocol.cpp",
    "../User/Device/dev_host_serial.h",
    "../User/Device/dev_host_serial.cpp"
)) {
    Assert-True ($projectPaths -contains $requiredPath) "Keil project missing $requiredPath"
}

$eideText = Get-Content -Raw -Encoding UTF8 $eideFile
foreach ($requiredPath in @(
    "../User/Common/host_overload_command.h",
    "../User/Protocol/host_serial_protocol.h",
    "../User/Protocol/host_serial_protocol.cpp",
    "../User/Device/dev_host_serial.h",
    "../User/Device/dev_host_serial.cpp"
)) {
    Assert-True ($eideText.Contains("path: $requiredPath")) "EIDE project missing $requiredPath"
}

$gxx = Get-Command g++.exe -ErrorAction SilentlyContinue
Assert-True ($null -ne $gxx) "g++.exe is required for host protocol tests"
$protocolBinary = Join-Path ([System.IO.Path]::GetTempPath()) ("a133p_host_protocol_{0}.exe" -f ([guid]::NewGuid().ToString("N")))
$deviceBinary = Join-Path ([System.IO.Path]::GetTempPath()) ("a133p_host_device_{0}.exe" -f ([guid]::NewGuid().ToString("N")))

try {
    & $gxx.Source -std=c++11 -Wall -Wextra -Werror `
        -I $userRoot `
        (Join-Path $PSScriptRoot "test_host_serial_protocol.cpp") `
        $protocolSource `
        -o $protocolBinary
    Assert-True ($LASTEXITCODE -eq 0) "Failed to compile host protocol test"
    & $protocolBinary
    Assert-True ($LASTEXITCODE -eq 0) "Host protocol test failed"

    & $gxx.Source -std=c++11 -Wall -Wextra -Werror `
        -I (Join-Path $PSScriptRoot "TestSupport") `
        -I $userRoot `
        (Join-Path $PSScriptRoot "test_host_serial_device.cpp") `
        (Join-Path $userRoot "Driver/drv_uart_dma.cpp") `
        $protocolSource `
        $deviceSource `
        -o $deviceBinary
    Assert-True ($LASTEXITCODE -eq 0) "Failed to compile host serial device test"
    & $deviceBinary
    Assert-True ($LASTEXITCODE -eq 0) "Host serial device test failed"
} finally {
    Remove-Item -LiteralPath $protocolBinary -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $deviceBinary -Force -ErrorAction SilentlyContinue
}

Write-Host "Host serial protocol integration checks passed."
