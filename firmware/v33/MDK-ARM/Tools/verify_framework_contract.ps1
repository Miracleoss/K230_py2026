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

function Assert-ProductionMainEntry {
    param([string]$Text)

    Assert-True ($Text.Contains('#include "task_bridge.h"')) "Production main must include task_bridge.h"
    foreach ($forbidden in @(
        "BMI088_MinimalInit",
        "BMI088_ReadSample",
        "HAL_SPI_TransmitReceive",
        "bmi088_test_"
    )) {
        Assert-True (-not $Text.Contains($forbidden)) "Production main must not contain minimal BMI088 path '$forbidden'"
    }

    $initializers = @(
        "MX_GPIO_Init();",
        "MX_DMA_Init();",
        "MX_USART1_UART_Init();",
        "MX_SPI1_Init();",
        "MX_TIM1_Init();",
        "MX_TIM3_Init();",
        "MX_TIM5_Init();",
        "Task_Init();"
    )
    $lastIndex = -1
    foreach ($initializer in $initializers) {
        $index = $Text.IndexOf($initializer)
        Assert-True ($index -ge 0) "Production main is missing '$initializer'"
        Assert-True ($index -gt $lastIndex) "Production main initialization order changed at '$initializer'"
        $lastIndex = $index
    }

    Assert-True ($Text.Contains("Task_Loop();")) "Production main must call Task_Loop in its infinite loop"
    Assert-True (-not [regex]::IsMatch($Text, '(?m)^\s*HAL_Delay\s*\(\s*10U\s*\)\s*;')) "Production main must not run the minimal fixed-delay sample loop"
}

function Get-CppBraceBlock {
    param([string]$Text, [int]$OpenBraceIndex)

    Assert-True (($OpenBraceIndex -ge 0) -and ($OpenBraceIndex -lt $Text.Length) -and ($Text[$OpenBraceIndex] -eq '{')) "Expected C++ block opening brace"
    $depth = 0
    for ($index = $OpenBraceIndex; $index -lt $Text.Length; ++$index) {
        if ($Text[$index] -eq '{') {
            ++$depth
        }
        elseif ($Text[$index] -eq '}') {
            --$depth
            if ($depth -eq 0) {
                return [pscustomobject]@{
                    Start = $OpenBraceIndex
                    End = $index
                    Body = $Text.Substring($OpenBraceIndex, $index - $OpenBraceIndex + 1)
                }
            }
        }
    }

    throw "Unterminated C++ block"
}

function Get-CppFunctionBlock {
    param([string]$Text, [string]$Name)

    $pattern = '(?m)\bvoid\s+' + [regex]::Escape($Name) + '\s*\(\s*void\s*\)\s*\{'
    $matches = @([regex]::Matches($Text, $pattern))
    Assert-True ($matches.Count -eq 1) "Expected exactly one $Name definition"
    return Get-CppBraceBlock $Text ($matches[0].Index + $matches[0].Length - 1)
}

function Find-CppIfBlock {
    param([string]$Text, $Scope, [string]$Pattern, [string]$Description)

    $matches = @([regex]::Matches($Text, $Pattern) | Where-Object { $_.Index -gt $Scope.Start -and $_.Index -lt $Scope.End })
    Assert-True ($matches.Count -eq 1) "Expected exactly one $Description conditional"
    $block = Get-CppBraceBlock $Text ($matches[0].Index + $matches[0].Length - 1)
    Assert-True ($block.End -lt $Scope.End) "$Description conditional must remain inside its parent block"
    return $block
}

function Assert-ExactlyOneMatchInBlock {
    param([string]$Text, $Scope, [string]$Pattern, [string]$Description)

    $matches = @([regex]::Matches($Text, $Pattern))
    Assert-True ($matches.Count -eq 1) "Expected exactly one $Description"
    Assert-True (($matches[0].Index -gt $Scope.Start) -and ($matches[0].Index -lt $Scope.End)) "$Description must remain inside the required block"
    return $matches[0]
}

function Assert-OneMatchWithinBlock {
    param([string]$Text, $Scope, [string]$Pattern, [string]$Description)

    $matches = @([regex]::Matches($Text, $Pattern) | Where-Object { $_.Index -gt $Scope.Start -and $_.Index -lt $Scope.End })
    Assert-True ($matches.Count -eq 1) "Expected exactly one $Description in its required block"
    return $matches[0]
}

function Get-CppBraceDepthAt {
    param([string]$Text, [int]$Position)

    $depth = 0
    for ($index = 0; $index -lt $Position; ++$index) {
        if ($Text[$index] -eq '{') {
            ++$depth
        }
        elseif ($Text[$index] -eq '}') {
            --$depth
        }
    }
    return $depth
}

function Get-InnermostCppBlockAt {
    param([string]$Text, [int]$Position)

    for ($index = $Position; $index -ge 0; --$index) {
        if ($Text[$index] -eq '{') {
            $block = Get-CppBraceBlock $Text $index
            if ($block.End -ge $Position) {
                return $block
            }
        }
    }

    throw "Expected a containing C++ block"
}

function Assert-Bmi088TaskWiring {
    param([string]$Text)

    $taskInit = Get-CppFunctionBlock $Text "Task_Init"
    $taskLoop = Get-CppFunctionBlock $Text "Task_Loop"

    Assert-True ($Text.Contains("static const uint8_t kBmi088BringupMaxAttempts = 5U;")) "BMI088 bring-up must use five bounded attempts"
    Assert-True ($Text.Contains("volatile uint8_t bmi088_bringup_attempt_count = 0U;")) "BMI088 bring-up attempt count must be visible in Watch"
    Assert-True ($Text.Contains("volatile ResultCode bmi088_init_attempt_results[5]")) "BMI088 initialization result history must be visible in Watch"
    Assert-True ($Text.Contains("volatile ResultCode bmi088_probe_attempt_results[5]")) "BMI088 probe result history must be visible in Watch"
    Assert-True ($Text.Contains("volatile uint8_t bmi088_acc_chip_id_attempts[5]")) "BMI088 ACC ID history must be visible in Watch"
    Assert-True ($Text.Contains("volatile uint8_t bmi088_gyro_chip_id_attempts[5]")) "BMI088 GYRO ID history must be visible in Watch"

    $watchSpecs = @(
        @{ Name = "bmi088_init_result"; Type = "ResultCode"; Initializer = "ResultCode::NotReady" },
        @{ Name = "bmi088_sample_result"; Type = "ResultCode"; Initializer = "ResultCode::NotReady" },
        @{ Name = "bmi088_raw_sample"; Type = "Bmi088RawSample"; Initializer = "\{\}" },
        @{ Name = "bmi088_sample_count"; Type = "uint32_t"; Initializer = "0U" }
    )
    foreach ($watch in $watchSpecs) {
        $pattern = '(?m)^[\t ]*(?!static\b)volatile\s+' + $watch.Type + '\s+' + $watch.Name + '\s*=\s*' + $watch.Initializer + '\s*;'
        $matches = @([regex]::Matches($Text, $pattern))
        Assert-True ($matches.Count -eq 1) "$($watch.Name) must be one non-static volatile Watch declaration"
        Assert-True ((Get-CppBraceDepthAt $Text $matches[0].Index) -eq 0) "$($watch.Name) must have file scope and external linkage"
        Assert-True ($matches[0].Index -lt $taskInit.Start) "$($watch.Name) must precede Task_Init"
    }

    $bodyDataMatches = @([regex]::Matches($Text, '(?m)^[\t ]*ImuData\s+bmi088_body_imu_data\s*=\s*\{\}\s*;'))
    Assert-True ($bodyDataMatches.Count -eq 1) "bmi088_body_imu_data must be one persistent non-volatile body-frame sample"
    Assert-True ((Get-CppBraceDepthAt $Text $bodyDataMatches[0].Index) -eq 0) "bmi088_body_imu_data must have file scope"
    Assert-True ($bodyDataMatches[0].Index -lt $taskInit.Start) "bmi088_body_imu_data must precede Task_Init"

    $resetPatterns = @(
        "bmi088_init_result\s*=\s*ResultCode::NotReady\s*;",
        "bmi088_sample_result\s*=\s*ResultCode::NotReady\s*;",
        "bmi088_raw_sample\.acc_x\s*=\s*0\s*;",
        "bmi088_raw_sample\.acc_y\s*=\s*0\s*;",
        "bmi088_raw_sample\.acc_z\s*=\s*0\s*;",
        "bmi088_raw_sample\.gyro_x\s*=\s*0\s*;",
        "bmi088_raw_sample\.gyro_y\s*=\s*0\s*;",
        "bmi088_raw_sample\.gyro_z\s*=\s*0\s*;",
        "bmi088_raw_sample\.temperature_raw\s*=\s*0\s*;",
        "bmi088_raw_sample\.temperature_deg_c\s*=\s*0\.0f\s*;",
        "bmi088_body_imu_data\s*=\s*ImuData\(\)\s*;",
        "bmi088_sample_count\s*=\s*0U\s*;",
        "bmi088_poll_divider\s*=\s*0U\s*;"
    )
    foreach ($pattern in $resetPatterns) {
        Assert-OneMatchWithinBlock $Text $taskInit $pattern "BMI088 Task_Init reset '$pattern'" | Out-Null
    }

    $retryMatches = @([regex]::Matches($Text, 'for\s*\(\s*uint8_t\s+attempt\s*=\s*0U\s*;\s*attempt\s*<\s*kBmi088BringupMaxAttempts\s*;\s*\+\+attempt\s*\)\s*\{') | Where-Object { $_.Index -gt $taskInit.Start -and $_.Index -lt $taskInit.End })
    Assert-True ($retryMatches.Count -eq 1) "BMI088 bring-up must have one bounded retry loop"
    $retryLoop = Get-CppBraceBlock $Text ($retryMatches[0].Index + $retryMatches[0].Length - 1)
    $probeCall = Assert-ExactlyOneMatchInBlock $Text $retryLoop "dart_system\.ProbeBmi088WithTrace\s*\(" "BMI088 traced probe call"
    $initializeCall = Assert-ExactlyOneMatchInBlock $Text $taskInit "dart_system\.InitializeBmi088\s*\(" "BMI088 initialization call"
    Assert-True (($initializeCall.Index -gt $retryLoop.Start) -and ($initializeCall.Index -lt $retryLoop.End)) "BMI088 initialization must run inside the retry loop"
    Assert-True ($initializeCall.Index -lt $probeCall.Index) "Each BMI088 attempt must initialize before recording its post-initialization probe"
    $successBlock = Find-CppIfBlock $Text $retryLoop "if\s*\(\s*bmi088_init_result\s*==\s*ResultCode::Ok\s*&&\s*bmi088_probe_result\s*==\s*ResultCode::Ok\s*\)\s*\{" "BMI088 bring-up success"
    Assert-True ($successBlock.Body -match '\bbreak\s*;') "Successful BMI088 bring-up must stop retrying"
    Assert-True ($retryLoop.Body.Contains("DelayBmi088Ms(30U);")) "Failed BMI088 bring-up attempts must wait 30 ms before retrying"

    $probeContext = Get-InnermostCppBlockAt $Text $probeCall.Index
    Assert-True (-not $probeContext.Body.Contains("StoreFirstTaskInitError(")) "BMI088 probe/init block must not block unrelated bring-up"
    $initErrorCalls = @([regex]::Matches($taskInit.Body, "StoreFirstTaskInitError\s*\((?<argument>[^)]*)\)"))
    foreach ($call in $initErrorCalls) {
        Assert-True ($call.Groups["argument"].Value -notmatch "bmi088_(?:probe|init)_result|probe_result|ProbeBmi088|InitializeBmi088") "BMI088 probe/init result must not flow into StoreFirstTaskInitError"
    }

    $dividerBlock = Find-CppIfBlock $Text $taskLoop "if\s*\(\s*bmi088_poll_divider\s*>=\s*5U\s*\)\s*\{" "BMI088 five-tick"
    $initializedBlock = Find-CppIfBlock $Text $taskLoop "if\s*\(\s*bmi088_init_result\s*==\s*ResultCode::Ok\s*\)\s*\{" "BMI088 initialized"
    Assert-True (($initializedBlock.Start -gt $dividerBlock.Start) -and ($initializedBlock.End -lt $dividerBlock.End)) "BMI088 initialized gate must be nested under the five-tick gate"
    $dividerIncrements = @([regex]::Matches($Text, "\+\+\s*bmi088_poll_divider\b"))
    Assert-True ($dividerIncrements.Count -eq 1) "BMI088 poll divider must increment exactly once"
    Assert-True (($dividerIncrements[0].Index -gt $taskLoop.Start) -and ($dividerIncrements[0].Index -lt $dividerBlock.Start)) "BMI088 poll divider must increment before the five-tick gate"
    $dividerResets = @([regex]::Matches($Text, "bmi088_poll_divider\s*=\s*0U\s*;") | Where-Object { $_.Index -gt $dividerBlock.Start -and $_.Index -lt $dividerBlock.End })
    Assert-True ($dividerResets.Count -eq 1) "BMI088 poll divider must reset once in its five-tick gate"

    $readCall = Assert-ExactlyOneMatchInBlock $Text $initializedBlock "dart_system\.ReadBmi088RawSample\s*\(" "BMI088 sample read"
    $sampleOk = Find-CppIfBlock $Text $initializedBlock "if\s*\(\s*bmi088_sample_result\s*==\s*ResultCode::Ok\s*\)\s*\{" "BMI088 sample-success"
    Assert-True ($readCall.Index -lt $sampleOk.Start) "BMI088 sample result must gate publication after the read"
    $storeCall = Assert-ExactlyOneMatchInBlock $Text $sampleOk "(?<!void )\bStoreBmi088RawSample\s*\(" "BMI088 raw sample publication"
    $countIncrement = Assert-ExactlyOneMatchInBlock $Text $sampleOk "\+\+\s*bmi088_sample_count\b" "BMI088 sample-count increment"
    Assert-True ($storeCall.Index -lt $countIncrement.Index) "BMI088 raw sample must publish before its successful count increment"

    $sampleCountWrites = @([regex]::Matches($Text, '(?m)(?:\+\+|--)\s*bmi088_sample_count\b|\bbmi088_sample_count\s*(?:\+\+|--|[+\-*/%&|^]?=)'))
    Assert-True ($sampleCountWrites.Count -eq 3) "BMI088 sample count may only be initialized, reset, and incremented once after a successful sample"
    $initResultWrites = @([regex]::Matches($Text, '(?m)\bbmi088_init_result\s*=(?!=)'))
    Assert-True ($initResultWrites.Count -eq 3) "BMI088 initialization result may only be declared, reset, and assigned from initialization"
    $sampleResultWrites = @([regex]::Matches($Text, '(?m)\bbmi088_sample_result\s*=(?!=)'))
    Assert-True ($sampleResultWrites.Count -eq 3) "BMI088 sample result may only be declared, reset, and assigned from one read attempt"
    $rawSampleWrites = @([regex]::Matches($Text, '(?m)(?:\+\+|--)\s*bmi088_raw_sample(?:\s*\.\s*[A-Za-z_][A-Za-z0-9_]*)?\b|\bbmi088_raw_sample(?:\s*\.\s*[A-Za-z_][A-Za-z0-9_]*)?\s*(?:\+\+|--|[+\-*/%&|^]?=)'))
    Assert-True ($rawSampleWrites.Count -eq 17) "BMI088 Watch sample may only be declared, fully reset, and fully published by StoreBmi088RawSample"

    $taintedResultNames = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::Ordinal)
    [void]$taintedResultNames.Add("bmi088_probe_result")
    [void]$taintedResultNames.Add("bmi088_init_result")
    $resultAssignments = @([regex]::Matches($taskInit.Body, '(?m)(?:\b(?:const\s+)?ResultCode\s+)?(?<lhs>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*(?<rhs>[^;]+);'))
    $addedTaint = $true
    while ($addedTaint) {
        $addedTaint = $false
        foreach ($assignment in $resultAssignments) {
            $rhs = $assignment.Groups["rhs"].Value
            $rhsIsTainted = $rhs -match '\b(?:ProbeBmi088(?:WithTrace)?|InitializeBmi088)\s*\('
            foreach ($name in @($taintedResultNames)) {
                if ($rhs -match ('\b' + [regex]::Escape($name) + '\b')) {
                    $rhsIsTainted = $true
                }
            }
            if ($rhsIsTainted -and $taintedResultNames.Add($assignment.Groups["lhs"].Value)) {
                $addedTaint = $true
            }
        }
    }
    foreach ($call in $initErrorCalls) {
        foreach ($name in @($taintedResultNames)) {
            Assert-True ($call.Groups["argument"].Value -notmatch ('\b' + [regex]::Escape($name) + '\b')) "BMI088 probe/init failures must not reach StoreFirstTaskInitError, including through temporary variables"
        }
    }

    $callbacks = @([regex]::Matches($Text, 'extern\s+"C"\s+[^;{]*?\b(?<name>[A-Za-z_][A-Za-z0-9_]*Callback)\s*\([^)]*\)\s*\{'))
    Assert-True ($callbacks.Count -gt 0) "Expected at least one extern C callback"
    foreach ($callback in $callbacks) {
        $callbackBlock = Get-CppBraceBlock $Text ($callback.Index + $callback.Length - 1)
        $forbiddenCallbackPattern = '\b(?:ProbeBmi088(?:WithTrace)?|InitializeBmi088|ReadBmi088RawSample)\b|\bbmi088_(?:spi|acc_cs|gyro_cs)\b|\.Transfer\s*\(|\.SetLow\s*\(|\.SetHigh\s*\(|\bHAL_SPI_[A-Za-z0-9_]*\s*\(|\bHAL_GPIO_WritePin\s*\('
        Assert-True (-not [regex]::IsMatch($callbackBlock.Body, $forbiddenCallbackPattern)) "Callback $($callback.Groups['name'].Value) must not perform BMI088, SPI, or CS work"
    }

    Assert-True (-not [regex]::IsMatch($Text, '(?m)^\s*(?:static\s+)?ImuData\s+imu_data\s*;')) "Task must not retain the placeholder imu_data object"
    Assert-True (-not [regex]::IsMatch($Text, '\bimu_data\b')) "Task must not retain placeholder imu_data references"
    Assert-True (-not [regex]::IsMatch($Text, '\btelemetry_divider\b')) "Task must not retain the obsolete telemetry divider"

    $conversionCall = Assert-ExactlyOneMatchInBlock $Text $sampleOk "ImuData\s+imu_candidate\s*=\s*Bmi088RawSampleToImuData\s*\(\s*candidate\s*,\s*now_ms\s*,\s*bmi088_sample_count\s*\)" "raw-to-SI IMU candidate conversion"
    $bodyStoreCall = Assert-ExactlyOneMatchInBlock $Text $sampleOk "StoreBmi088BodyImuData\s*\(\s*imu_candidate\s*\)" "successful-sample body IMU retention"
    $sendCall = Assert-ExactlyOneMatchInBlock $Text $sampleOk "Task_SendImuData\s*\(\s*bmi088_body_imu_data\s*,\s*test_command\s*\)" "successful-sample body IMU telemetry send"
    Assert-True ($conversionCall.Index -gt $countIncrement.Index) "IMU conversion must use the successfully incremented sample sequence"
    Assert-True ($bodyStoreCall.Index -gt $conversionCall.Index) "Body IMU retention must follow the one-time conversion"
    Assert-True ($sendCall.Index -gt $bodyStoreCall.Index) "IMU telemetry must send the retained body sample"
    Assert-True (([regex]::Matches($Text, 'Task_SendImuData\s*\(\s*imu_candidate\s*\)')).Count -eq 0) "IMU telemetry must not send a transient pre-retention sample"
    Assert-True (-not [regex]::IsMatch($Text, '\b(?:specific_force_[A-Za-z0-9_]+|gyro_[A-Za-z0-9_]+)\s*=\s*candidate\s*\.\s*(?:acc|gyro)_[A-Za-z0-9_]+')) "Task must not assign raw BMI088 counts directly to SI fields"
    Assert-True ($Text.Contains("volatile uint8_t imu_telemetry_mode = kTaskImuTelemetryHostProtocol")) "Task must default telemetry to the Host protocol"
    Assert-True ($Text.Contains("if (imu_telemetry_mode == kTaskImuTelemetryVofaJustFloat)")) "Task must expose a Watch-selectable VOFA mode"
    Assert-True ($Text.Contains("Task_SendVofaImuData(imu)")) "Task must retain a dedicated VOFA send entry"
}

function Assert-Bmi088MutationRejected {
    param([string]$Text, [string]$Target, [string]$Replacement, [string]$Description)

    $mutated = $Text.Replace($Target, $Replacement)
    Assert-True ($mutated -ne $Text) "Mutation setup failed for $Description"
    $rejected = $false
    try {
        Assert-Bmi088TaskWiring $mutated
    }
    catch {
        $rejected = $true
    }
    Assert-True $rejected "BMI088 task wiring contract must reject $Description"
}

function Read-LayerText {
    param([string]$Path)

    $text = ""
    foreach ($file in Get-ChildItem -Path $Path -File -Recurse) {
        $text += Read-Text $file.FullName
        $text += "`n"
    }
    return $text
}

$repoRoot = (Resolve-Path "$ProjectRoot/..").Path
$userRoot = Join-Path $repoRoot "User"
$taskBridge = Join-Path $userRoot "Task/task_bridge.cpp"
$productionMain = Join-Path $repoRoot "Core/Src/main.c"
$dartSystemHeader = Join-Path $userRoot "Application/app_dart_system.h"
$servoConfig = Join-Path $userRoot "Config/servo_config.cpp"

$requiredUserDirs = @(
    "Common",
    "Config",
    "Driver",
    "Device",
    "Algorithm",
    "Application",
    "Task"
)

foreach ($dir in $requiredUserDirs) {
    Assert-True (Test-Path (Join-Path $userRoot $dir)) "Missing User/$dir directory"
}

Assert-True (Test-Path $dartSystemHeader) "Missing Class_DartSystem header"
Assert-True (Test-Path (Join-Path $userRoot "Application/app_dart_system.cpp")) "Missing Class_DartSystem source"
Assert-True (Test-Path $taskBridge) "Missing task_bridge.cpp"
Assert-True (Test-Path $productionMain) "Missing production Core/Src/main.c"

$mainText = Read-Text $productionMain
$taskText = Read-Text $taskBridge
$dartText = Read-Text $dartSystemHeader
$applicationText = Read-LayerText (Join-Path $userRoot "Application")
$deviceText = Read-LayerText (Join-Path $userRoot "Device")
$algorithmText = Read-LayerText (Join-Path $userRoot "Algorithm")
$userText = Read-LayerText $userRoot

Assert-ProductionMainEntry $mainText
Assert-Bmi088TaskWiring $taskText
Assert-Bmi088MutationRejected $taskText "volatile ResultCode bmi088_init_result = ResultCode::NotReady;" "static volatile ResultCode bmi088_init_result = ResultCode::NotReady;" "a static BMI088 Watch symbol"
Assert-Bmi088MutationRejected $taskText "bmi088_sample_count = 0U;" "/* removed BMI088 sample-count reset */" "a missing Task_Init reset"
Assert-Bmi088MutationRejected $taskText "bmi088_init_result == ResultCode::Ok &&" "true &&" "an initialization retry detached from successful initialization"
Assert-Bmi088MutationRejected $taskText "if (bmi088_sample_result == ResultCode::Ok)" "if (true)" "a successful-sample publication gate removed"
Assert-Bmi088MutationRejected $taskText "++bmi088_sample_count;" "++bmi088_sample_count; bmi088_sample_count += 1U;" "an extra successful-sample count increment"
Assert-Bmi088MutationRejected $taskText "StoreBmi088RawSample(candidate);" "StoreBmi088RawSample(candidate); bmi088_raw_sample.acc_x = candidate.acc_x;" "an extra partial Watch-sample publication"
Assert-Bmi088MutationRejected $taskText "bmi088_init_result = dart_system.InitializeBmi088(DelayBmi088Ms);" "bmi088_init_result = dart_system.InitializeBmi088(DelayBmi088Ms); ResultCode forwarded_bmi088_error = bmi088_init_result; StoreFirstTaskInitError(forwarded_bmi088_error);" "a BMI088 initialization error forwarded through a temporary"
Assert-Bmi088MutationRejected $taskText "host_uart.OnRxErrorIsr(huart->ErrorCode);" "dart_system.ReadBmi088RawSample(candidate);" "BMI088 sampling in a callback"
Assert-Bmi088MutationRejected $taskText "Bmi088RawSampleToImuData(candidate, now_ms, bmi088_sample_count)" "ImuData()" "a missing raw-to-SI conversion"
Assert-Bmi088MutationRejected $taskText "StoreBmi088BodyImuData(imu_candidate);" "/* removed body retention */" "a missing persistent body IMU sample"
Assert-Bmi088MutationRejected $taskText "Task_SendImuData(bmi088_body_imu_data, test_command);" "Task_SendImuData(imu_candidate, test_command);" "sending a transient non-body IMU sample"
$imuMutation = $taskText + "`nvoid Task_Bmi088ContractMutation(void)`n{`n    imu_candidate.gyro_x_radps = candidate.gyro_x;`n}`n"
$imuMutationRejected = $false
try {
    Assert-Bmi088TaskWiring $imuMutation
}
catch {
    $imuMutationRejected = $true
}
Assert-True $imuMutationRejected "BMI088 task wiring contract must reject raw data assigned to an SI field"
Write-Host "BMI088 framework wiring mutation checks passed."

$requiredCompositionRootTerms = @(
    "static Class_SpiBus bmi088_spi",
    "static Class_GpioPin bmi088_acc_cs",
    "static Class_GpioPin bmi088_gyro_cs",
    "static Class_PwmChannel tim1_pwm_ch1",
    "static Class_PwmChannel tim1_pwm_ch2",
    "static Class_PwmChannel tim1_pwm_ch3",
    "static Class_PwmChannel tim1_pwm_ch4",
    "static Class_PwmChannel imu_heater_pwm",
    "static Class_UartDma host_uart",
    "static Class_Timebase system_timebase",
    "static Class_DartSystem dart_system",
    "volatile uint8_t imu_telemetry_mode = kTaskImuTelemetryHostProtocol",
    "Task_SendVofaImuData",
    "host_serial.SendVofaJustFloatData",
    "static uint8_t host_rx_dma_buffer[256]",
    "static volatile ResultCode host_uart_rx_result",
    "static volatile ResultCode servo_pwm_bringup_result",
    "static volatile uint8_t servo_pwm_failed_step",
    "volatile ResultCode bmi088_probe_result = ResultCode::NotReady",
    "volatile uint8_t bmi088_acc_chip_id = 0U",
    "volatile uint8_t bmi088_gyro_chip_id = 0U",
    "volatile ResultCode bmi088_probe_acc_activation_result = ResultCode::NotReady",
    "volatile ResultCode bmi088_probe_acc_id_result = ResultCode::NotReady",
    "volatile ResultCode bmi088_probe_gyro_id_result = ResultCode::NotReady",
    "volatile uint8_t bmi088_probe_acc_activation_rx[3] = {0U}",
    "volatile uint8_t bmi088_probe_acc_id_rx[3] = {0U}",
    "volatile uint8_t bmi088_probe_gyro_id_rx[2] = {0U}",
    "volatile uint8_t bmi088_bringup_attempt_count = 0U",
    "volatile ResultCode bmi088_init_attempt_results[5]",
    "volatile ResultCode bmi088_probe_attempt_results[5]",
    "volatile uint8_t bmi088_acc_chip_id_attempts[5]",
    "volatile uint8_t bmi088_gyro_chip_id_attempts[5]",
    "dart_system.ProbeBmi088WithTrace(acc_chip_id, gyro_chip_id, probe_trace)",
    "bmi088_probe_acc_activation_result = probe_trace.acc_activation_result",
    "bmi088_probe_acc_id_result = probe_trace.acc_id_result",
    "bmi088_probe_gyro_id_result = probe_trace.gyro_id_result",
    "bmi088_probe_acc_activation_rx[index] = probe_trace.acc_activation_rx[index]",
    "bmi088_probe_acc_id_rx[index] = probe_trace.acc_id_rx[index]",
    "bmi088_probe_gyro_id_rx[index] = probe_trace.gyro_id_rx[index]",
    "volatile ResultCode bmi088_init_result = ResultCode::NotReady",
    "volatile ResultCode bmi088_sample_result = ResultCode::NotReady",
    "volatile Bmi088RawSample bmi088_raw_sample = {}",
    "ImuData bmi088_body_imu_data = {}",
    "volatile uint32_t bmi088_sample_count = 0U",
    "static uint8_t bmi088_poll_divider = 0U",
    "static void DelayBmi088Ms(uint32_t delay_ms)",
    "HAL_Delay(delay_ms)",
    "bmi088_init_result == ResultCode::Ok &&",
    "bmi088_probe_result == ResultCode::Ok",
    "dart_system.InitializeBmi088(DelayBmi088Ms)",
    "dart_system.ReadBmi088RawSample(candidate)",
    "Bmi088RawSampleToImuData(candidate, now_ms, bmi088_sample_count)",
    "StoreBmi088BodyImuData(imu_candidate)",
    "Task_SendImuData(bmi088_body_imu_data, test_command)",
    "bmi088_raw_sample.acc_x = sample.acc_x",
    "bmi088_raw_sample.acc_y = sample.acc_y",
    "bmi088_raw_sample.acc_z = sample.acc_z",
    "bmi088_raw_sample.gyro_x = sample.gyro_x",
    "bmi088_raw_sample.gyro_y = sample.gyro_y",
    "bmi088_raw_sample.gyro_z = sample.gyro_z",
    "bmi088_raw_sample.temperature_raw = sample.temperature_raw",
    "bmi088_raw_sample.temperature_deg_c = sample.temperature_deg_c",
    "++bmi088_sample_count"
)

foreach ($needle in $requiredCompositionRootTerms) {
    Assert-True ($taskText.Contains($needle)) "Missing Task composition root term '$needle'"
}

$requiredDartSystemMembers = @(
    "Class_Bmi088 bmi088_",
    "Class_Servo servos_[kSurfaceCount]",
    "Class_ImuHeater imu_heater_",
    "Class_XTailMixer xtail_mixer_",
    "Class_AttitudeEstimator attitude_estimator_",
    "SystemStatus system_status_",
    "ResultCode ProbeBmi088(uint8_t& acc_chip_id, uint8_t& gyro_chip_id)",
    "ResultCode InitializeBmi088(Bmi088DelayMsFunction delay_ms)",
    "ResultCode ReadBmi088RawSample(Bmi088RawSample& sample)",
    "ResultCode ProbeBmi088WithTrace(",
    "bool IsBmi088Initialized() const",
    "ResultCode BindServo(Class_PwmChannel* servo_pwm, const ServoConfig* config)",
    "ResultCode ApplyBodyControlCommand(const BodyControlCommand& command)",
    "ResultCode UpdateClosedLoop(",
    "void ResetClosedLoop()",
    "Class_ClosedLoopController closed_loop_controller_"
)

foreach ($needle in $requiredDartSystemMembers) {
    Assert-True ($dartText.Contains($needle)) "Missing Class_DartSystem member '$needle'"
}

$forbiddenDriverMembers = @(
    "Class_SpiBus spi_",
    "Class_GpioPin gpio_",
    "Class_PwmChannel pwm_",
    "Class_UartDma uart_",
    "Class_Timebase timebase_"
)

foreach ($needle in $forbiddenDriverMembers) {
    Assert-True (-not $dartText.Contains($needle)) "Class_DartSystem owns Driver value member '$needle'"
}

$forbiddenHalTerms = @(
    "HAL_",
    "SPI_HandleTypeDef",
    "TIM_HandleTypeDef",
    "UART_HandleTypeDef",
    "GPIO_TypeDef"
)

foreach ($needle in $forbiddenHalTerms) {
    Assert-True (-not $applicationText.Contains($needle)) "Application layer leaks HAL term '$needle'"
    Assert-True (-not $deviceText.Contains($needle)) "Device layer leaks HAL term '$needle'"
    Assert-True (-not $algorithmText.Contains($needle)) "Algorithm layer leaks HAL term '$needle'"
}

$forbiddenAlgorithmDependencies = @(
    "Driver/",
    "Device/",
    "Application/",
    "Task/"
)

foreach ($needle in $forbiddenAlgorithmDependencies) {
    Assert-True (-not $algorithmText.Contains($needle)) "Algorithm layer has forbidden dependency '$needle'"
}

$forbiddenTaskRuntimeTerms = @(
    ".Transfer(",
    ".SetDutyCycle(",
    ".SetCompareTicks(",
    ".SendAsync(",
    ".SetHigh(",
    ".SetLow(",
    ".Write(",
    ".Update(",
    ".Reset("
)

foreach ($needle in $forbiddenTaskRuntimeTerms) {
    Assert-True (-not $taskText.Contains($needle)) "Task contains forbidden runtime call '$needle'"
}

$requiredXtailTaskTerms = @(
    "BodyControlCommand command = last_closed_loop_command",
    "command.valid = command.valid && host_fresh && imu_fresh",
    "dart_system.ApplyBodyControlCommand(command)",
    "dart_system.UpdateClosedLoop(",
    "n_y_c = latest_overload.pitch_overload_g",
    "n_z_c = latest_overload.yaw_overload_g",
    "last_control_sample_seq",
    "last_control_timestamp_ms",
    "dart_system.ResetClosedLoop()"
)

foreach ($needle in $requiredXtailTaskTerms) {
    Assert-True ($taskText.Contains($needle)) "Missing Stage 3D.1 Task mixer wiring '$needle'"
}

$requiredUartRxTerms = @(
    "host_uart.StartReceiveCircular(host_rx_dma_buffer, sizeof(host_rx_dma_buffer))",
    "host_serial.Poll()",
    'extern "C" void HAL_UART_ErrorCallback',
    "if (huart == &huart1)",
    "host_uart.OnRxErrorIsr(huart->ErrorCode)"
)

foreach ($needle in $requiredUartRxTerms) {
    Assert-True ($taskText.Contains($needle)) "Missing Stage 3B.0 UART RX term '$needle'"
}

$callbackMatch = [regex]::Match(
    $taskText,
    '(?s)extern "C" void HAL_UART_ErrorCallback\s*\([^)]*\)\s*\{(?<body>.*?)\r?\n\}'
)
Assert-True ($callbackMatch.Success) "Missing HAL_UART_ErrorCallback definition"
$callbackBody = $callbackMatch.Groups["body"].Value

$forbiddenCallbackTerms = @(
    "memcpy(",
    "ReadAvailable(",
    "StartReceiveCircular(",
    "StartReceiveToIdle(",
    "ServiceReceive(",
    "HAL_UART_AbortReceive(",
    "SendAsync(",
    "host_serial",
    "output_test(",
    "host_rx_dma_buffer[",
    "host_rx_debug_buffer[",
    "for (",
    "while (",
    "printf("
)

foreach ($needle in $forbiddenCallbackTerms) {
    Assert-True (-not $callbackBody.Contains($needle)) "UART RX ISR contains forbidden operation '$needle'"
}
Assert-True ($callbackBody.Contains("host_uart.OnRxErrorIsr(huart->ErrorCode)")) "UART error ISR must only notify the DMA wrapper"
Assert-True (-not $taskText.Contains("HAL_UARTEx_RxEventCallback")) "Task must not define an RX event callback for NDTR-polled DMA"

$requiredTim5Terms = @(
    "timebase_start_result = system_timebase.Start()",
    'extern "C" void HAL_TIM_PeriodElapsedCallback',
    "if (htim == &htim5)",
    "system_timebase.OnTickIsr()",
    "if (now_ms == last_processed_ms)",
    "ImuData imu_candidate =",
    "imu_tx_result = Task_SendImuData(bmi088_body_imu_data, test_command)"
)

foreach ($needle in $requiredTim5Terms) {
    Assert-True ($taskText.Contains($needle)) "Missing minimal TIM5 scheduling term '$needle'"
}

Assert-True (([regex]::Matches($taskText, "system_timebase\.OnTickIsr\(")).Count -eq 1) "TIM5 timebase tick must be forwarded exactly once"
Assert-True (-not $taskText.Contains("StoreFirstTaskInitError(probe_result)")) "BMI088 probe failure must not block unrelated bring-up"
Assert-True (-not $taskText.Contains("StoreFirstTaskInitError(bmi088_init_result)")) "BMI088 initialization failure must not block unrelated bring-up"

$tim5CallbackMatch = [regex]::Match(
    $taskText,
    '(?s)extern "C" void HAL_TIM_PeriodElapsedCallback\s*\([^)]*\)\s*\{(?<body>.*?)\r?\n\}'
)
Assert-True ($tim5CallbackMatch.Success) "Missing HAL_TIM_PeriodElapsedCallback definition"
$tim5CallbackBody = $tim5CallbackMatch.Groups["body"].Value

$forbiddenTim5CallbackTerms = @(
    "host_serial",
    "dart_system",
    "Task_",
    "SendImuData",
    "ApplyBodyControlCommand",
    "for (",
    "while (",
    "printf("
)

foreach ($needle in $forbiddenTim5CallbackTerms) {
    Assert-True (-not $tim5CallbackBody.Contains($needle)) "TIM5 ISR contains forbidden business operation '$needle'"
}

Assert-True ($taskText.Contains("host_uart_rx_result = rx_start_result")) "Initial UART RX start result must be recorded"
Assert-True (-not $taskText.Contains("StartReceiveToIdle")) "Task must retire Stage 3B.0 Normal-DMA RX start semantics"
Assert-True (-not $taskText.Contains("TakeRxEvent")) "Task must retire Stage 3B.0 event-oriented RX consumption"
Assert-True (-not $taskText.Contains("rearm_result")) "Task must not retain an RX re-arm path"
Assert-True (([regex]::Matches($taskText, "host_uart\.StartReceiveCircular\(")).Count -eq 1) "Task must start circular RX exactly once"

$forbiddenSecondaryRxTerms = @(
    "host_rx_debug_buffer",
    "host_rx_debug_size",
    "host_rx_read_count",
    "host_rx_overflow_count",
    "rx_copy_buffer",
    "rx_snapshot",
    "debug_rx_copy",
    "software_ring_buffer",
    "secondary_fifo",
    "ReadAvailable("
)

foreach ($needle in $forbiddenSecondaryRxTerms) {
    Assert-True (-not $taskText.Contains($needle)) "Forbidden secondary RX/task term '$needle'"
}

$taskLoopMatch = [regex]::Match(
    $taskText,
    '(?s)void Task_Loop\s*\(\s*void\s*\)\s*\{(?<body>.*?)\r?\n\}'
)
Assert-True ($taskLoopMatch.Success) "Missing Task_Loop definition"
$taskLoopBody = $taskLoopMatch.Groups["body"].Value
Assert-True (-not $taskLoopBody.Contains("StartReceiveCircular")) "Task_Loop must not restart circular RX"
Assert-True (-not $taskLoopBody.Contains("StartReceiveToIdle")) "Task_Loop must not re-arm RX"
Assert-True (-not $taskLoopBody.Contains("host_uart")) "Task_Loop must not consume or inspect UART during Stage 3B.0R-FIX"
Assert-True (-not $taskText.Contains("host_uart.SendAsync(")) "Stage 3B.0R must not use UART TX"
Assert-True (-not $taskLoopBody.Contains("ProbeBmi088WithTrace(")) "Task_Loop must not repeat the one-shot BMI088 probe"
Assert-True ($taskLoopBody.Contains("++bmi088_poll_divider")) "Task_Loop must advance the dedicated BMI088 poll divider"
Assert-True ($taskLoopBody.Contains("if (bmi088_poll_divider >= 5U)")) "Task_Loop must sample BMI088 every five processed milliseconds"
Assert-True ($taskLoopBody.Contains("if (bmi088_init_result == ResultCode::Ok)")) "Task_Loop must only sample an initialized BMI088"
Assert-True ($taskLoopBody.Contains("bmi088_sample_result = dart_system.ReadBmi088RawSample(candidate)")) "Task_Loop must store each BMI088 sample attempt result"
Assert-True ($taskLoopBody.Contains("if (bmi088_sample_result == ResultCode::Ok)")) "Task_Loop must publish only successful BMI088 samples"
Assert-True ($taskLoopBody.Contains("StoreBmi088RawSample(candidate)")) "Task_Loop must publish a complete successful sample"
Assert-True ($taskLoopBody.Contains("++bmi088_sample_count")) "Task_Loop must count successful BMI088 samples"

$usartSource = Read-Text (Join-Path $repoRoot "Core/Src/usart.c")
Assert-True ($usartSource.Contains("huart1.Init.BaudRate = 115200")) "USART1 baud must remain 115200"
Assert-True ($usartSource.Contains("huart1.Init.WordLength = UART_WORDLENGTH_8B")) "USART1 word length must remain 8 bits"
Assert-True ($usartSource.Contains("huart1.Init.StopBits = UART_STOPBITS_1")) "USART1 must remain one stop bit"
Assert-True ($usartSource.Contains("huart1.Init.Parity = UART_PARITY_NONE")) "USART1 parity must remain disabled"
Assert-True ($usartSource.Contains("GPIO_PIN_6|GPIO_PIN_7")) "USART1 must remain on PB6/PB7"
Assert-True ($usartSource.Contains("GPIO_AF7_USART1")) "USART1 PB6/PB7 must remain AF7"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Instance = DMA2_Stream2")) "USART1 RX DMA must remain DMA2 Stream2"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4")) "USART1 RX DMA must remain Channel4"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY")) "USART1 RX DMA direction must remain peripheral-to-memory"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE")) "USART1 RX DMA peripheral increment must remain disabled"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE")) "USART1 RX DMA memory increment must remain enabled"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE")) "USART1 RX DMA peripheral alignment must remain byte"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE")) "USART1 RX DMA memory alignment must remain byte"
Assert-True ($usartSource.Contains("hdma_usart1_rx.Init.Mode = DMA_CIRCULAR")) "USART1 RX DMA must be CubeMX-generated circular mode"
Assert-True ($usartSource.Contains("hdma_usart1_tx.Init.Mode = DMA_NORMAL")) "USART1 TX DMA must remain normal mode"

$allowedServoPwmChannels = @(
    "tim1_pwm_ch1",
    "tim1_pwm_ch2",
    "tim1_pwm_ch3",
    "tim1_pwm_ch4"
)

$servoText = Read-Text $servoConfig
$requiredServoNeutralPatterns = @{
    "Servo1" = 'SurfaceId::Servo1RightUpper,\s*PwmChannelId::Tim1Ch1,\s*333U,\s*2U,\s*true,\s*1470U,\s*0,\s*970U,\s*1970U,.*?true'
    "Servo2" = 'SurfaceId::Servo2LeftUpper,\s*PwmChannelId::Tim1Ch2,\s*333U,\s*2U,\s*true,\s*1470U,\s*0,\s*970U,\s*1970U,.*?true'
    "Servo3" = 'SurfaceId::Servo3LeftLower,\s*PwmChannelId::Tim1Ch3,\s*333U,\s*2U,\s*true,\s*1470U,\s*0,\s*970U,\s*1970U,.*?true'
    "Servo4" = 'SurfaceId::Servo4RightLower,\s*PwmChannelId::Tim1Ch4,\s*333U,\s*2U,\s*true,\s*1570U,\s*0,\s*1070U,\s*2070U,.*?true'
}

foreach ($entry in $requiredServoNeutralPatterns.GetEnumerator()) {
    Assert-True ([regex]::IsMatch($servoText, $entry.Value, [System.Text.RegularExpressions.RegexOptions]::Singleline)) "$($entry.Key) calibrated neutral pulse is missing"
}

$requiredServoCalibrationTerms = @(
    '#include "Config/servo_config.h"',
    "volatile uint16_t test_PulseWidthUs1 = 0U",
    "volatile uint16_t test_PulseWidthUs2 = 0U",
    "volatile uint16_t test_PulseWidthUs3 = 0U",
    "volatile uint16_t test_PulseWidthUs4 = 0U",
    "test_PulseWidthUs1 = kServoConfigs[0].neutral_pulse_us",
    "test_PulseWidthUs2 = kServoConfigs[1].neutral_pulse_us",
    "test_PulseWidthUs3 = kServoConfigs[2].neutral_pulse_us",
    "test_PulseWidthUs4 = kServoConfigs[3].neutral_pulse_us"
)

foreach ($needle in $requiredServoCalibrationTerms) {
    Assert-True ($taskText.Contains($needle)) "Missing servo calibration wiring '$needle'"
}

$requiredSetPulseCalls = @(
    "tim1_pwm_ch1.SetPulseWidthUs(test_PulseWidthUs1)",
    "tim1_pwm_ch2.SetPulseWidthUs(test_PulseWidthUs2)",
    "tim1_pwm_ch3.SetPulseWidthUs(test_PulseWidthUs3)",
    "tim1_pwm_ch4.SetPulseWidthUs(test_PulseWidthUs4)"
)

$requiredStartCalls = @(
    "tim1_pwm_ch1.Start()",
    "tim1_pwm_ch2.Start()",
    "tim1_pwm_ch3.Start()",
    "tim1_pwm_ch4.Start()"
)

function Assert-InIncreasingOrder {
    param(
        [string[]]$Needles,
        [string]$Description
    )

    $lastIndex = -1
    foreach ($needle in $Needles) {
        $index = $taskText.IndexOf($needle)
        Assert-True ($index -ge 0) "Missing $Description '$needle'"
        Assert-True ($index -gt $lastIndex) "$Description calls are out of order at '$needle'"
        $lastIndex = $index
    }
}

Assert-InIncreasingOrder $requiredSetPulseCalls "TIM1 servo set-pulse call"
Assert-InIncreasingOrder $requiredStartCalls "TIM1 servo start call"

$lastSetPulseIndex = $taskText.IndexOf($requiredSetPulseCalls[-1])
$firstStartIndex = $taskText.IndexOf($requiredStartCalls[0])
Assert-True ($lastSetPulseIndex -lt $firstStartIndex) "All four TIM1 servo pulse widths must be set before any TIM1 servo Start call"

$allSetPulseCalls = [regex]::Matches($taskText, '(?<![A-Za-z0-9_])([A-Za-z_][A-Za-z0-9_]*)(\.|->)SetPulseWidthUs\s*\(([^)]*)\)')
Assert-True ($allSetPulseCalls.Count -eq 4) "Task must contain exactly four safe-startup SetPulseWidthUs calls"
$expectedPulseArgumentByChannel = @{
    "tim1_pwm_ch1" = "test_PulseWidthUs1"
    "tim1_pwm_ch2" = "test_PulseWidthUs2"
    "tim1_pwm_ch3" = "test_PulseWidthUs3"
    "tim1_pwm_ch4" = "test_PulseWidthUs4"
}
$pulseCallCountByChannel = @{}
foreach ($call in $allSetPulseCalls) {
    $receiver = $call.Groups[1].Value
    $accessor = $call.Groups[2].Value
    $argument = ($call.Groups[3].Value -replace '\s', '')
    Assert-True (($accessor -eq ".") -and ($allowedServoPwmChannels -contains $receiver) -and ($argument -eq $expectedPulseArgumentByChannel[$receiver])) "Unexpected SetPulseWidthUs call '$($call.Value)'"
    $pulseCallCountByChannel[$receiver] = 1 + [int]$pulseCallCountByChannel[$receiver]
}
foreach ($channel in $allowedServoPwmChannels) {
    Assert-True ($pulseCallCountByChannel[$channel] -eq 1) "$channel must be initialized once before PWM start"
}

$allStartCalls = [regex]::Matches($taskText, '(?<![A-Za-z0-9_])([A-Za-z_][A-Za-z0-9_]*)(\.|->)Start\s*\(\s*\)')
Assert-True ($allStartCalls.Count -eq 5) "Task must contain four servo Start calls and one timebase Start call"
foreach ($call in $allStartCalls) {
    $receiver = $call.Groups[1].Value
    $accessor = $call.Groups[2].Value
    $allowedReceiver = ($allowedServoPwmChannels -contains $receiver) -or ($receiver -eq "system_timebase")
    Assert-True (($accessor -eq ".") -and $allowedReceiver) "Unexpected Start call '$($call.Value)'"
}
Assert-True (([regex]::Matches($taskText, "system_timebase\.Start\(")).Count -eq 1) "TIM5 timebase must start exactly once"

$lastServoStartIndex = $taskText.IndexOf($requiredStartCalls[-1])
$timebaseStartIndex = $taskText.IndexOf("system_timebase.Start()")
Assert-True ($lastServoStartIndex -lt $timebaseStartIndex) "TIM5 timebase must start after all servo PWM channels"

Assert-True (([regex]::Matches($taskText, "dart_system\.ProbeBmi088WithTrace\(")).Count -eq 1) "Task must probe BMI088 exactly once during startup"

$forbiddenScopeTerms = @(
    "Class_FlightController",
    "Class_HostCommunication",
    "A133P_HEADER",
    "A133P_FRAME",
    "BMI088_REG",
    "WriteRegister",
    "Class_Scheduler",
    "EventQueue",
    "namespace ",
    "new ",
    "delete "
)

foreach ($needle in $forbiddenScopeTerms) {
    Assert-True (-not $userText.Contains($needle)) "Forbidden Stage 2 scope term '$needle'"
}

$requiredServoChannels = @(
    "PwmChannelId::Tim1Ch1",
    "PwmChannelId::Tim1Ch2",
    "PwmChannelId::Tim1Ch3",
    "PwmChannelId::Tim1Ch4"
)
foreach ($channel in $requiredServoChannels) {
    Assert-True ($servoText.Contains($channel)) "ServoConfig is missing frozen PWM mapping '$channel'"
}
Assert-True (-not $servoText.Contains("PwmChannelId::Unassigned")) "All four Servo PWM channels must be assigned"

$forbiddenFiles = @(
    "class_flight_controller.h",
    "class_flight_controller.cpp",
    "class_host_communication.h",
    "class_host_communication.cpp",
    "class_scheduler.h",
    "class_scheduler.cpp",
    "class_event_queue.h",
    "class_event_queue.cpp",
    "class_pid.h",
    "class_pid.cpp"
)

foreach ($file in $forbiddenFiles) {
    Assert-True (-not (Test-Path (Join-Path $userRoot $file))) "Forbidden Stage 2 file exists at User root: $file"
    Assert-True (-not (Get-ChildItem -Path $userRoot -Recurse -File -Filter $file)) "Forbidden Stage 2 file exists: $file"
}

$xtailSource = Read-Text (Join-Path $userRoot "Algorithm/alg_xtail_mixer.cpp")
$estimatorSource = Read-Text (Join-Path $userRoot "Algorithm/alg_attitude_estimator.cpp")
Assert-True ($xtailSource.Contains("ResultCode Class_XTailMixer::Mix(")) "Stage 3D.1 XTail mixer implementation is missing"
Assert-True (-not $estimatorSource.Contains("Reset(")) "Attitude estimator Reset must remain unimplemented"
Assert-True (-not $estimatorSource.Contains("Update(")) "Attitude estimator Update must remain unimplemented"

Write-Host "Framework contract checks passed."
