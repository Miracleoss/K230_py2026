param(
    [string]$ProjectRoot = (Resolve-Path "$PSScriptRoot/..").Path
)

$ErrorActionPreference = "Stop"

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if (-not $Condition) { throw $Message }
}

function Read-Text {
    param([string]$Path)
    return Get-Content -Raw -Encoding UTF8 $Path
}

$repoRoot = (Resolve-Path "$ProjectRoot/..").Path
$algorithmRoot = Join-Path $repoRoot "User/Algorithm"
$configRoot = Join-Path $repoRoot "User/Config"
$keilProject = Join-Path $ProjectRoot "A133P_Dart.uvprojx"
$eideProject = Join-Path $ProjectRoot ".eide/eide.yml"

$header = Join-Path $algorithmRoot "alg_closed_loop_controller.h"
$source = Join-Path $algorithmRoot "alg_closed_loop_controller.cpp"
$config = Join-Path $configRoot "control_config.h"
$testSource = Join-Path $ProjectRoot "Tools/test_closed_loop_controller.cpp"

foreach ($path in @($header, $source, $config, $testSource)) {
    Assert-True (Test-Path $path) "Missing closed-loop file: $path"
}

$headerText = Read-Text $header
$sourceText = Read-Text $source
$configText = Read-Text $config
$keilText = Read-Text $keilProject
$eideText = Read-Text $eideProject

foreach ($needle in @(
    "Class_ClosedLoopController",
    "ClosedLoopControlOutput",
    "ResultCode Update(",
    "n_y_meas",
    "n_z_meas",
    "omega_x_c",
    "omega_y_c",
    "omega_z_c",
    "ctrl_R",
    "ctrl_P",
    "ctrl_Y"
)) {
    Assert-True ($headerText.Contains($needle)) "Missing controller API term '$needle'"
}

foreach ($needle in @(
    "specific_force_y_mps2",
    "specific_force_z_mps2",
    "gravity_mps2",
    "UpdatePi(",
    "UpdatePid(",
    "Reset()",
    "ctrl_midpoint"
)) {
    Assert-True ($sourceText.Contains($needle)) "Missing controller implementation term '$needle'"
}

Assert-True ($configText.Contains("kControlGravityMps2 = 9.81f")) "Gravity constant must be 9.81 m/s^2"
Assert-True ($configText.Contains("kControlDtMinS")) "Controller dt limits are not configured"
Assert-True ($configText.Contains("kControlOutputMidpoint = 0.5f")) "Controller midpoint must be 0.5"

foreach ($needle in @(
    "../User/Algorithm/alg_closed_loop_controller.h",
    "../User/Algorithm/alg_closed_loop_controller.cpp"
)) {
    Assert-True ($keilText.Contains($needle)) "Keil project missing $needle"
    Assert-True ($eideText.Contains($needle)) "EIDE project missing $needle"
}
Assert-True ($keilText.Contains("../User/Config/control_config.h")) "Keil project missing control_config.h"
Assert-True ($eideText.Contains("../User/Config/control_config.h")) "EIDE project missing control_config.h"

$gxx = Get-Command g++.exe -ErrorAction SilentlyContinue
Assert-True ($null -ne $gxx) "g++.exe is required for the closed-loop controller test"
$binary = Join-Path ([System.IO.Path]::GetTempPath()) ("a133p_closed_loop_{0}.exe" -f ([guid]::NewGuid().ToString("N")))
try {
    & $gxx.Source `
        -std=c++11 -Wall -Wextra -Werror `
        -I (Join-Path $repoRoot "User") `
        $testSource $source `
        -o $binary
    Assert-True ($LASTEXITCODE -eq 0) "Failed to compile closed-loop controller test"
    & $binary
    Assert-True ($LASTEXITCODE -eq 0) "Closed-loop controller behavior test failed"
} finally {
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

Write-Host "Closed-loop controller contract checks passed."
