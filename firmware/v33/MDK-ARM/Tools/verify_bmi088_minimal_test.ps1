$ErrorActionPreference = "Stop"

$mainPath = Join-Path $PSScriptRoot "bmi088_minimal_main_reference.c"
$text = Get-Content -LiteralPath $mainPath -Raw

function Assert-Contains([string]$pattern, [string]$message) {
    if (-not [regex]::IsMatch($text, $pattern)) {
        throw $message
    }
}

Assert-Contains '\bvolatile\s+uint8_t\s+bmi088_test_acc_rx\s*\[\s*3\s*\]' "Missing three-byte ACC RX Watch buffer"
Assert-Contains '\bvolatile\s+uint8_t\s+bmi088_test_gyro_rx\s*\[\s*2\s*\]' "Missing two-byte GYRO RX Watch buffer"
Assert-Contains '\bbmi088_test_acc_chip_id\s*=\s*bmi088_test_acc_rx\s*\[\s*2\s*\]' "ACC ID must come from RX byte 2"
Assert-Contains '\bbmi088_test_gyro_chip_id\s*=\s*bmi088_test_gyro_rx\s*\[\s*1\s*\]' "GYRO ID must come from RX byte 1"
Assert-Contains '\bbmi088_test_pass\s*=\s*\(\s*bmi088_test_acc_chip_id\s*==\s*0x1EU\s*\)\s*&&\s*\(\s*bmi088_test_gyro_chip_id\s*==\s*0x0FU\s*\)' "Pass flag must require both documented chip IDs"
Assert-Contains '\bvolatile\s+uint8_t\s+bmi088_test_init_pass\b' "Missing initialization pass flag"
Assert-Contains '\bvolatile\s+uint32_t\s+bmi088_test_sample_count\b' "Missing successful sample counter"
Assert-Contains '\bvolatile\s+int16_t\s+bmi088_test_acc_x\b' "Missing ACC X raw Watch value"
Assert-Contains '\bvolatile\s+int16_t\s+bmi088_test_acc_y\b' "Missing ACC Y raw Watch value"
Assert-Contains '\bvolatile\s+int16_t\s+bmi088_test_acc_z\b' "Missing ACC Z raw Watch value"
Assert-Contains '\bvolatile\s+int16_t\s+bmi088_test_gyro_x\b' "Missing GYRO X raw Watch value"
Assert-Contains '\bvolatile\s+int16_t\s+bmi088_test_gyro_y\b' "Missing GYRO Y raw Watch value"
Assert-Contains '\bvolatile\s+int16_t\s+bmi088_test_gyro_z\b' "Missing GYRO Z raw Watch value"
Assert-Contains '\bvolatile\s+int16_t\s+bmi088_test_temperature_raw\b' "Missing temperature raw Watch value"
Assert-Contains '\bvolatile\s+float\s+bmi088_test_temperature_deg_c\b' "Missing temperature Celsius Watch value"

$transfers = [regex]::Matches($text, '\bHAL_SPI_TransmitReceive\s*\(')
if ($transfers.Count -ne 2) {
    throw "Minimal test must contain one ACC and one GYRO HAL transfer helper; found $($transfers.Count) calls"
}

Assert-Contains '\bBMI088_WriteAccRegister\s*\(\s*0x7EU\s*,\s*0xB6U\s*\)' "Missing ACC soft reset"
Assert-Contains '\bBMI088_WriteGyroRegister\s*\(\s*0x14U\s*,\s*0xB6U\s*\)' "Missing GYRO soft reset"
Assert-Contains '\bBMI088_WriteAccRegister\s*\(\s*0x7CU\s*,\s*0x00U\s*\)' "Missing ACC active power configuration"
Assert-Contains '\bBMI088_WriteAccRegister\s*\(\s*0x7DU\s*,\s*0x04U\s*\)' "Missing ACC power enable"
Assert-Contains '\bBMI088_WriteAccRegister\s*\(\s*0x40U\s*,\s*0xA9U\s*\)' "Missing ACC bandwidth/ODR configuration"
Assert-Contains '\bBMI088_WriteAccRegister\s*\(\s*0x41U\s*,\s*0x01U\s*\)' "Missing ACC range configuration"
Assert-Contains '\bBMI088_WriteGyroRegister\s*\(\s*0x10U\s*,\s*0x04U\s*\)' "Missing GYRO bandwidth configuration"
Assert-Contains '\bBMI088_WriteGyroRegister\s*\(\s*0x0FU\s*,\s*0x00U\s*\)' "Missing GYRO range configuration"

Assert-Contains '\bBMI088_ReadAccRegisters\s*\(\s*0x12U\s*,\s*acc_data\s*,\s*6U\s*\)' "Missing six-byte ACC sample read"
Assert-Contains '\bBMI088_ReadGyroRegisters\s*\(\s*0x02U\s*,\s*gyro_data\s*,\s*6U\s*\)' "Missing six-byte GYRO sample read"
Assert-Contains '\bBMI088_ReadAccRegisters\s*\(\s*0x22U\s*,\s*temperature_data\s*,\s*2U\s*\)' "Missing two-byte temperature read"
Assert-Contains '\bbmi088_test_sample_status\s*=\s*BMI088_ReadSample\s*\(\s*\)' "Main loop must publish the sample status"
Assert-Contains '\bHAL_Delay\s*\(\s*10U\s*\)' "Main loop must pace samples at 10 ms"

if ([regex]::IsMatch($text, '(?m)^\s*Task_(?:Init|Loop)\s*\(')) {
    throw "main.c must not enter the application task path during the minimal test"
}

Write-Host "BMI088 minimal-test source contract passed."
