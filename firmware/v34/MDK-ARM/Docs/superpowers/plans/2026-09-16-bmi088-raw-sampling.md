# BMI088 Raw Sampling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Initialize BMI088 deterministically and expose a 200 Hz task-context stream of six-axis raw counts and decoded temperature for Keil Watch validation.

**Architecture:** `Class_Bmi088` owns register transactions, initialization state, byte decoding, and configuration verification while receiving delay services through a function pointer. `Class_DartSystem` adds thin forwarding methods. `task_bridge.cpp` schedules polling from the existing TIM5 1 kHz timebase and publishes dedicated Watch symbols without placing raw counts into `ImuData`.

**Tech Stack:** C++11, STM32F4 HAL, Keil ARMCLANG, PowerShell contract scripts, MinGW g++ host behavior tests.

**Spec:** `Docs/superpowers/specs/2026-09-16-bmi088-raw-sampling-design.md`

## Global Constraints

- Device and Application code must not include or call STM32 HAL APIs.
- SPI transactions remain blocking and execute only in task context, never in TIM5 or EXTI interrupt context.
- ACC uses an address byte plus one dummy byte for every read; GYRO has no dummy byte.
- Configure ACC as ±6 g, normal filter, 200 Hz and GYRO as ±2000 °/s, 23 Hz bandwidth, 200 Hz ODR.
- Keep raw values separate from the SI-unit, calibrated `ImuData` type.
- Preserve unrelated uncommitted files and existing debug behavior.

---

### Task 1: Add a failing BMI088 device behavior test

**Files:**
- Create: `Tools/test_bmi088_device.cpp`
- Modify: `Tools/verify_device_contract.ps1`

**Interfaces:**
- Consumes: existing `Class_SpiBus::Bind/Transfer`, `Class_GpioPin::Bind/SetLow/SetHigh`, and HAL test declarations.
- Produces: an executable behavior specification for `Bmi088RawSample`, `Bmi088DelayMsFunction`, `Class_Bmi088::Initialize`, `ReadRawSample`, and `IsInitialized`.

- [ ] **Step 1: Create an in-memory BMI088 HAL model**

Implement `HAL_GPIO_WritePin()` so it records ACC/GYRO CS state and `HAL_SPI_TransmitReceive()` so it interprets transfers according to the selected chip. Maintain 256-byte ACC/GYRO register arrays, return `0x1E` and `0x0F` Chip IDs, emulate ACC dummy bytes, emulate GYRO bandwidth bit 7 reading as one, record write operations, and support an injected transfer failure index.

```cpp
struct RecordedWrite {
    bool accelerometer;
    uint8_t address;
    uint8_t value;
};

static uint8_t g_acc_registers[256];
static uint8_t g_gyro_registers[256];
static RecordedWrite g_writes[16];
static uint32_t g_write_count;
static uint32_t g_transfer_count;
static uint32_t g_fail_transfer_number;
static bool g_acc_cs_low;
static bool g_gyro_cs_low;
```

- [ ] **Step 2: Write initialization assertions**

Bind real driver objects to fake HAL handles, then assert:

```cpp
passed &= Expect(!bmi088.IsInitialized(), "new BMI088 must not be initialized");
passed &= Expect(bmi088.Initialize(nullptr) == ResultCode::ConfigError,
                 "initialization requires a delay callback");
passed &= Expect(bmi088.Initialize(RecordDelayMs) == ResultCode::Ok,
                 "valid register responses must initialize BMI088");
passed &= Expect(bmi088.IsInitialized(), "successful initialization must latch readiness");
```

Check the exact write list and delay list:

```text
ACC 0x7E=0xB6, GYRO 0x14=0xB6,
ACC 0x7C=0x00, ACC 0x7D=0x04,
ACC 0x40=0xA9, ACC 0x41=0x01,
GYRO 0x10=0x04, GYRO 0x0F=0x00

delays: 1, 30, 5, 5, 40, 1, 1, 10 ms
```

Add a register-corruption switch to the HAL model and assert that a mismatching readback returns `InvalidData` and leaves `IsInitialized()` false.

- [ ] **Step 3: Write raw-sample decoding assertions**

Load register pairs that cover positive and negative two's-complement values, then assert all axes and temperature:

```cpp
passed &= Expect(sample.acc_x == 0x1234, "ACC X must decode little-endian");
passed &= Expect(sample.acc_y == static_cast<int16_t>(0xFEDC), "ACC Y must keep sign");
passed &= Expect(sample.gyro_z == static_cast<int16_t>(0x8000), "GYRO Z must keep sign");
passed &= Expect(sample.temperature_raw == -8, "temperature must sign-extend 11 bits");
passed &= Expect(sample.temperature_deg_c == 22.0f, "temperature conversion must use 0.125 C/LSB + 23 C");
```

Record transaction sizes and assert ACC XYZ is 8 bytes, GYRO XYZ is 7 bytes, and temperature is 4 bytes.

- [ ] **Step 4: Write failure atomicity assertions**

Seed the output sample with sentinel values, fail the second sample transfer, and assert `HardwareError`, both CS lines high, and every output field unchanged.

- [ ] **Step 5: Add the host compile/run command to the Device contract**

Append a `try/finally` block to `verify_device_contract.ps1` that builds and runs:

```powershell
& $gxx.Source `
    -std=c++11 -Wall -Wextra -Werror `
    -I $testSupport `
    -I (Join-Path $repoRoot "User") `
    $testSource `
    (Join-Path $deviceRoot "dev_bmi088.cpp") `
    (Join-Path $repoRoot "User/Driver/drv_gpio_pin.cpp") `
    (Join-Path $repoRoot "User/Driver/drv_spi_bus.cpp") `
    -o $testBinary
```

- [ ] **Step 6: Run the test and verify RED**

Run: `powershell -ExecutionPolicy Bypass -File .\Tools\verify_device_contract.ps1`

Expected: FAIL because `Bmi088RawSample`, `Initialize`, `ReadRawSample`, and `IsInitialized` do not exist yet.

---

### Task 2: Implement BMI088 initialization and raw reading

**Files:**
- Modify: `../User/Device/dev_bmi088.h`
- Modify: `../User/Device/dev_bmi088.cpp`
- Modify: `Tools/verify_device_contract.ps1`

**Interfaces:**
- Consumes: `Class_SpiBus`, two `Class_GpioPin` instances, and `Bmi088DelayMsFunction`.
- Produces: `Bmi088RawSample`, `Initialize()`, `ReadRawSample()`, and `IsInitialized()`.

- [ ] **Step 1: Add the public raw-sample API**

```cpp
struct Bmi088RawSample {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature_raw;
    float temperature_deg_c;
};

typedef void (*Bmi088DelayMsFunction)(uint32_t delay_ms);

ResultCode Initialize(Bmi088DelayMsFunction delay_ms);
ResultCode ReadRawSample(Bmi088RawSample& sample);
bool IsInitialized() const;
```

Add private single-write, burst-read, masked-verification, chip-select transfer, and decoding helpers plus `bool initialized_`.

- [ ] **Step 2: Implement transaction helpers**

Use fixed-size stack buffers. `ReadAccRegisters()` permits 1-6 payload bytes and transfers `length + 2`; `ReadGyroRegisters()` transfers `length + 1`. All helpers restore CS high after a started transfer.

- [ ] **Step 3: Implement the exact initialization state machine**

Perform the eight writes and eight delays specified in Task 1. Re-activate ACC SPI after its reset, Probe both IDs after both reset delays, and verify `ACC_PWR_CONF`, `ACC_PWR_CTRL`, `ACC_CONF`, `ACC_RANGE`, `GYRO_BANDWIDTH`, and `GYRO_RANGE`. Set `initialized_` only after the final verification.

- [ ] **Step 4: Implement failure-atomic sample reading**

Read ACC, GYRO, then temperature into local buffers. Decode into a local `Bmi088RawSample candidate = {};`, and assign `sample = candidate` only after all reads succeed.

- [ ] **Step 5: Evolve the static Device contract**

Add the new public/private API terms to `$contracts["dev_bmi088.h"]`, remove the obsolete Stage-2 prohibition on `WriteRegister`, and retain all HAL/dependency prohibitions.

- [ ] **Step 6: Run the Device contract and verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File .\Tools\verify_device_contract.ps1`

Expected: `BMI088 device behavior checks passed.` followed by `Device contract checks passed.`

---

### Task 3: Forward BMI088 operations through the Application layer

**Files:**
- Modify: `../User/Application/app_dart_system.h`
- Modify: `../User/Application/app_dart_system.cpp`
- Modify: `Tools/test_dart_system_control_output.cpp`
- Modify: `Tools/verify_application_contract.ps1`

**Interfaces:**
- Consumes: Device interfaces from Task 2.
- Produces: `InitializeBmi088`, `ReadBmi088RawSample`, and `IsBmi088Initialized`.

- [ ] **Step 1: Add failing unbound-forwarding assertions**

In the existing DartSystem host test, add:

```cpp
static void NoDelay(uint32_t) {}
Bmi088RawSample raw = {};
passed &= Expect(unbound_system.InitializeBmi088(NoDelay) == ResultCode::NotReady,
                 "unbound BMI088 initialization must report NotReady");
passed &= Expect(unbound_system.ReadBmi088RawSample(raw) == ResultCode::NotReady,
                 "unbound BMI088 sampling must report NotReady");
passed &= Expect(!unbound_system.IsBmi088Initialized(),
                 "unbound BMI088 must not report initialized");
```

- [ ] **Step 2: Run the Application contract and verify RED**

Run: `powershell -ExecutionPolicy Bypass -File .\Tools\verify_application_contract.ps1`

Expected: compilation failure for the missing methods.

- [ ] **Step 3: Add thin forwarding methods**

```cpp
ResultCode Class_DartSystem::InitializeBmi088(Bmi088DelayMsFunction delay_ms)
{
    const ResultCode result = bmi088_.Initialize(delay_ms);
    StoreLastError(result);
    return result;
}
```

Use the same pattern for `ReadBmi088RawSample`; the const query directly returns `bmi088_.IsInitialized()`.

- [ ] **Step 4: Update Application and Framework contract terms**

Require the three new signatures while preserving the rule that Application contains no HAL types or calls.

- [ ] **Step 5: Run Application and Framework contracts and verify GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\Tools\verify_application_contract.ps1
powershell -ExecutionPolicy Bypass -File .\Tools\verify_framework_contract.ps1
```

Expected: both scripts report their contract checks passed.

---

### Task 4: Schedule 200 Hz sampling and expose Keil Watch symbols

**Files:**
- Modify: `../User/Task/task_bridge.cpp`
- Modify: `Tools/verify_application_contract.ps1`
- Modify: `Tools/verify_framework_contract.ps1`

**Interfaces:**
- Consumes: `Class_DartSystem` BMI088 methods from Task 3 and existing TIM5 millisecond timebase.
- Produces: `bmi088_init_result`, `bmi088_sample_result`, `bmi088_raw_sample`, and `bmi088_sample_count` Watch symbols.

- [ ] **Step 1: Add failing contract requirements for the task wiring**

Require the four Watch symbols, the HAL delay wrapper, initialization call, independent five-tick divider, sampling call, success-only field copy, and success-only count increment. Keep `.Transfer(`, `.SetLow(`, and `.SetHigh(` forbidden in Task code.

- [ ] **Step 2: Run Application and Framework contracts and verify RED**

Expected: missing BMI088 initialization and polling terms.

- [ ] **Step 3: Add Watch state and delay adapter**

```cpp
volatile ResultCode bmi088_init_result = ResultCode::NotReady;
volatile ResultCode bmi088_sample_result = ResultCode::NotReady;
volatile Bmi088RawSample bmi088_raw_sample = {};
volatile uint32_t bmi088_sample_count = 0U;

static void DelayBmi088Ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}
```

Add `StoreBmi088RawSample()` that copies all eight fields individually into the volatile Watch object.

- [ ] **Step 4: Initialize BMI088 without blocking unrelated bring-up**

After the existing Probe, call `InitializeBmi088(DelayBmi088Ms)` only when Probe is `Ok`. Record the result in `bmi088_init_result`; do not call `StoreFirstTaskInitError()` for this bring-up probe stage.

- [ ] **Step 5: Add a dedicated 200 Hz poll divider**

Every fifth processed TIM5 tick, if `bmi088_init_result == ResultCode::Ok`, read into a local sample. Always update `bmi088_sample_result`; copy fields and increment `bmi088_sample_count` only on `Ok`.

- [ ] **Step 6: Run contracts and verify GREEN**

Run the Application and Framework scripts again. Expected: both pass.

---

### Task 5: Full build and artifact inspection

**Files:**
- Verify only: `A133P_Dart.uvprojx`
- Inspect: `A133P_Dart/A133P_Dart.build_log.htm`
- Inspect: `A133P_Dart/A133P_Dart.map`

**Interfaces:**
- Consumes: all changes from Tasks 1-4.
- Produces: a Keil-loadable image with retained Watch symbols.

- [ ] **Step 1: Run all focused contract checks**

```powershell
powershell -ExecutionPolicy Bypass -File .\Tools\verify_device_contract.ps1
powershell -ExecutionPolicy Bypass -File .\Tools\verify_application_contract.ps1
powershell -ExecutionPolicy Bypass -File .\Tools\verify_framework_contract.ps1
```

Expected: all scripts pass with no host compiler warning or test failure.

- [ ] **Step 2: Build the Keil target**

Run the existing approved hidden Keil command for target `A133P_Dart` and inspect its generated log.

Expected: ARMCLANG exits zero with zero errors.

- [ ] **Step 3: Confirm retained symbols and code paths**

Search the Map file for `Class_Bmi088::Initialize`, `Class_Bmi088::ReadRawSample`, `bmi088_raw_sample`, and `bmi088_sample_count`.

- [ ] **Step 4: Remove only research temporaries**

Delete `bmi088_datasheet_temp.pdf`, its extracted text, and downloaded `bmi08*_temp.*` files. Do not remove or reset any existing project artifacts or unrelated user modifications.

- [ ] **Step 5: Hand off exact Keil Watch validation**

Ask the user to flash the built image and watch:

```text
bmi088_probe_result
bmi088_init_result
bmi088_sample_result
bmi088_sample_count
bmi088_raw_sample.acc_x / acc_y / acc_z
bmi088_raw_sample.gyro_x / gyro_y / gyro_z
bmi088_raw_sample.temperature_raw
bmi088_raw_sample.temperature_deg_c
```

Expected on hardware: the first three results are `Ok`, the counter changes about 200 times per second, one static ACC axis carries gravity, GYRO axes sit near zero when still, and the corresponding axes respond to manual motion.
