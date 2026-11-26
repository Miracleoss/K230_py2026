# UART Fixed-Frame Protocol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement tested fixed-frame overload reception and six-axis IMU transmission over the existing USART1 circular-DMA driver.

**Architecture:** A HAL-free protocol module manually packs and unpacks fixed byte layouts. A small host-serial device drains the existing UART ring and owns persistent double TX buffers, while the task bridge only composes the objects and exposes physical-data entry points.

**Tech Stack:** C++11, STM32F4 HAL, Keil MDK project XML, host-side `g++` behavior tests, PowerShell verification.

**Spec:** `Docs/superpowers/specs/2026-09-05-uart-fixed-frame-protocol-design.md`

## Global Constraints

- Preserve USART1 at 115200 8N1 and preserve circular RX DMA.
- Do not parse frames or calculate CRC in interrupt context.
- Do not reinterpret physical overload values as normalized `BodyControlCommand` values.
- Do not transmit fake IMU samples while live BMI088 acquisition is unavailable.
- Preserve all unrelated user worktree changes.

---

### Task 1: Pure protocol codec

**Files:**
- Create: `../User/Common/host_overload_command.h`
- Create: `../User/Protocol/host_serial_protocol.h`
- Create: `../User/Protocol/host_serial_protocol.cpp`
- Test: `Tools/test_host_serial_protocol.cpp`

**Interfaces:**
- Produces: `uint16_t HostProtocolCrc16(const uint8_t*, uint16_t)`.
- Produces: `Class_HostControlFrameParser::Feed(uint8_t, HostOverloadCommand&)` and `Reset()`.
- Produces: `ResultCode HostProtocolEncodeImuFrame(const ImuData&, uint8_t*, uint16_t)`.

- [ ] **Step 1: Write failing protocol tests**

Test the CCITT-FALSE reference vector, the exact 14-byte receive layout, signed values, noise/splits/back-to-back frames, CRC/footer failures, the exact 20-byte transmit layout, SI-to-wire scaling, zero reserved bytes, and invalid/out-of-range floats.

- [ ] **Step 2: Run the test and confirm the missing interfaces fail compilation**

Run:

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -I ..\User Tools\test_host_serial_protocol.cpp ..\User\Protocol\host_serial_protocol.cpp -o $env:TEMP\a133p_host_protocol_test.exe
```

Expected: compilation fails because the protocol files/interfaces do not exist.

- [ ] **Step 3: Implement the minimal codec**

Use explicit byte shifts rather than packed structs or pointer casts. Decode command values with `static_cast<int16_t>(uint16_value)` and scale by `0.01f`. Reject IMU values unless finite and representable after scaling.

- [ ] **Step 4: Run the protocol test**

Run the compile command above, then:

```powershell
& $env:TEMP\a133p_host_protocol_test.exe
```

Expected: prints `Host serial protocol checks passed.` and exits 0.

### Task 2: UART-facing host serial device

**Files:**
- Create: `../User/Device/dev_host_serial.h`
- Create: `../User/Device/dev_host_serial.cpp`
- Test: `Tools/test_host_serial_device.cpp`

**Interfaces:**
- Consumes: `Class_UartDma`, `Class_HostControlFrameParser`, and `HostProtocolEncodeImuFrame` from Task 1.
- Produces: `Bind`, `Poll`, `TakeLatestOverload`, `SendImuData`, and diagnostic counter accessors.

- [ ] **Step 1: Write failing device tests**

Use the HAL UART stubs to verify binding, ring draining, valid-command publication, no publication for invalid frames, parser reset after ring overflow, DMA busy propagation, and alternating persistent TX buffers.

- [ ] **Step 2: Run the test and confirm the missing class fails compilation**

Compile the test with `drv_uart_dma.cpp`, `host_serial_protocol.cpp`, and `dev_host_serial.cpp`; expect failure before the device exists.

- [ ] **Step 3: Implement the device**

Keep all parsing in `Poll()`. Encode into `tx_buffers_[next_tx_buffer_]`, call `SendAsync`, and toggle the buffer index only after `ResultCode::Ok`, ensuring an in-flight DMA buffer is never overwritten.

- [ ] **Step 4: Run the device and protocol tests**

Expected: both binaries exit 0 with their success messages.

### Task 3: Task and Keil integration

**Files:**
- Modify: `../User/Task/task_bridge.h`
- Modify: `../User/Task/task_bridge.cpp`
- Modify: `A133P_Dart.uvprojx`
- Create: `Tools/verify_host_serial_protocol.ps1`

**Interfaces:**
- Consumes: `Class_HostSerial` from Task 2.
- Produces: `Task_GetLatestHostOverload(...)` for a coherent physical-g snapshot and `Task_SendImuData(const ImuData*)` for valid sampled data.

- [ ] **Step 1: Add a failing integration verifier**

Require the new files, exact Keil group entries, binding order, task-context polling, unchanged RX callback behavior, and the two public task APIs.

- [ ] **Step 2: Run the verifier and confirm it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File Tools\verify_host_serial_protocol.ps1
```

Expected: failure for missing integration terms.

- [ ] **Step 3: Integrate without changing normalized flight control semantics**

Bind the host-serial device after the UART driver, poll it in `Task_Loop`, cache the latest physical overload command independently from `test_Roll/test_Pitch/test_Yaw`, and leave the ISR as one `OnRxEventIsr` notification. Add protocol/device files to the Keil project.

- [ ] **Step 4: Run all new host tests and the integration verifier**

Expected: all new tests pass. Historical stage-contract scripts that deliberately forbid protocol functionality are not protocol acceptance tests and are left unchanged because they contain user modifications.

### Task 4: Final build verification

**Files:**
- Verify only: all files above and the existing Keil target.

**Interfaces:**
- Consumes: completed Tasks 1 through 3.
- Produces: evidence that the host-test build and embedded target compile.

- [ ] **Step 1: Run the focused PowerShell verifier from a clean command invocation**

Expected: protocol, device, and integration checks all pass.

- [ ] **Step 2: Build the Keil target with the locally available command-line tool**

Use the project toolchain already installed for `A133P_Dart.uvprojx`. Expected: zero compiler/linker errors.

- [ ] **Step 3: Inspect the final diff**

Confirm only the protocol implementation, its tests/docs, Keil project membership, and the narrow task-bridge integration are present; do not overwrite unrelated user edits.
