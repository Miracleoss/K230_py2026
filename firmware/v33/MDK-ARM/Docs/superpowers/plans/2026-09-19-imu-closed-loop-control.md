# IMU 闭环控制接入 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 将主机俯仰/偏航过载指令、机体系 IMU 比力和角速度接入双外环 PI、三轴角速度 PID，并通过现有 BodyControlCommand/X 尾混控输出舵机控制。

**Architecture:** 新增无 HAL 依赖的 Class_ClosedLoopController，由 Class_DartSystem 聚合并暴露 UpdateClosedLoop 入口。Task_Loop 负责主机指令锁存、主机/IMU 新鲜度和 5 ms 新样本调度；控制器只在新 IMU 样本到达时更新。ctrl_R/P/Y 使用 [0,1] 且 0.5 为零控制，进入现有 BodyControlCommand 前统一映射为 2*ctrl-1。

**Tech Stack:** ARMCLANG/Keil 工程、C++11、MinGW g++ 独立行为测试、现有 PowerShell contract checks。

**Spec:** Docs/ClosedLoop_IMU_Control_PreChange_Report.md

## Global Constraints

- IMU 控制内部角速度统一使用 rad/s；普通 Host 遥测帧继续使用 0.1 deg/s/LSB，VOFA+ 继续使用 rad/s。
- 比力控制变量使用 f_x/f_y/f_z 语义；指令变量使用 _c；实测反馈变量使用 _meas；最终 R/P/Y 输出使用 ctrl_R/ctrl_P/ctrl_Y。
- n_y_c = 主机 pitch_overload_g，n_z_c = 主机 yaw_overload_g；roll_overload_g 不进入本闭环。
- n_y_meas = f_y / 9.81，n_z_meas = f_z / 9.81；omega_x_c 固定为 0。
- 只有 data_ready_seq 或 sample_timestamp_ms 变化的新 IMU 样本才推进 PI/PID。
- ctrl_* 的 0.5 映射到现有 BodyControlCommand 的 0；所有无效/超时路径回中。
- Algorithm 层不得包含 HAL、Driver、Device、Application 或 Task 依赖；不得动态分配内存。
- 不实现当前空壳 Class_AttitudeEstimator；不改变主机帧头、长度、CRC 和字段顺序。
- 飞控增益不能凭空猜测；控制器接受显式配置，硬件默认配置采用零增益安全输出，测试使用非零测试配置验证数学行为。
- 新增源文件必须加入 MDK-ARM/A133P_Dart.uvprojx 和 EIDE 工程文件。

## Review Focus

- 同一 IMU 样本重复调用 Update：必须不重复积分/微分；由 Task 样本门控测试覆盖。
- dt_s 为零、回退、过大或 IMU invalid：必须 Reset 并输出无效/中位控制；由控制器测试覆盖。
- 主机过载字段错接：pitch 必须进入 n_y_c，yaw 必须进入 n_z_c；由 Task 静态契约和映射测试覆盖。
- PI/PID 饱和和积分增长：输出必须限幅，积分必须 anti-windup；由控制器测试覆盖。
- [0,1] 到 [-1,1] 中位转换：ctrl=0.5 必须得到 BodyControlCommand 三轴 0；由控制器和应用层测试覆盖。

---

### Task 1: 新增控制配置与纯闭环算法

**Files:**
- Create: User/Config/control_config.h
- Create: User/Algorithm/alg_closed_loop_controller.h
- Create: User/Algorithm/alg_closed_loop_controller.cpp
- Create: Tools/test_closed_loop_controller.cpp
- Modify: Tools/verify_algorithm_contract.ps1
- Modify: MDK-ARM/A133P_Dart.uvprojx
- Modify: MDK-ARM/.eide/eide.yml

**Interfaces:**
- ClosedLoopControllerConfig MakeDefaultClosedLoopControllerConfig()
- ClosedLoopControlOutput fields: n_y_meas, n_z_meas, omega_x_c, omega_y_c, omega_z_c, ctrl_R, ctrl_P, ctrl_Y, valid.
- Class_ClosedLoopController constructor, explicit config constructor, Reset, and Update(float n_y_c, float n_z_c, const ImuData& imu, float dt_s, ClosedLoopControlOutput& output).
- Config contains gravity, overload input limits, PI gains/output limits, rate PID gains/integral/output limits, dt bounds, and output midpoint. Default gains are zero for safe hardware bring-up; tests construct nonzero configs.

- [ ] Step 1: Write the failing test
  Add a standalone main using the real Class_ClosedLoopController API. Cover zero commands and zero rates producing n_y_meas=0, n_z_meas=0 and ctrl_R/P/Y=0.5; positive n_y_c producing positive omega_y_c while omega_x_c remains 0; positive omega_y_c tracking error producing ctrl_P above 0.5 and capping at 1.0; invalid IMU, dt_s=0, and dt_s above max returning InvalidData with valid=false and ctrl_* at 0.5; repeated timestamp being rejected without advancing the integral.
- [ ] Step 2: Run the test to verify it fails
  Run g++ with C++11, Wall, Wextra, Werror, include path User, test_closed_loop_controller.cpp, and alg_closed_loop_controller.cpp. Expected: compile failure because the new header/source do not exist.
- [ ] Step 3: Write minimal implementation
  Implement finite checks, configured input clamps, f_y/f_z overload calculation, two independent PI states, three independent PID states, conditional anti-windup, dt validation, Reset, and signed-to-[0,1] mapping. Keep all constants in control_config.h and no hardware includes.
- [ ] Step 4: Run the test to verify it passes
  Compile with the same command and execute the temporary binary. Expected: exit code 0 and the test pass message.
- [ ] Step 5: Run Algorithm contract checks
  Run Tools/verify_algorithm_contract.ps1. Expected: the new algorithm files are registered and contain no HAL/Driver dependency.
- [ ] Step 6: Refactor
  Remove duplicated clamp/finite helpers only after the standalone test remains green.
- [ ] Step 7: Commit
  Commit message: feat: add imu overload and rate closed-loop controller.

### Task 2: Aggregate the controller in Class_DartSystem

**Files:**
- Modify: User/Application/app_dart_system.h
- Modify: User/Application/app_dart_system.cpp
- Modify: Tools/test_dart_system_control_output.cpp
- Modify: Tools/verify_application_contract.ps1

**Interfaces:**
- Include Algorithm/alg_closed_loop_controller.h.
- Add ResultCode UpdateClosedLoop(float n_y_c, float n_z_c, const ImuData& imu, float dt_s, BodyControlCommand& command).
- Class_DartSystem owns Class_ClosedLoopController closed_loop_controller_.
- UpdateClosedLoop calls the controller, maps ctrl_R/P/Y as roll=2*ctrl_R-1, pitch=2*ctrl_P-1, yaw=2*ctrl_Y-1, fills timestamp from IMU, and sets command.valid from controller output.

- [ ] Step 1: Write the failing test
  Extend the existing DartSystem behavior test with the UpdateClosedLoop API and assert that neutral ctrl values produce BodyControlCommand roll/pitch/yaw equal to zero. Use the controller configuration injection path for a nonzero mathematical test if exposed; otherwise keep the pure-controller test as the numeric coverage and use an application compile/API assertion.
- [ ] Step 2: Run the test to verify it fails
  Run Tools/verify_application_contract.ps1. Expected: failure because Class_DartSystem has no UpdateClosedLoop or controller member.
- [ ] Step 3: Write minimal implementation
  Add the controller member and UpdateClosedLoop. Keep host timeout logic in Task. Keep ApplyBodyControlCommand unchanged as the final mixer/servo safety boundary.
- [ ] Step 4: Run the test to verify it passes
  Run Tools/verify_application_contract.ps1. Expected: compile and behavior checks pass without changing existing servo neutral behavior.
- [ ] Step 5: Refactor
  Keep the signed mapping in exactly one helper inside app_dart_system.cpp.
- [ ] Step 6: Commit
  Commit message: feat: expose closed-loop update through DartSystem.

### Task 3: Replace test command wiring with sample-gated closed-loop scheduling

**Files:**
- Modify: User/Task/task_bridge.cpp
- Modify: User/Task/task_bridge.h
- Modify: Tools/verify_application_contract.ps1
- Modify: Tools/verify_framework_contract.ps1

**Interfaces:**
- Keep Task_TakeLatestHostOverload and all UART/IMU bridge functions.
- Add task-local latched n_y_c, n_z_c, last_host_command_ms, last_control_sample_seq, and last_control_timestamp_ms.
- Use kHostCommandTimeoutMs, kImuStaleTimeoutMs, and configured overload/rate limits.
- Poll host first, take latest command second, read IMU at the existing five-tick divider, and call DartSystem.UpdateClosedLoop only after a successful new sample.
- On no command yet, host timeout, IMU timeout, invalid sample, invalid dt, or controller failure: reset controller, make command invalid, and let ApplyBodyControlCommand neutralize bound servos.
- Remove test_Roll/test_Pitch/test_Yaw from the runtime command path and remove output_test invocation from the closed-loop path.

- [ ] Step 1: Write the failing static/runtime test
  Add assertions for host_serial.Poll appearing before TakeLatestHostOverload; pitch_overload_g mapping to n_y_c; yaw_overload_g mapping to n_z_c; data_ready_seq/timestamp gating; removal of test variables from the runtime command; and Reset plus invalid command on timeout.
- [ ] Step 2: Run the tests to verify they fail
  Run verify_application_contract.ps1 and verify_framework_contract.ps1. Expected: new assertions fail against current test-variable wiring.
- [ ] Step 3: Write minimal implementation
  Reorder Task_Loop, retain the last valid host command, calculate dt from IMU sample timestamps, and call UpdateClosedLoop exactly once per new sample. Preserve BMI088 Watch/public sample stores and telemetry.
- [ ] Step 4: Run the tests to verify they pass
  Run both scripts again. Expected: all scheduling assertions pass.
- [ ] Step 5: Refactor
  Keep host/IMU freshness checks in small static helpers; do not add work to TIM5 ISR.
- [ ] Step 6: Commit
  Commit message: feat: wire imu closed loop into task scheduler.

### Task 4: Update project manifests and verification contracts

**Files:**
- Modify: MDK-ARM/A133P_Dart.uvprojx
- Modify: MDK-ARM/.eide/eide.yml
- Modify: Tools/verify_algorithm_contract.ps1
- Modify: Tools/verify_application_contract.ps1
- Modify: Tools/verify_framework_contract.ps1
- Create: Tools/verify_closed_loop_controller.ps1

**Interfaces:**
- Keil Algorithm group contains alg_closed_loop_controller.h as FileType 5 and alg_closed_loop_controller.cpp as FileType 8.
- Keil Config group contains control_config.h as FileType 5.
- EIDE lists the same paths.
- The dedicated controller verification script compiles and runs Tools/test_closed_loop_controller.cpp with MinGW g++ and strict warnings.

- [ ] Step 1: Write the failing manifest check
  Add checks for the new paths and run the new script before manifest edits.
- [ ] Step 2: Run the test to verify it fails
  Run verify_closed_loop_controller.ps1. Expected: missing project path or missing test source failure.
- [ ] Step 3: Write minimal manifest/contract implementation
  Add explicit XML/EIDE entries and replace stale Stage 2 assertions that require test-variable command wiring or forbid all PID text. Keep the prohibition on class_pid.* because the implementation remains inside alg_closed_loop_controller.*.
- [ ] Step 4: Run the test to verify it passes
  Run the dedicated script and all existing contract scripts.
- [ ] Step 5: Commit
  Commit message: build: register closed-loop sources and contracts.

### Task 5: Full verification and report update

**Files:**
- Modify: Docs/ClosedLoop_IMU_Control_PreChange_Report.md only for implementation status and final line references.

- [ ] Step 1: Run the focused controller test with verify_closed_loop_controller.ps1.
- [ ] Step 2: Run framework, driver, device, algorithm, application, and host protocol verification scripts.
- [ ] Step 3: Run the configured ARMCLANG/Keil target build and capture its exit code/log.
- [ ] Step 4: Inspect the final diff and confirm no unrelated Core file or generated driver changed.
- [ ] Step 5: Update the report with implemented file/line references and exact verification results.
- [ ] Step 6: Commit message: feat: complete imu based overload to rpy closed loop.

