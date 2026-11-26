# UART Fixed-Frame Protocol Design

## Scope

Implement the lower-computer side of a full-duplex USART1 protocol. The lower computer receives one fixed 14-byte overload-command frame and can transmit one fixed 20-byte six-axis IMU frame. The two directions intentionally use independent layouts and independent frame headers.

Live BMI088 sampling is outside this protocol change because the current BMI088 device only probes chip IDs. The protocol exposes an IMU-send entry point that accepts an already valid `ImuData` sample.

## Wire conventions

- USART1 remains 115200 baud, 8 data bits, no parity, 1 stop bit.
- Multi-byte integers are little-endian two's-complement values.
- CRC is CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value `0xFFFF`, no reflection, final XOR `0x0000`.
- The transmitted CRC is little-endian: low byte first.
- CRC covers only the bytes between the two-byte header and the CRC field.
- Both reserved bytes are transmitted as zero. A receiver ignores their values.
- A failed header, footer, or CRC check does not publish a command. The stream parser resumes searching for its receive header.

## Host-to-controller overload command

The fixed frame is 14 bytes:

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | Header 0 | `0xAA` |
| 1 | 1 | Header 1 | `0x55` |
| 2 | 2 | Roll overload | signed `int16_t`, `0.01 g/LSB` |
| 4 | 2 | Yaw overload | signed `int16_t`, `0.01 g/LSB` |
| 6 | 2 | Pitch overload | signed `int16_t`, `0.01 g/LSB` |
| 8 | 1 | Reserved 0 | ignored |
| 9 | 1 | Reserved 1 | ignored |
| 10 | 2 | CRC16 | over offsets 2 through 9 |
| 12 | 1 | Footer 0 | `0x0D` |
| 13 | 1 | Footer 1 | `0x0A` |

The decoded physical value is `raw / 100.0f` g. The protocol applies no narrower business limit than the `int16_t` representation. Physical overload commands remain separate from the existing normalized `BodyControlCommand`, whose `[-1, +1]` semantics are unchanged.

## Controller-to-host IMU telemetry

The fixed frame is 20 bytes:

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | Header 0 | `0x55` |
| 1 | 1 | Header 1 | `0xAA` |
| 2 | 2 | Acc X | signed `int16_t`, `0.01 m/s^2/LSB` |
| 4 | 2 | Acc Y | signed `int16_t`, `0.01 m/s^2/LSB` |
| 6 | 2 | Acc Z | signed `int16_t`, `0.01 m/s^2/LSB` |
| 8 | 2 | Gyro X | signed `int16_t`, `0.1 deg/s/LSB` |
| 10 | 2 | Gyro Y | signed `int16_t`, `0.1 deg/s/LSB` |
| 12 | 2 | Gyro Z | signed `int16_t`, `0.1 deg/s/LSB` |
| 14 | 1 | Reserved 0 | `0x00` |
| 15 | 1 | Reserved 1 | `0x00` |
| 16 | 2 | CRC16 | over offsets 2 through 15 |
| 18 | 1 | Footer 0 | `0x0D` |
| 19 | 1 | Footer 1 | `0x0A` |

Acceleration is encoded as `round(m/s^2 * 100)`. The internal gyroscope value is radians per second, so it is encoded as `round(rad/s * 180/pi * 10)`. NaN, infinity, or a value outside the `int16_t` representation rejects the send instead of wrapping.

## Software structure

- `Common/host_overload_command.h` defines the physical overload command type.
- `Protocol/host_serial_protocol.h/.cpp` owns constants, CRC, fixed-frame parsing, and IMU encoding. It has no HAL dependency.
- `Device/dev_host_serial.h/.cpp` connects the pure protocol to `Class_UartDma`, drains received bytes in task context, retains the latest valid command, and uses two persistent buffers for asynchronous DMA transmission.
- `Task/task_bridge.cpp` owns and binds the host-serial device, polls it from `Task_Loop`, and exposes C-callable snapshot/send functions. UART ISR work remains limited to notifying the circular DMA driver.

## Verification

Host-side C++ tests cover the CRC reference vector `123456789 -> 0x29B1`, signed/little-endian command decoding, fragmented and concatenated frames, noise recovery, bad CRC/footer rejection, IMU scaling, reserved bytes, invalid floats, UART draining, overflow recovery, and safe double-buffered DMA transmission.
