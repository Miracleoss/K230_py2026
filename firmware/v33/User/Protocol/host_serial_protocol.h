/**
 * @file host_serial_protocol.h
 * @brief 飞控与上位机之间的二进制协议。
 *
 * @details 所有 16 位、32 位数值和 CRC 字段均采用小端字节序。
 * CRC 算法为 CRC-16/CCITT-FALSE（多项式 0x1021，初始值 0xFFFF，
 * 不反射、不异或输出）。
 *
 * 上位机到飞控的控制帧（14 字节）：
 * @code
 * 偏移  大小  字段                 编码
 * 0     2     帧头                 AA 55
 * 2     1     是否已经发射         uint8, 0: 未发射, 非0: 已发射
 * 3     1     是否识别到基地绿灯   uint8, 0: 未识别, 非0: 已识别
 * 4     2     偏航过载             int16, 0.01 g/LSB
 * 6     2     俯仰过载             int16, 0.01 g/LSB
 * 8     1     保留 0               由解码器保留
 * 9     1     保留 1               由解码器保留
 * 10    2     CRC-16               覆盖字节 2 到 9
 * 12    2     帧尾                 0D 0A
 * @endcode
 *
 * 飞控到上位机的 IMU 帧（29 字节）：
 * @code
 * 偏移  大小  字段                 编码
 * 0     2     帧头                 55 AA
 * 2     2     AccX                 int16, 0.01 m/s^2/LSB
 * 4     2     AccY                 int16, 0.01 m/s^2/LSB
 * 6     2     AccZ                 int16, 0.01 m/s^2/LSB
 * 8     2     GyroX                int16, 0.1 deg/s/LSB
 * 10    2     GyroY                int16, 0.1 deg/s/LSB
 * 12    2     GyroZ                int16, 0.1 deg/s/LSB
 * 14    1     P instruction        uint8, body pitch [-1, 1] mapped to 0..255
 * 15    1     Y instruction        uint8, body yaw [-1, 1] mapped to 0..255
 * 16    2     Acc_Yaw_PID_output   int16, 0.001 rad/s/LSB //yaw过载环 PID 输出
 * 18    2     Acc_Pitch_PID_output int16, 0.001 rad/s/LSB //pitch过载环 PID 输出
 * 20    4     Imu_index          uint32, 0..2^32-1 //IMU帧序列
 * 24    1     Trigger_flag         uint8, 0: not triggered, 1: triggered //当前图像帧是否触发
 * 25    2     CRC-16               覆盖字节 2 到 24
 * 27    2     帧尾                 0D 0A
 * @endcode
 */
#ifndef USER_PROTOCOL_HOST_SERIAL_PROTOCOL_H
#define USER_PROTOCOL_HOST_SERIAL_PROTOCOL_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Common/control_command.h"
#include "Common/host_overload_command.h"
#include "Common/imu_data.h"

/** 一条上位机到飞控控制帧的总字节数。 */
static const uint16_t kHostControlFrameSize = 14U;

/** 一条飞控到上位机 IMU 帧的总字节数。 */
static const uint16_t kHostImuFrameSize = 29U;

/** JustFloat uses six 32-bit little-endian IEEE754 values and a 4-byte tail. */
static const uint16_t kVofaJustFloatChannelCount = 6U;
static const uint16_t kVofaJustFloatFrameSize =
    static_cast<uint16_t>(kVofaJustFloatChannelCount * 4U + 4U);

/**
 * @brief 对一段连续字节序列计算 CRC-16/CCITT-FALSE。
 * @param data 指向首字节的指针。仅当 @p length 为零时允许为空。
 * @param length 参与计算的字节数。
 * @return 计算得到的 CRC。当指针为空且长度非零时返回零。
 */
uint16_t HostProtocolCrc16(const uint8_t* data, uint16_t length);

/**
 * @brief 用于上位机到飞控过载控制帧的增量解析器。
 *
 * 按到达顺序送入接收到的字节。解析器会丢弃前导噪声，在 AA 55 帧头上
 * 同步，并且仅在帧尾和 CRC 均已校验通过后才发布一条命令。
 */
class Class_HostControlFrameParser {
public:
    /** @brief 创建一个空解析器，并将所有诊断计数器清零。 */
    Class_HostControlFrameParser();

    /**
     * @brief 向解析器送入一个接收到的字节。
     * @param byte 来自 UART 接收流的下一字节。
     * @param command 仅在解码出一条完整有效帧时才更新的目标。
     * @return 若 @p command 被更新则返回 true；否则返回 false。
     */
    bool Feed(uint8_t byte, HostOverloadCommand& command);

    /**
     * @brief 丢弃一条只接收到部分的帧，并等待新的帧头。
     * @note 诊断计数器会被有意保留。
     */
    void Reset();

    /** @return 因 CRC 无效而被拒绝的完整帧数量。 */
    uint32_t GetCrcErrorCount() const;

    /** @return 因帧尾无效而被拒绝的完整帧数量。 */
    uint32_t GetFormatErrorCount() const;

private:
    /**
     * @brief 校验并解码存储在 @ref frame_ 中的完整帧。
     * @param command 解码出的物理过载值的目标。
     * @return 当帧尾和 CRC 均有效时返回 true；否则返回 false。
     */
    bool ValidateAndDecode(HostOverloadCommand& command);

    uint8_t frame_[kHostControlFrameSize];  ///< 正在接收的控制帧缓冲区。
    uint16_t frame_index_;                  ///< 下一个字节写入的索引。
    uint32_t crc_error_count_;              ///< 累积的 CRC 拒绝计数。
    uint32_t format_error_count_;           ///< 累积的帧尾拒绝计数。
};

/**
 * @brief 将一份物理 IMU 采样编码为一条飞控到上位机的帧。
 *
 * 加速度输入以 m/s^2 表示，按 0.01 m/s^2 每 LSB 编码。陀螺仪输入以
 * rad/s 表示，先转换为 deg/s，再按 0.1 deg/s 每 LSB 编码。BodyControlCommand
 * 的 pitch/yaw 输入按 [-1, 1] 线性映射到 [0, 255]；无效指令发送安全中位值
 * 128。数值会四舍五入到最接近的整数；NaN 和超出范围的值会被拒绝，而不是回绕。
 * 过载环角速度输出按 0.001 rad/s 每 LSB 编码，超出 int16 范围时饱和到边界。
 * Imu_index 直接取自 imu.data_ready_seq，Trigger_flag 当前恒为 1。
 *
 * @param imu 以 ImuData 定义的物理单位表示的有效 IMU 采样。
 * @param command 实际执行的机体控制指令；有效时编码其 pitch/yaw 字段。
 * @param overload_yaw_rate_radps 偏航过载环 PI 输出（角速度指令，rad/s）。
 * @param overload_pitch_rate_radps 俯仰过载环 PI 输出（角速度指令，rad/s）。
 * @param frame 编码后帧的目标缓冲区。
 * @param capacity @p frame 的可用字节大小。
 * @return 成功时返回 ResultCode::Ok；当采样、有效指令、目标指针或容量
 *         无效时返回 ResultCode::InvalidData。
 */
ResultCode HostProtocolEncodeImuFrame(
    const ImuData& imu,
    const BodyControlCommand& command,
    float overload_yaw_rate_radps,
    float overload_pitch_rate_radps,
    uint8_t* frame,
    uint16_t capacity);

/**
 * Encode six SI-unit IMU channels for VOFA+ JustFloat.
 *
 * The frame contains ACC XYZ in m/s^2 followed by GYRO XYZ in rad/s, all as
 * little-endian IEEE754 float32 values, followed by the JustFloat sync tail
 * 00 00 80 7F. It intentionally has no Host header or CRC.
 */
ResultCode VofaProtocolEncodeJustFloatFrame(
    const ImuData& imu,
    uint8_t* frame,
    uint16_t capacity);

#endif /* USER_PROTOCOL_HOST_SERIAL_PROTOCOL_H */
