#ifndef USER_PROTOCOL_DART_LIGHT_PROTOCOL_H
#define USER_PROTOCOL_DART_LIGHT_PROTOCOL_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Common/dart_light_status.h"

static const uint8_t  kDartLightHeader           = 0xAA;
static const uint8_t  kDartLightCmdQueryStatus   = 0x01;
static const uint8_t  kDartLightCmdSetTeam       = 0x02;
static const uint16_t kDartLightMaxPayloadSize   = 32U;
static const uint16_t kDartLightMaxFrameSize     = 4U + kDartLightMaxPayloadSize; // Header(1) + CMD(1) + LEN(1) + DATA(N) + CRC(1)

/**
 * @brief 解析出的完整通信帧结构
 */
struct DartLightFrame {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[kDartLightMaxPayloadSize];
    uint8_t crc;
};

/**
 * @brief 对一段字节序列计算 CRC-8/MAXIM-DOW
 * @param data 数据指针
 * @param length 数据字节数
 * @return 8 位校验和（初值 0x00，多项式 0x31 反转 0x8C）
 */
uint8_t DartLightCrc8(const uint8_t* data, uint16_t length);

/**
 * @brief 编码“状态及错误码查询”请求帧（AA 01 00 CRC）
 * @param frame 目标缓冲区
 * @param capacity 缓冲区容量（至少 4 字节）
 * @param out_length 编码后的有效帧长度
 */
ResultCode DartLightEncodeQueryStatus(uint8_t* frame, uint16_t capacity, uint16_t& out_length);

/**
 * @brief 编码“设置灯光颜色”请求帧（AA 02 01 team CRC）
 * @param team 目标阵营颜色
 * @param frame 目标缓冲区
 * @param capacity 缓冲区容量（至少 5 字节）
 * @param out_length 编码后的有效帧长度
 */
ResultCode DartLightEncodeSetTeam(DartLightTeam team, uint8_t* frame, uint16_t capacity, uint16_t& out_length);

/**
 * @brief 解码状态查询返回帧
 * @param frame 接收到的完整有效帧
 * @param status 解码出的状态结构体
 */
ResultCode DartLightDecodeStatus(const DartLightFrame& frame, DartLightStatus& status);

/**
 * @brief 解码设置灯光颜色返回帧
 * @param frame 接收到的完整有效帧
 * @param success 操作是否成功（true: 0x00 成功, false: 0x01 失败）
 */
ResultCode DartLightDecodeSetTeamAck(const DartLightFrame& frame, bool& success);

/**
 * @brief 飞镖发光装置通信协议增量解析器
 */
class Class_DartLightFrameParser {
public:
    Class_DartLightFrameParser();

    /**
     * @brief 送入一个接收到的字节
     * @param byte 接收到的单字节
     * @param frame 成功组装出合法帧时的输出目标
     * @return 组装出完整且校验通过的帧时返回 true，否则返回 false
     */
    bool Feed(uint8_t byte, DartLightFrame& frame);

    /// 复位解析状态机
    void Reset();

    uint32_t GetCrcErrorCount() const;
    uint32_t GetFormatErrorCount() const;

private:
    enum class ParseStep {
        WaitHeader,
        WaitCmd,
        WaitLen,
        WaitData,
        WaitCrc
    };

    ParseStep step_;
    uint8_t raw_buffer_[kDartLightMaxFrameSize];
    uint16_t raw_index_;
    uint8_t expected_data_len_;
    uint8_t data_received_count_;

    uint32_t crc_error_count_;
    uint32_t format_error_count_;
};

#endif /* USER_PROTOCOL_DART_LIGHT_PROTOCOL_H */

