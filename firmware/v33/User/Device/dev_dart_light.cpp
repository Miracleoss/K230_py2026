#include "Device/dev_dart_light.h"

Class_DartLight::Class_DartLight()
    : uart_(nullptr),
      parser_(),
      latest_status_(),
      state_(State::Idle),
      target_team_(kDartLightDefaultTeam),
      current_team_(kDartLightDefaultTeam),
      set_team_pending_(false),
      query_status_pending_(false),
      retry_count_(0U),
      expected_cmd_(0U),
      request_sent_us_(0ULL),
      last_transaction_end_us_(0ULL),
      last_set_team_result_(ResultCode::NotReady),
      last_query_result_(ResultCode::NotReady),
      tx_frame_(),
      timeout_count_(0U),
      set_fail_count_(0U)
{
    latest_status_.valid = false;
}

ResultCode Class_DartLight::Bind(Class_UartHalfDuplex* uart)
{
    if (uart == nullptr || !uart->IsBound()) {
        return ResultCode::ConfigError;
    }

    uart_ = uart;
    parser_.Reset();
    state_ = State::Idle;
    set_team_pending_ = false;
    query_status_pending_ = false;
    retry_count_ = 0U;
    latest_status_ = DartLightStatus();
    latest_status_.valid = false;
    last_set_team_result_ = ResultCode::NotReady;
    last_query_result_ = ResultCode::NotReady;
    return ResultCode::Ok;
}

bool Class_DartLight::IsBound() const
{
    return uart_ != nullptr;
}

ResultCode Class_DartLight::RequestSetTeam(DartLightTeam team)
{
    target_team_ = team;
    set_team_pending_ = true;
    retry_count_ = 0U;
    last_set_team_result_ = ResultCode::NotReady;
    return ResultCode::Ok;
}

ResultCode Class_DartLight::RequestStatus()
{
    query_status_pending_ = true;
    last_query_result_ = ResultCode::NotReady;
    return ResultCode::Ok;
}

ResultCode Class_DartLight::Poll(uint64_t now_us)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    // 1. 读取接收流中所有可用字节送入协议解析器
    while (uart_->Available() > 0U) {
        uint8_t byte = 0U;
        if (uart_->ReadByte(byte) != ResultCode::Ok) {
            break;
        }

        DartLightFrame frame = {};
        if (parser_.Feed(byte, frame)) {
            // 成功解析到一帧完整数据
            if (state_ == State::WaitResponse && frame.cmd == expected_cmd_) {
                if (expected_cmd_ == kDartLightCmdSetTeam) {
                    bool ack_ok = false;
                    DartLightDecodeSetTeamAck(frame, ack_ok);
                    if (ack_ok) {
                        last_set_team_result_ = ResultCode::Ok;
                        current_team_ = target_team_;
                        set_team_pending_ = false;
                    } else {
                        last_set_team_result_ = ResultCode::HardwareError;
                        ++set_fail_count_;
                        if (retry_count_ >= kDartLightSetTeamMaxRetries) {
                            set_team_pending_ = false;
                        }
                    }
                } else if (expected_cmd_ == kDartLightCmdQueryStatus) {
                    DartLightDecodeStatus(frame, latest_status_);
                    latest_status_.update_timestamp_us = now_us;
                    latest_status_.valid = true;
                    last_query_result_ = ResultCode::Ok;
                    query_status_pending_ = false;
                }

                state_ = State::Idle;
                last_transaction_end_us_ = now_us;
            }
        }
    }

    // 2. 若在等待响应，检查是否超时（规格最长 2 ms，默认配 5 ms 裕量）
    if (state_ == State::WaitResponse) {
        if (now_us - request_sent_us_ >= kDartLightResponseTimeoutUs) {
            ++timeout_count_;
            if (expected_cmd_ == kDartLightCmdSetTeam) {
                last_set_team_result_ = ResultCode::Timeout;
                if (retry_count_ >= kDartLightSetTeamMaxRetries) {
                    set_team_pending_ = false;
                }
            } else {
                last_query_result_ = ResultCode::Timeout;
                query_status_pending_ = false;
            }

            state_ = State::Idle;
            last_transaction_end_us_ = now_us;
        }
    }

    // 3. 空闲态：检查指令间隔保护（防止发送过密冲突），发送待处理命令
    if (state_ == State::Idle && !uart_->IsTxBusy()) {
        const bool gap_satisfied = (last_transaction_end_us_ == 0ULL) ||
            (now_us - last_transaction_end_us_ >= kDartLightInterCommandGapUs);

        if (gap_satisfied) {
            if (set_team_pending_) {
                ++retry_count_;
                uint16_t out_len = 0U;
                const ResultCode enc_result =
                    DartLightEncodeSetTeam(target_team_, tx_frame_, sizeof(tx_frame_), out_len);
                if (enc_result == ResultCode::Ok) {
                    uart_->FlushRx();
                    parser_.Reset();
                    const ResultCode tx_result = uart_->SendAsync(tx_frame_, out_len);
                    if (tx_result == ResultCode::Ok) {
                        expected_cmd_ = kDartLightCmdSetTeam;
                        request_sent_us_ = now_us;
                        state_ = State::WaitResponse;
                    }
                }
            } else if (query_status_pending_) {
                uint16_t out_len = 0U;
                const ResultCode enc_result =
                    DartLightEncodeQueryStatus(tx_frame_, sizeof(tx_frame_), out_len);
                if (enc_result == ResultCode::Ok) {
                    uart_->FlushRx();
                    parser_.Reset();
                    const ResultCode tx_result = uart_->SendAsync(tx_frame_, out_len);
                    if (tx_result == ResultCode::Ok) {
                        expected_cmd_ = kDartLightCmdQueryStatus;
                        request_sent_us_ = now_us;
                        state_ = State::WaitResponse;
                        query_status_pending_ = false;
                    }
                }
            }
        }
    }

    return (state_ == State::Idle && !set_team_pending_ && !query_status_pending_)
               ? ResultCode::Ok
               : ResultCode::NotReady;
}

bool Class_DartLight::IsBusy() const
{
    return (state_ != State::Idle) || set_team_pending_ || query_status_pending_;
}

DartLightTeam Class_DartLight::GetTargetTeam() const
{
    return target_team_;
}

DartLightTeam Class_DartLight::GetCurrentTeam() const
{
    return current_team_;
}

ResultCode Class_DartLight::GetLastSetTeamResult() const
{
    return last_set_team_result_;
}

ResultCode Class_DartLight::GetLastQueryResult() const
{
    return last_query_result_;
}

const DartLightStatus& Class_DartLight::GetStatus() const
{
    return latest_status_;
}

uint32_t Class_DartLight::GetTimeoutCount() const
{
    return timeout_count_;
}

uint32_t Class_DartLight::GetSetFailCount() const
{
    return set_fail_count_;
}

uint32_t Class_DartLight::GetCrcErrorCount() const
{
    return parser_.GetCrcErrorCount();
}

uint32_t Class_DartLight::GetFormatErrorCount() const
{
    return parser_.GetFormatErrorCount();
}

