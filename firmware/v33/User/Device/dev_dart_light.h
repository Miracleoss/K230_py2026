#ifndef USER_DEVICE_DEV_DART_LIGHT_H
#define USER_DEVICE_DEV_DART_LIGHT_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Common/dart_light_status.h"
#include "Config/dart_light_config.h"
#include "Driver/drv_uart_half_duplex.h"
#include "Protocol/dart_light_protocol.h"

/**
 * @brief 飞镖发光装置设备类
 *
 * 负责与飞镖发光装置进行单线半双工通信协调。
 * 基于事务状态机（一问一答、非阻塞），自动处理命令发送、等待回包、超时重试及帧解析。
 */
class Class_DartLight {
public:
    Class_DartLight();

    /// 绑定底层半双工串口驱动
    ResultCode Bind(Class_UartHalfDuplex* uart);

    /// 查询设备是否已成功绑定
    bool IsBound() const;

    /// 请求设置发光阵营颜色（红/蓝）
    ResultCode RequestSetTeam(DartLightTeam team);

    /// 请求查询装置状态与错误码（接口保留供随时调用）
    ResultCode RequestStatus();

    /// 主循环周期轮询驱动（建议每毫秒调用一次）
    ResultCode Poll(uint64_t now_us);

    /// 查询通信事务是否正在忙碌（发送中或等待回包中）
    bool IsBusy() const;

    /// 获取目标请求阵营
    DartLightTeam GetTargetTeam() const;

    /// 获取当前装置已确认的阵营
    DartLightTeam GetCurrentTeam() const;

    /// 获取最近一次设置颜色的结果（Ok/Timeout/HardwareError/NotReady）
    ResultCode GetLastSetTeamResult() const;

    /// 获取最近一次状态查询的结果
    ResultCode GetLastQueryResult() const;

    /// 获取最近一次查询到的状态数据
    const DartLightStatus& GetStatus() const;

    // 诊断统计接口
    uint32_t GetTimeoutCount() const;
    uint32_t GetSetFailCount() const;
    uint32_t GetCrcErrorCount() const;
    uint32_t GetFormatErrorCount() const;

private:
    enum class State {
        Idle,
        WaitResponse
    };

    Class_UartHalfDuplex* uart_;
    Class_DartLightFrameParser parser_;
    DartLightStatus latest_status_;

    State state_;
    DartLightTeam target_team_;
    DartLightTeam current_team_;
    bool set_team_pending_;
    bool query_status_pending_;
    uint8_t retry_count_;

    uint8_t expected_cmd_;
    uint64_t request_sent_us_;
    uint64_t last_transaction_end_us_;

    ResultCode last_set_team_result_;
    ResultCode last_query_result_;

    uint8_t tx_frame_[16];

    uint32_t timeout_count_;
    uint32_t set_fail_count_;
};

#endif /* USER_DEVICE_DEV_DART_LIGHT_H */

