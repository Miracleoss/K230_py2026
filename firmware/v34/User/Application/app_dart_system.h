#ifndef USER_APPLICATION_APP_DART_SYSTEM_H
#define USER_APPLICATION_APP_DART_SYSTEM_H

#include <stdint.h>

#include "Algorithm/alg_closed_loop_controller.h"
#include "Algorithm/alg_attitude_estimator.h"
#include "Algorithm/alg_xtail_mixer.h"
#include "Common/common_types.h"
#include "Common/imu_data.h"
#include "Common/system_status.h"
#include "Device/dev_bmi088.h"
#include "Device/dev_imu_heater.h"
#include "Device/dev_servo.h"

class Class_GpioPin;
class Class_PwmChannel;
class Class_SpiBus;
struct BodyControlCommand;
struct ImuHeaterConfig;
struct ServoConfig;

/**
 * @brief 飞控系统顶层聚合类
 *
 * 作为整机飞控系统的顶层容器，聚合并协调以下模块：
 *   - BMI088 惯性传感器（SPI 总线 + 双片选）
 *   - 四路舵机（X 尾翼四舵面）
 *   - IMU 加热器（PWM 通道）
 *   - X 尾翼混控算法（机体控制指令 -> 舵面指令）
 *   - 姿态估计算法（IMU 数据 -> 姿态状态）
 *   - 系统状态（SystemStatus）
 *
 * 本类负责各模块的绑定（Bind）、错误记录，以及机体控制指令到舵机输出的协调；
 * 任务层只负责硬件组合、事件转发和周期调度。
 */
class Class_DartSystem {
public:
    Class_DartSystem();

    /// 绑定 BMI088 惯性传感器（SPI 总线 + 加速度计/陀螺仪片选引脚）
    ResultCode BindBmi088(Class_SpiBus* spi_bus, Class_GpioPin* acc_cs, Class_GpioPin* gyro_cs);
    ResultCode ProbeBmi088(uint8_t& acc_chip_id, uint8_t& gyro_chip_id);
    ResultCode ProbeBmi088WithTrace(
        uint8_t& acc_chip_id,
        uint8_t& gyro_chip_id,
        Bmi088ProbeTrace& trace);
    ResultCode InitializeBmi088(Bmi088DelayMsFunction delay_ms);
    ResultCode ReadBmi088RawSample(Bmi088RawSample& sample);

    /// 绑定 IMU 加热器（PWM 通道）
    ResultCode BindImuHeater(Class_PwmChannel* heater_pwm);

    /// 配置 IMU 加热器温度控制并复位积分状态
    ResultCode ConfigureImuHeater(const ImuHeaterConfig& config);

    /// 启动 IMU 加热器 PWM（初始占空比 0）
    ResultCode StartImuHeater();

    /// 依据当前 IMU 温度执行一次加热器 PI 温度闭环
    ResultCode UpdateImuHeater(float temperature_deg_c, float dt_s);

    /// 按配置中的逻辑舵面标识绑定一路舵机
    ResultCode BindServo(Class_PwmChannel* servo_pwm, const ServoConfig* config);

    /// 将机体控制指令混控并输出到四路舵机
    ResultCode ApplyBodyControlCommand(const BodyControlCommand& command);

    /// 使用机体系 IMU 样本执行一次过载/角速度闭环，并生成机体控制指令。
    ///
    /// 应用层边界约定：n_y_c/n_z_c 分别是机体 +Y（上向俯仰）/+Z（右向偏航）轴方向的过载目标，
    /// 单位为 g；默认有限输入的闭环钳位范围为 [-327.67, +327.67] g。
    /// @param n_y_c 机体 +Y 轴（上向俯仰）过载目标（g），超出配置范围时由闭环内部钳位。
    /// @param n_z_c 机体 +Z 轴（右向偏航）过载目标（g），超出配置范围时由闭环内部钳位。
    /// @param imu 已统一到机体坐标系的 IMU 样本；比力单位为 m/s^2，角速度单位为
    ///            rad/s，采样时间戳单位为 ms，且样本必须标记为有效。
    /// @param dt_s 本次控制周期，单位为 s；默认允许范围为 [0.001, 0.020] s。
    /// @param command 输出的机体归一化指令：roll/pitch/yaw 分别对应绕机体 +X/+Z/+Y
    ///                轴的滚转/俯仰/偏航需求，单位无量纲、范围为 [-1, +1]；时间戳沿用 imu。
    ///                无效输出仍保持三轴中位值 0.0，并将 valid 置为 false。
    /// @param enable_roll 是否使能 Roll 轴闭环（已发射时为 true）。
    /// @param enable_pitch_yaw 是否使能 Pitch/Yaw 轴闭环（已识别绿灯时为 true）。
    /// @return 新样本且通过输入检查时返回 ResultCode::Ok；否则返回相应错误码，
    ///         command 保持无效的安全中位输出。
    ResultCode UpdateClosedLoop(
        float n_y_c,
        float n_z_c,
        const ImuData& imu,
        float dt_s,
        BodyControlCommand& command,
        bool enable_roll = true,
        bool enable_pitch_yaw = true);

    /// 清除闭环积分、微分和样本门控状态。
    ///
    /// 这是应用层重新开始闭环或由 Task 层判定数据年龄超时/解算失败后调用的复位边界；下一次
    /// UpdateClosedLoop 不会继承复位前的积分、微分和上一样本状态。
    void ResetClosedLoop();

    /// 返回最近一次闭环解算的中间量快照（含过载环角速度输出 omega_y_c/omega_z_c）。
    ///
    /// 未解算或复位后为安全中性快照（角速度输出为 0、valid 为 false）；供遥测层
    /// 取用过载环 PID 输出。该快照为上一拍结果，不随本次采样同步刷新。
    const ClosedLoopControlOutput& GetLastClosedLoopOutput() const;

    /// 查询 BMI088 是否已绑定
    bool IsBmi088Bound() const;

    bool IsBmi088Initialized() const;

    /// 查询 IMU 加热器是否已绑定
    bool IsImuHeaterBound() const;

    /// 查询指定下标的舵机是否已绑定
    bool IsServoBound(uint32_t index) const;

private:
    /// 记录最近一次错误码
    void StoreLastError(ResultCode result);

    /// 将全部已绑定舵机恢复到中立位置
    void SetAllServosNeutral();

    Class_Bmi088 bmi088_;                         ///< BMI088 惯性传感器
    Class_Servo servos_[kSurfaceCount];           ///< 四路舵机（对应四个舵面）
    Class_ImuHeater imu_heater_;                  ///< IMU 加热器
    Class_XTailMixer xtail_mixer_;                ///< X 尾翼混控器
    Class_AttitudeEstimator attitude_estimator_;  ///< 姿态估计器
    /// 应用层持有的闭环状态；运行时仅由 UpdateClosedLoop 和 ResetClosedLoop 访问，
    /// 控制器的无效结果在跨出此边界时保持为安全中位指令。
    Class_ClosedLoopController closed_loop_controller_; ///< 过载 PI 与角速度 PID 控制器
    ClosedLoopControlOutput last_closed_loop_output_;   ///< 最近一次闭环解算的中间量快照
    SystemStatus system_status_;                  ///< 系统状态
};

#endif /* USER_APPLICATION_APP_DART_SYSTEM_H */
