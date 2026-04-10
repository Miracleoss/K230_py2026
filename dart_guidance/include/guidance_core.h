#ifndef GUIDANCE_CORE_H
#define GUIDANCE_CORE_H

#include "guidance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 完整的制导计算流程
 * 
 * 整合所有步骤：距离估计、视线角计算、角速度计算、比例导引
 * 
 * @param input 制导输入数据
 * @param config 系统配置
 * @param state 系统状态（会被更新）
 * @param prev_output 上一次的输出（用于角速度计算）
 * @return GuidanceOutput 制导输出
 */
GuidanceOutput calculate_guidance_output(GuidanceInput input, GuidanceConfig config,
                                        GuidanceState* state, GuidanceOutput* prev_output);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_CORE_H */
