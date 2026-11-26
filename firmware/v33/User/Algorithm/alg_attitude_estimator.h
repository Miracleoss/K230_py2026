#ifndef USER_ALGORITHM_ALG_ATTITUDE_ESTIMATOR_H
#define USER_ALGORITHM_ALG_ATTITUDE_ESTIMATOR_H

#include "Common/attitude_state.h"
#include "Common/common_types.h"
#include "Common/imu_data.h"

class Class_AttitudeEstimator {
public:
    Class_AttitudeEstimator();

    ResultCode Reset();
    ResultCode Update(const ImuData& imu, AttitudeState& attitude);
};

#endif /* USER_ALGORITHM_ALG_ATTITUDE_ESTIMATOR_H */
