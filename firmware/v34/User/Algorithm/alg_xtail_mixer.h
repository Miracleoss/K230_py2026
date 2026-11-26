#ifndef USER_ALGORITHM_ALG_XTAIL_MIXER_H
#define USER_ALGORITHM_ALG_XTAIL_MIXER_H

#include "Common/common_types.h"
#include "Common/control_command.h"
#include "Common/surface_command.h"

class Class_XTailMixer {
public:
    Class_XTailMixer();

    ResultCode Mix(const BodyControlCommand& input, SurfaceCommand& output) const;
};

#endif /* USER_ALGORITHM_ALG_XTAIL_MIXER_H */
