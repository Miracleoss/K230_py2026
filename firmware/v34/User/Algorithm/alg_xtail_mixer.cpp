#include "Algorithm/alg_xtail_mixer.h"

Class_XTailMixer::Class_XTailMixer()
{
}

ResultCode Class_XTailMixer::Mix(
    const BodyControlCommand& input,
    SurfaceCommand& output) const
{
    output.delta1 = input.roll + input.yaw + input.pitch;
    output.delta2 = input.roll + input.yaw - input.pitch;
    output.delta3 = input.roll - input.yaw - input.pitch;
    output.delta4 = input.roll - input.yaw + input.pitch;

    float max_abs = output.delta1 >= 0.0f ? output.delta1 : -output.delta1;
    const float delta2_abs = output.delta2 >= 0.0f ? output.delta2 : -output.delta2;
    const float delta3_abs = output.delta3 >= 0.0f ? output.delta3 : -output.delta3;
    const float delta4_abs = output.delta4 >= 0.0f ? output.delta4 : -output.delta4;

    if (delta2_abs > max_abs) {
        max_abs = delta2_abs;
    }
    if (delta3_abs > max_abs) {
        max_abs = delta3_abs;
    }
    if (delta4_abs > max_abs) {
        max_abs = delta4_abs;
    }

    if (max_abs > 1.0f) {
        output.delta1 /= max_abs;
        output.delta2 /= max_abs;
        output.delta3 /= max_abs;
        output.delta4 /= max_abs;
    }

    output.timestamp_ms = input.timestamp_ms;
    output.valid = input.valid;
    return ResultCode::Ok;
}
