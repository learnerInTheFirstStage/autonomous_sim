#pragma once
#include "common/point.h"

namespace common {

struct Pose
{
    Point2D position;
    double yaw{0.0};  // heading {rad}
};


}  // namespace common