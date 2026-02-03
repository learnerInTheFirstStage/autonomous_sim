#pragma once

#include <vector>
#include "common/point.h"

namespace planner {

struct Path {
    std::vector<common::Point2D> points;
};

}  // namespace planner