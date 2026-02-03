#pragma once

#include "map/grid_map.h"
#include "planner/path.h"

namespace planner {

class Planner {

public:
    virtual ~Planner() = default;

    virtual bool Plan(
        const map::GridMap& map,
        const common::Point2D& start,
        const common::Point2D& goal,
        Path& path) = 0;

};

}  // namespace planner