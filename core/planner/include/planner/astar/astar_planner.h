#pragma once

#include "planner/planner.h"

namespace planner {

class AStarPlanner : public Planner {

public:
    bool Plan(
        const map::GridMap& map,
        const common::Point2D& start,
        const common::Point2D& goal,
        Path& path) override;

};

}  // namespace planner