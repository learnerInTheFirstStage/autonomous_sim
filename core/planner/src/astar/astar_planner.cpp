#include "planner/astar/astar_planner.h"
#include "planner/astar/astar_node.h"
#include "common/point.h"

#include <queue>
#include <unordered_map>
#include <cmath>
#include <vector>

namespace planner {

namespace {

struct NodeCompare {
    bool operator()(const AStarNode* a, const AStarNode* b) const {
        return a->f > b->f;
    }
};

double Heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

}  // namespace

bool AStarPlanner::Plan(
    const map::GridMap& map,
    const common::Point2D& start,
    const common::Point2D& goal,
    Path& path) {
        
    int sx = static_cast<int>(start.x);
    int sy = static_cast<int>(start.y);
    int gx = static_cast<int>(goal.x);
    int gy = static_cast<int>(goal.y);

    std::priority_queue<AStarNode*, std::vector<AStarNode*>, NodeCompare> open;
    std::unordered_map<int, AStarNode*> all_nodes;

    auto key = [](int x, int y) {
        return y * 100000 + x;  // simple hash 
    };

    AStarNode* start_node = new AStarNode(sx, sy);
    start_node->h = Heuristic(sx, sy, gx, gy);
    start_node->f = start_node->h;

    open.push(start_node);
    all_nodes[key(sx, sy)] = start_node;

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    AStarNode* goal_node = nullptr;

    while (!open.empty()) {
        AStarNode* current = open.top();
        open.pop();

        if (current->x == gx && current->y == gy) {
            goal_node = current;
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (map.IsOccupied(nx, ny)) {
                continue;
            }

            int k = key(nx, ny);
            double new_g = current->g + 1.0;

            if (all_nodes.count(k) == 0) {
                AStarNode* node = new AStarNode(nx, ny);
                node->g = new_g;
                node->h = Heuristic(nx, ny, gx, gy);
                node->f = node->g + node->h;
                node->parent = current;

                open.push(node);
                all_nodes[k] = node;
            }
        }
    }

    if (!goal_node) {
        return false;
    }

    // reconstruct path
    AStarNode* node = goal_node;
    while (node) {
        path.points.push_back(common::Point2D({static_cast<double>(node->x), static_cast<double>(node->y)}));

        node = node->parent;
    }

    std::reverse(path.points.begin(), path.points.end());

    return true;

}

}  // namespace planner