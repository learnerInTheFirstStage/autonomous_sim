#include "planner/astar/astar_planner.h"
#include "planner/astar/astar_node.h"
#include "common/point.h"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <vector>
#include <unordered_set>

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
    
    std::unordered_set<long long> closed;
    int sx = static_cast<int>(start.x);
    int sy = static_cast<int>(start.y);
    int gx = static_cast<int>(goal.x);
    int gy = static_cast<int>(goal.y);

    std::priority_queue<AStarNode*, std::vector<AStarNode*>, NodeCompare> open;
    std::unordered_map<long long, AStarNode*> all_nodes;

    auto key = [](int x, int y) {
        return (static_cast<long long>(x) << 32) | (unsigned int)y;
    };

    AStarNode* start_node = new AStarNode(sx, sy);
    start_node->h = Heuristic(sx, sy, gx, gy);
    start_node->f = start_node->h;

    open.push(start_node);
    all_nodes[key(sx, sy)] = start_node;

    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    AStarNode* goal_node = nullptr;

    while (!open.empty()) {
        AStarNode* current = open.top();
        open.pop();

        long long current_key = key(current->x, current->y);

        if (closed.count(current_key)) {
            continue;
        }

        closed.insert(current_key);

        if (current->x == gx && current->y == gy) {
            goal_node = current;
            break;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (map.IsOccupied(nx, ny)) {
                continue;
            }

            long long k = key(nx, ny);

            if (closed.count(k)) {
                continue;
            }

            double step = (i < 4) ? 1.0 : 1.414;
            double new_g = current->g + step;

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

    for (auto& kv : all_nodes) {
        delete kv.second;
    }

    return true;

}

}  // namespace planner