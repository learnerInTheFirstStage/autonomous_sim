#include "map/grid_map.h"
#include <stdexcept>

namespace map {

GridMap::GridMap(int width, int height, double resolution)
    : width_(width),
      height_(height),
      resolution_(resolution),
      data_(width * height, 0) {}

int GridMap::Index(int x, int y) const {
    return y * width_ + x;
}

bool GridMap::IsOccupied(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return true;  // out of bounds treated as obstacl
    }

    return data_[Index(x, y)] != 0;
}

bool GridMap::IsOccupiedWorld(double wx, double wy) const {
    int x = static_cast<int>(wx / resolution_);
    int y = static_cast<int>(wy / resolution_);
    return IsOccupied(x, y);
}

void GridMap::SetOccupied(int x, int y) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        throw std::out_of_range("Grid index out of range");
    }

    data_[Index(x, y)] = 1;
}

}  // namespace map