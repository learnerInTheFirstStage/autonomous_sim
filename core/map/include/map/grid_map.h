#pragma once

#include <vector>
#include "common/point.h"

namespace map {

class GridMap {
public:
    GridMap(int width, int height, double resolution);

    bool IsOccupied(int x, int y) const;
    bool IsOccupiedWorld(double wx, double wy) const;

    void SetOccupied(int x, int y);


private:
    int width_;
    int height_;
    double resolution_;

    std::vector<uint8_t> data_;

    int Index(int x, int y) const;

};

}  // namespace map