#include <iostream>
#include "map/grid_map.h"

int main() {
    map::GridMap grid(10, 10, 1.0);
    grid.SetOccupied(3, 3);

    std::cout << "IS (3, 3) occupied? "
              << grid.IsOccupied(3, 3) << std::endl;
    
    std::cout << "IS (5, 5) occupied? "
              << grid.IsOccupied(5, 5) << std::endl;
    
    return 0;
}