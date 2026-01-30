#include <iostream>
#include "common/pose.h"

int main() {
    common::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.yaw = 0.5;

    std::cout << "Pose: ("
              << pose.position.x << ", "
              << pose.position.y << "), yaw="
              << pose.yaw << std::endl;
    
    return 0;
}