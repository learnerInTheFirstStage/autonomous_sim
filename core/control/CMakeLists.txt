add_library(control
    src/pure_pursuit.cpp
)

target_include_directories(control
    PUBLIC
        include
)

target_link_libraries(control
    PUBLIC
        common
        planner
)