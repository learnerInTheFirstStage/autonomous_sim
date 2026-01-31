add_library(planner
    src/planner.cpp
)

target_include_directories(planner
    PUBLIC
        include
)

target_link_libraries(planner
    PUBLIC
        common
        map
)