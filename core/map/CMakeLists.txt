add_library(map
    src/grid_map.cpp
)

target_include_directories(map
    PUBLIC
        include
)

target_link_libraries(map
    PUBLIC common
)