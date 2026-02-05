add_library(vehicle
    src/vehicle_model.cpp
)


target_include_directories(vehicle
    PUBLIC 
        include
)

target_link_libraries(vehicle
    common
)