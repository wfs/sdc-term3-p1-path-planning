project(Path_Planning)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# BDD-Style Tests
#set(sources src/test_main.cpp)

# Udacity provided boiler-plate code
set(sources src/main.cpp src/spline.hpp src/ego_vehicle.cpp src/ego_vehicle.hpp src/traffic.cpp src/traffic.hpp)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
