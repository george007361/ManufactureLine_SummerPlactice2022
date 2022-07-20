cmake_minimum_required(VERSION 3.16)
project(test)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(LowLevelController)

add_executable(test
        gate_node.cpp
        )
target_link_libraries(test lowlevelcontrollerlib)
