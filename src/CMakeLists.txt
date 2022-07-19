cmake_minimum_required(VERSION 3.20)
project(test)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(test
        gate_node.cpp
        LampController.hpp
        ManipulatorController.hpp
        UDPSocket.hpp)
