cmake_minimum_required(VERSION 3.20)

include_directories(include src)

add_library(lowlevelcontrollerlib INTERFACE include/LampController.hpp include/LowLevelController.h include/ManipulatorController.hpp include/UDPSocket.hpp)