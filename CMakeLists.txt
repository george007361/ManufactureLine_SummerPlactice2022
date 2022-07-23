cmake_minimum_required(VERSION 3.5)

project(ManualControlProgram LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(ManualControlProgram ManualControlProgram.cpp)
