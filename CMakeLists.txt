cmake_minimum_required(VERSION 3.16)
project(motor-control-pid-encoder CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(motor_pid_sim src/main.cpp src/pid.cpp)
target_compile_options(motor_pid_sim PRIVATE -Wall -Wextra)
