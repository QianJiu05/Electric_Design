cmake_minimum_required(VERSION 3.31)
set(CMAKE_C_STANDARD 11)
project(mathtest C)
add_executable(mathtest using_math.c
        using_math.h)