cmake_minimum_required(VERSION 3.23)
project(untitled)
set(VCPKG_TARGET_TRIPLET "x64-mingw-static" CACHE STRING "" FORCE)
set(VCPKG_FEATURE_FLAGS "version")          # 用于支持自定义依赖版本
set(CMAKE_CXX_STANDARD 14)

include_directories("D:/SDK/eigen3")

#安装
add_executable(untitled main.cpp Robot.cpp Robot.h)
