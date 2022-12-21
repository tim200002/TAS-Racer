cmake_minimum_required(VERSION 3.1)
project(cpp_path_planner)

find_package(xtl REQUIRED)
find_package(xtensor REQUIRED)

add_executable(cpp_path_planner src/main.cpp)

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS /MANIFEST:NO)
endif()

target_link_libraries(cpp_path_planner xtensor xtensor::optimize xtensor::use_xsimd)