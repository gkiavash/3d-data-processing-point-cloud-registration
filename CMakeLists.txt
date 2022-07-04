cmake_minimum_required(VERSION 3.20)
project(mvs_reg)

set(CMAKE_CXX_STANDARD 23)
include_directories(src)
find_package(Open3D REQUIRED)
aux_source_directory( src SRC )
add_library(Registration STATIC ${SRC})
target_link_libraries(Registration PRIVATE Open3D::Open3D)

add_executable(registration registration_trial.cpp)
target_sources(registration PRIVATE registration_trial.cpp)
target_link_libraries(registration PRIVATE Open3D::Open3D  Registration )
