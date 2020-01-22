cmake_minimum_required(VERSION 3.1)
project(msc-research)

add_compile_options(-std=c++11)
add_definitions(-Wall)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory and run CMake from there. You may need to remove CMakeCache.txt.")
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/alignment/fine_registration/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/alignment/rotate_align/")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/super_resolution/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/depth_capture/")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/clean/cloud_downsampling/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/clean/outlier_removal/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/clean/scale/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/clean/transform/")
