cmake_minimum_required(VERSION 3.1)
project(msc-research)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory and run CMake from there. You may need to remove CMakeCache.txt.")
endif()

# Add targets
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/alignment/fine_registration/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/alignment/pair_align/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/alignment/rotate_align/")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/super_resolution/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/depth_capture/")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tools/cloud_downsampling/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tools/crop_cloud/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tools/outlier_removal/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tools/scale/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tools/transform/")

# Install meshlab files
install(DIRECTORY meshlab/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/msc-research
)
