cmake_minimum_required(VERSION 3.1)
project(LowCost3DReconstruction)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory and run CMake from there. You may need to remove CMakeCache.txt.")
endif()

# Add targets
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/depth_capture/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/depth_capture_kv2/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/super_resolution/")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/capture/super_resolution_kv2/")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/pcl_tools/")

# Install meshlab files
install(DIRECTORY meshlab/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/LowCost3DReconstruction
)
