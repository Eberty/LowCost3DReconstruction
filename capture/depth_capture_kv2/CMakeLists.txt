cmake_minimum_required(VERSION 3.1)
project(depth_capture_kv2)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

option(ENABLE_OPENCL "Enable OpenCL support" ON)

# Set cmake prefix path to enable cmake to find freenect2
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

if(ENABLE_OPENCL)
  find_package(OpenCL)
  if(OpenCL_FOUND)
    add_definitions("-DHAVE_OPENCL")
  endif(OpenCL_FOUND)
endif(ENABLE_OPENCL)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${freenect2_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} depth_capture_kv2.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARIES} ${Boost_LIBRARIES} ${GLFW_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/LowCost3DReconstruction
)
