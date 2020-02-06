cmake_minimum_required(VERSION 3.1)
project(centroid_align)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} centroid_align.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/msc-research
)
