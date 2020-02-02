cmake_minimum_required(VERSION 3.1)
project(pair_align)

add_compile_options(-std=c++11)
add_definitions(-Wall)

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} pair_align.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
