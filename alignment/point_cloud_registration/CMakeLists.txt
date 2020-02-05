cmake_minimum_required(VERSION 3.1)
project(point_cloud_registration)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${teaserpp_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} point_cloud_registration.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBS} teaserpp::teaser_registration)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/msc-research
)
