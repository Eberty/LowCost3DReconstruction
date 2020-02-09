cmake_minimum_required(VERSION 3.1)
project(depth_capture)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

find_package(OpenCV REQUIRED)

find_package(Boost COMPONENTS program_options REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(FREENECT libfreenect)
if (NOT FREENECT_FOUND)
  find_package(libfreenect REQUIRED)
endif()

add_executable(${PROJECT_NAME} depth_capture.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${FREENECT_LIBRARIES} -lfreenect -lfreenect_sync)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/msc-research
)
