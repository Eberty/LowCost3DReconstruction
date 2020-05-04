cmake_minimum_required(VERSION 3.1)
project(super_resolution_kv2)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

find_package(OpenCV REQUIRED core imgproc imgcodecs video)

find_package(Boost COMPONENTS program_options REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(${PROJECT_NAME} super_resolution_kv2.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenMP_CXX_LIBRARIES} -fopenmp)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/LowCost3DReconstruction
)
