cmake_minimum_required(VERSION 3.1)
project(bundle_merge)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(bundle_merge bundle_merge.cpp)
target_link_libraries(bundle_merge ${Boost_LIBRARIES})

install(TARGETS bundle_merge
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/LowCost3DReconstruction
)
