cmake_minimum_required(VERSION 3.1)
project(rotate_align)

add_compile_options(-std=c++11)
add_definitions(-Wall)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTOUIC ON) # UI files

find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(Qt5 REQUIRED Widgets)
find_package(Boost COMPONENTS program_options REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} main.cpp rotate_align.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES} Qt5::Widgets)
