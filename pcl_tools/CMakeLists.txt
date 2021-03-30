cmake_minimum_required(VERSION 3.1)
project(pcl_tools)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall)

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS program_options timer REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(accumulate_clouds accumulate_clouds.cpp)
target_link_libraries(accumulate_clouds ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(centroid_align centroid_align.cpp)
target_link_libraries(centroid_align ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cloud_downsampling cloud_downsampling.cpp)
target_link_libraries(cloud_downsampling ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(crop_cloud crop_cloud.cpp)
target_link_libraries(crop_cloud ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(fine_registration fine_registration.cpp)
target_link_libraries(fine_registration ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(normal_estimation normal_estimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(outlier_removal outlier_removal.cpp)
target_link_libraries(outlier_removal ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planar_segmentation planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scale scale.cpp)
target_link_libraries(scale ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(transform transform.cpp)
target_link_libraries(transform ${PCL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  accumulate_clouds centroid_align cloud_downsampling crop_cloud fine_registration normal_estimation outlier_removal planar_segmentation scale transform
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/LowCost3DReconstruction
)
