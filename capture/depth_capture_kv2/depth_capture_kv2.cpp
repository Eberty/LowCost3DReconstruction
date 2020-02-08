/*
 * Copyright (c) 2020, Eberty Alves
 */

// Project headers
#include "kinect2.h"

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

// Boost library
#include <boost/program_options.hpp>

// Typedefs
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> PointCloudColorHandler;

int main(int argc, char *argv[]) {
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] [output.ply]\n -processor options 0,1,2 correspond to "
                                           "CPU, OPENCL, and OPENGL respectively\n";
  Kinect2::Processor freenectprocessor = Kinect2::Processor::OPENGL;
  std::vector<int> ply_file_indices;
  if (argc > 1) {
    int fnp_int;
    ply_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
    pcl::console::parse_argument(argc, argv, "-processor", fnp_int);
    freenectprocessor = static_cast<Kinect2::Processor>(fnp_int);
  }

  PointC::Ptr cloud;
  Kinect2 kinect(freenectprocessor);

  cloud = kinect.getCloud();
  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  PointCloudColorHandler color_hander(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_hander, "cloud");

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer->setBackgroundColor(0, 0, 0);

  bool done = false;
  while ((!viewer->wasStopped()) && (!done)) {
    viewer->spinOnce();

    cloud = kinect.updateCloud(cloud);
    PointCloudColorHandler color_hander(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, color_hander, "cloud");

    if (ply_file_indices.size() > 0) {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
      pcl::io::savePLYFileBinary(std::string(argv[ply_file_indices[0]]), *cloud);
      done = true;
    }
  }

  kinect.shutDown();
  return 0;
}
