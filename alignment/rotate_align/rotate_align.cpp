/*
 * Copyright (c) 2018-2019, Eberty Alves, Pedro Raimundo
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/program_options.hpp>

int main(int argc, char **argv) {
  try {
    // Declaration of variables
    std::string capture_name;
    uint capture_step;
    uint num_captures;
    double distance_x;
    double distance_y;
    double distance_z;

    bool b_visualize;
    bool b_accumulated_file;
    std::string accumulated_file_name;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("visualize,v", "Visualization of transformation")
    ("capture_name,n", po::value<std::string>(&capture_name)->default_value("artefact_srmesh_"), "Prefix of saved files")
    ("capture_step,s", po::value<uint>(&capture_step)->default_value(20), "Angles (in degrees) for each capture")
    ("num_captures,c", po::value<uint>(&num_captures)->default_value(18), "Number of captures")
    ("distance_x,x", po::value<double>(&distance_x)->default_value(44.0), "Distance in X from kinect to the center of the table")
    ("distance_y,y", po::value<double>(&distance_y)->default_value(60.0), "Distance in Y from kinect to the center of the table")
    ("distance_z,z", po::value<double>(&distance_z)->default_value(632.5), "Distance in Z from kinect to the center of the table")
    ("accumulated_file,a", po::value<std::string>(&accumulated_file_name), "Saves the accumulated mesh in a .ply file");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Semi-automatic rotation tool to orient the lateral views with regard to the first." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    capture_name = vm["capture_name"].as<std::string>();
    capture_step = vm["capture_step"].as<uint>();
    num_captures = vm["num_captures"].as<uint>();
    distance_x = vm["distance_x"].as<double>();
    distance_y = vm["distance_y"].as<double>();
    distance_z = vm["distance_z"].as<double>();

    b_visualize = vm.count("visualize");

    b_accumulated_file = vm.count("accumulated_file");
    if (b_accumulated_file) {
      accumulated_file_name = vm["accumulated_file"].as<std::string>();
    }

    std::cout << "Capture name: " << capture_name << std::endl;
    std::cout << "Step: " << capture_step << std::endl;
    std::cout << "N. of captures: " << num_captures << std::endl;
    std::cout << "Save accumulated file: " << (b_accumulated_file ? "Yes" : "No") << std::endl;

    // Convenient typedefs
    typedef pcl::PointXYZRGBNormal PointT;
    typedef pcl::PointCloud<PointT> PointC;

    // Defines point clouds, save ply with translated points
    std::vector<PointC::Ptr> point_clouds;

    for (size_t i = 0; i < num_captures; i++) {
      point_clouds.push_back(PointC::Ptr(new PointC));
      std::string ply_file = std::string(capture_name) + std::to_string(i * capture_step) + ".ply";
      if (pcl::io::loadPLYFile<PointT>(ply_file, *point_clouds[i]) != -1) {
        std::cout << "MESH " << (i * capture_step) << " LOADED ALRIGHT!" << std::endl;
      } else {
        std::cout << ply_file << " is not a valid file." << std::endl;
      }
    }

    // Transform to the origin and alignment according to rotational parameters
    for (size_t i = 0; i < num_captures; i++) {
      // Cleaning
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*point_clouds[i], *point_clouds[i], indices);
      // Translating to center
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      // Distance from kinect to the center of the table
      transform.translation() << distance_x, distance_y, distance_z;
      pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], transform);
      // Rotate around center
      transform = Eigen::Affine3f::Identity();
      Eigen::Matrix3f rotation(Eigen::AngleAxisf(-DEG2RAD(i * capture_step + i * 1.17), Eigen::Vector3f::UnitY()));
      transform.rotate(rotation);
      pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], transform);
      pcl::io::savePLYFileBinary(capture_name + std::to_string(i * capture_step) + "_rotated.ply", *point_clouds[i]);
    }

    PointC::Ptr accumulated(new PointC);

    pcl::copyPointCloud(*point_clouds[0], *accumulated);
    for (size_t i = 1; i < num_captures; i++) {
      *accumulated = *accumulated + *point_clouds[i];
    }

    // Save accumulated aligned mesh
    if (b_accumulated_file) {
      pcl::io::savePLYFileBinary(accumulated_file_name, *accumulated);
    }

    // Visualization
    if (b_visualize) {
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> accumulated_color_hander(accumulated);

      pcl::visualization::PCLVisualizer viewer("Matrix transformation");
      viewer.addPointCloud(accumulated, accumulated_color_hander, "Accumulated Cloud");
      viewer.addCoordinateSystem(1.0, "cloud", 0);
      viewer.setBackgroundColor(0, 0, 0, 0);
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Accumulated Cloud");

      // Show in GUI
      while (!viewer.wasStopped()) {  // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
      }
    }

    return 0;
  } catch (boost::program_options::error &msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (...) {
    std::cerr << "Some error has occurred." << std::endl;
  }
  return -1;
}
