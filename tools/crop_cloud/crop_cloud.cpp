/*
 * Copyright (c) 2019, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Boost
#include <boost/program_options.hpp>

// Typedefs
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;

int main(int argc, char* argv[]) {
  try {
    // Declaration of variables
    std::string input_file_name;
    std::string output_file_name;
    double radius;
    bool negative;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&input_file_name)->required(), "Input file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("radius,r", po::value<double>(&radius)->required(), "Standard radius to remove")
    ("negative,n", "Saves the removed points in a .ply file");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Filter all data inside of a box." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("output") && vm.count("radius")) {
      input_file_name = vm["input"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
      radius = vm["radius"].as<double>();
    } else {
      throw std::string("Correct mode of use: " + std::string(argv[0]) + " -i input.ply -o output.ply -r [radius]");
    }
    negative = vm.count("negative");

    PointC::Ptr point_cloud(new PointC);
    PointC::Ptr cloud_filtered(new PointC);

    // Load the point cloud data from disk
    if (pcl::io::loadPLYFile<PointT>(input_file_name, *point_cloud) == -1) {
      throw std::string("Couldn't load input file");
    }

    std::cout << "Cloud before filtering: " << std::endl;
    std::cout << *point_cloud << std::endl;

    // Compute point cloud centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*point_cloud, centroid);

    // Create the filtering object
    pcl::CropBox<PointT> crop_box;
    crop_box.setMin(Eigen::Vector4f(centroid[0] - radius, centroid[1] - radius, centroid[2] - radius, 1.0));
    crop_box.setMax(Eigen::Vector4f(centroid[0] + radius, centroid[1] + radius, centroid[2] + radius, 1.0));
    crop_box.setInputCloud(point_cloud);
    crop_box.filter(*cloud_filtered);

    // Save ply with points croped
    std::cout << "Cloud after filtering: " << std::endl;
    std::cout << *cloud_filtered << std::endl;
    pcl::io::savePLYFileBinary(output_file_name, *cloud_filtered);

    // Save points outside the crop box
    if (negative) {
      crop_box.setNegative(true);
      crop_box.filter(*cloud_filtered);
      size_t pos = output_file_name.rfind(".ply");
      if (pos != std::string::npos) output_file_name.erase(pos, 4);
      pcl::io::savePLYFileBinary(output_file_name + "_negative.ply", *cloud_filtered);
    }

    return 0;
  } catch (boost::program_options::error& msg) {
    std::cout << "ERROR: " << msg.what() << std::endl;
  } catch (std::string msg) {
    std::cout << msg << std::endl;
  }

  return -1;
}
