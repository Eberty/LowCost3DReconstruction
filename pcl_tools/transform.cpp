/*
 * Copyright (c) 2020, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// Boost
#include <boost/program_options.hpp>

// Typedefs
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;

int main(int argc, char* argv[]) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  try {
    // Declaration of variables
    std::string input_file_name;
    std::string output_file_name;
    std::string transform_file;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&input_file_name)->required(), "Input file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("transform,t", po::value<std::string>(&transform_file)->required(), "File containing a 4x4 transformation matrix");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Transforms Point cloud." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("output") && vm.count("transform")) {
      input_file_name = vm["input"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
      transform_file = vm["transform"].as<std::string>();
    } else {
      throw std::logic_error("Correct mode of use: " + std::string(argv[0]) +
                             " -i input.ply -o output.ply -t transform_file.txt");
    }

    PointC::Ptr source_cloud(new PointC);
    PointC::Ptr transformed_cloud(new PointC);

    // Load the point cloud data from disk
    if (pcl::io::loadPLYFile<PointT>(input_file_name, *source_cloud) == -1) {
      throw std::runtime_error("Couldn't load input point cloud: " + input_file_name);
    }

    // Load the transform matrix data from txt file
    double matrix[4][4];
    std::ifstream file(transform_file);
    if (file.is_open()) {
      for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
          if (!(file >> matrix[i][j])) {
            throw std::runtime_error("Error on read transform file: " + transform_file);
          }
        }
      }
    } else {
      throw std::runtime_error("Unable to open file: " + transform_file);
    }

    // Transform
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j++) {
        transform(i, j) = matrix[i][j];
      }
    }
    pcl::transformPointCloudWithNormals(*source_cloud, *transformed_cloud, transform);

    pcl::io::savePLYFileBinary(output_file_name, *transformed_cloud);

    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "An unknown error has occurred." << std::endl;
  }

  return -1;
}
