/*
 * Copyright (c) 2019, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
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
    uint number_of_neighbors;
    double deviation_multiplier;
    bool outliers_file;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&input_file_name)->required(), "Input file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("outliers_file,f", "Saves the outliers in a ply file")
    ("neighbors,n", po::value<uint>(&number_of_neighbors)->default_value(50), "N. of neighbors to analyze for each point")
    ("dev_mult,d", po::value<double>(&deviation_multiplier)->default_value(1.0), "Standard deviation multiplier");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("output")) {
      input_file_name = vm["input"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw std::string("Correct mode of use: " + std::string(argv[0]) + " -i input.ply -o output.ply [opts]");
    }

    number_of_neighbors = vm["neighbors"].as<uint>();
    deviation_multiplier = vm["dev_mult"].as<double>();
    outliers_file = vm.count("outliers_file");

    PointC::Ptr cloud(new PointC);
    PointC::Ptr cloud_filtered(new PointC);

    // Load the point cloud data from disk
    if (pcl::io::loadPLYFile<PointT>(input_file_name, *cloud) == -1) {
      throw std::string("Couldn't load input file");
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(number_of_neighbors);
    sor.setStddevMulThresh(deviation_multiplier);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    pcl::io::savePLYFileBinary(output_file_name, *cloud_filtered);

    // Saves the outliers in a ply file
    if (outliers_file) {
      sor.setNegative(true);
      sor.filter(*cloud_filtered);
      pcl::io::savePLYFileBinary("outliers.ply", *cloud_filtered);
    }

    return 0;
  } catch (boost::program_options::error& msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (std::string msg) {
    std::cerr << msg << std::endl;
  }

  return -1;
}
