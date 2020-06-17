/*
 * Copyright (c) 2020, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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
    double threshold;
    bool negative;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&input_file_name)->required(), "Input file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("threshold,t", po::value<double>(&threshold)->default_value(0.05), "Standard radius to remove")
    ("negative,n", "Saves the removed points in a .ply file");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Plane model segmentation." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("output")) {
      input_file_name = vm["input"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw std::string("Correct mode of use: " + std::string(argv[0]) + " -i input.ply -o output.ply");
    }
    threshold = vm["threshold"].as<double>();
    negative = vm.count("negative");

    PointC::Ptr point_cloud(new PointC);
    PointC::Ptr cloud_filtered(new PointC);

    // Load the point cloud data from disk
    if (pcl::io::loadPLYFile<PointT>(input_file_name, *point_cloud) == -1) {
      throw std::string("Couldn't load input file");
    }

    std::cout << "Cloud before filtering: " << std::endl;
    std::cout << *point_cloud << std::endl;

    // Create the segmentation object
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(point_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      throw std::string("Could not estimate a planar model for the given dataset");
    }

    // Extract indices
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(point_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    // Save ply with removed points
    std::cout << "Cloud after filtering: " << std::endl;
    std::cout << *cloud_filtered << std::endl;
    pcl::io::savePLYFileBinary(output_file_name, *cloud_filtered);

    // Save points of the plane
    if (negative) {
      extract.setNegative(false);
      extract.filter(*cloud_filtered);
      size_t pos = output_file_name.rfind(".ply");
      if (pos != std::string::npos) output_file_name.erase(pos, 4);
      pcl::io::savePLYFileBinary(output_file_name + "_plane.ply", *cloud_filtered);
    }

    return 0;
  } catch (boost::program_options::error& msg) {
    std::cout << "ERROR: " << msg.what() << std::endl;
  } catch (std::string msg) {
    std::cout << msg << std::endl;
  }

  return -1;
}
