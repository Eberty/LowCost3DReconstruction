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
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

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
    double cluster_percentage;
    double tolerance;
    bool outliers_file;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&input_file_name)->required(), "Input file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("outliers_file,f", "Saves the removed points in a .ply file")
    ("cluster_percentage,p", po::value<double>(&cluster_percentage)->default_value(0.25), "Percentage (0 to 1) of points that a cluster needs to contain in order to be considered valid")
    ("tolerance,t", po::value<double>(&tolerance)->default_value(0.02), "Spatial cluster tolerance as a measure in the L2 Euclidean space");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Euclidean cluster extraction." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("output")) {
      input_file_name = vm["input"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw std::logic_error("Correct mode of use: " + std::string(argv[0]) + " -i input.ply -o output.ply");
    }
    cluster_percentage = vm["cluster_percentage"].as<double>();
    tolerance = vm["tolerance"].as<double>();
    outliers_file = vm.count("outliers_file");

    if (cluster_percentage < 0 || cluster_percentage > 1) {
      throw std::logic_error("cluster_percentage must be a value between 0 and 1");
    }

    PointC::Ptr cloud(new PointC);
    PointC::Ptr cloud_filtered(new PointC);

    // Load the point cloud data from disk
    if (pcl::io::loadPLYFile<PointT>(input_file_name, *cloud) == -1) {
      throw std::runtime_error("Couldn't load input point cloud: " + input_file_name);
    }

    std::cout << "Cloud before filtering: " << std::endl;
    std::cout << *cloud << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    // Create the euclidean cluster extraction object
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(cloud->size() * cluster_percentage);
    ec.setMaxClusterSize(cloud->size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
      throw std::runtime_error("Could not extact clusters for the given dataset");
    } else {
      std::cout << cluster_indices.size() << " cluster(s) extracted." << std::endl << std::endl;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (std::vector<pcl::PointIndices>::iterator it_seg = cluster_indices.begin(); it_seg != cluster_indices.end(); it_seg++) {
      std::copy(it_seg->indices.begin(), it_seg->indices.end(), std::back_inserter(inliers->indices));
    }

    // Extract indices
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*cloud_filtered);

    std::cout << "Cloud after filtering: " << std::endl;
    std::cout << *cloud_filtered << std::endl;
    pcl::io::savePLYFileBinary(output_file_name, *cloud_filtered);

    // Saves the outliers in a ply file
    if (outliers_file) {
      extract.setNegative(true);
      extract.filter(*cloud_filtered);
      size_t pos = output_file_name.rfind(".ply");
      if (pos != std::string::npos) output_file_name.erase(pos, 4);
      pcl::io::savePLYFileBinary(output_file_name + "_outliers.ply", *cloud_filtered);
    }

    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "An unknown error has occurred." << std::endl;
  }

  return -1;
}
