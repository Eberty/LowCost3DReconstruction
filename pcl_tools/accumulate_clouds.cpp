/*
 * Copyright (c) 2020, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Boost library
#include <boost/program_options.hpp>
#include <boost/progress.hpp>

// Typedefs
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;

int main(int argc, char* argv[]) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  try {
    // Declaration of variables
    std::string src_file_name;
    std::string tgt_file_name;
    std::string output_file_name;

    double radius;
    bool negative;
    bool with_redundant;

    uint number_of_neighbors;
    double deviation_multiplier;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&src_file_name)->required(), "Input cloud file (.ply)")
    ("target,t", po::value<std::string>(&tgt_file_name)->required(), "Input target file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("radius,r", po::value<double>(&radius)->default_value(0.05), "Standard radius to select point neighbours")
    ("clean_neighbors,c", po::value<uint>(&number_of_neighbors)->default_value(50), "N. of neighbors to analyze for each point to clean")
    ("dev_mult,d", po::value<double>(&deviation_multiplier)->default_value(1.0), "Standard deviation multiplier to clean")
    ("negative,n", "Saves the not redundant points in a .ply file")
    ("all,a", "Saves accumulated cloud with all points in a .ply file");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Accumulate points clouds removing redundant points" << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("target") && vm.count("output")) {
      src_file_name = vm["input"].as<std::string>();
      tgt_file_name = vm["target"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw std::logic_error("Correct mode of use: " + std::string(argv[0]) +
                             " -i input.ply -t target.ply -o output.ply [opts]");
    }

    number_of_neighbors = vm["clean_neighbors"].as<uint>();
    deviation_multiplier = vm["dev_mult"].as<double>();
    radius = vm["radius"].as<double>();
    negative = vm.count("negative");
    with_redundant = vm.count("all");

    PointC::Ptr cloud_src(new PointC);
    PointC::Ptr cloud_tgt(new PointC);

    // Load point clouds data from disk
    if (pcl::io::loadPLYFile<PointT>(src_file_name, *cloud_src) == -1) {
      throw std::runtime_error("Couldn't load input point cloud: " + src_file_name);
    }
    std::cout << "Loaded " << cloud_src->size() << " data points from " << src_file_name << std::endl;

    if (pcl::io::loadPLYFile<PointT>(tgt_file_name, *cloud_tgt) == -1) {
      throw std::runtime_error("Couldn't load target point cloud: " + tgt_file_name);
    }
    std::cout << "Loaded " << cloud_tgt->size() << " data points from " << tgt_file_name << std::endl;

    std::cout << "Cloud before accumulate: " << std::endl;
    std::cout << *cloud_tgt << std::endl;

    if (!with_redundant) {
      pcl::CropBox<PointT> crop_box;
      crop_box.setNegative(true);
      crop_box.setInputCloud(cloud_src);
      boost::progress_display show_progress(cloud_tgt->points.size());
      for (size_t i = 0; i < cloud_tgt->points.size(); i++) {
        PointT* point = &cloud_tgt->points[i];
        crop_box.setMin(Eigen::Vector4f(point->x - radius, point->y - radius, point->z - radius, 1.0));
        crop_box.setMax(Eigen::Vector4f(point->x + radius, point->y + radius, point->z + radius, 1.0));
        crop_box.filter(*cloud_src);
        ++show_progress;
      }

      // Create the filtering object
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud_src);
      sor.setMeanK(number_of_neighbors);
      sor.setStddevMulThresh(deviation_multiplier);
      sor.filter(*cloud_src);
    }

    PointC::Ptr accumulated(new PointC);
    pcl::copyPointCloud(*cloud_tgt, *accumulated);
    *accumulated += *cloud_src;

    std::cout << "Cloud after accumulate: " << std::endl;
    std::cout << *accumulated << std::endl;
    pcl::io::savePLYFileBinary(output_file_name, *accumulated);

    // Save points of second cloud
    if (negative) {
      size_t pos = output_file_name.rfind(".ply");
      if (pos != std::string::npos) output_file_name.erase(pos, 4);
      pcl::io::savePLYFileBinary(output_file_name + "_negative.ply", *cloud_src);
    }

    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "An unknown error has occurred." << std::endl;
  }

  return -1;
}
