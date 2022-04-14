/*
 * Copyright (c) 2020, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/impl/io.hpp>

// Boost library
#include <boost/program_options.hpp>

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

    std::string accumulated_file_name;
    bool b_accumulated_file;

    double roll;
    double pitch;
    double yaw;
    double elevation;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&src_file_name)->required(), "Input cloud file (.ply)")
    ("target,t", po::value<std::string>(&tgt_file_name)->required(), "Input target file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("accumulated,a", po::value<std::string>(&accumulated_file_name), "Saves the accumulated cloud in a .ply file")
    ("roll,r", po::value<double>(&roll)->default_value(0.0), "Rotation in X - degrees (Roll)")
    ("pitch,p", po::value<double>(&pitch)->default_value(0.0), "Rotation in Y - degrees (Pitch)")
    ("yaw,y", po::value<double>(&yaw)->default_value(0.0), "Rotation in Z - degrees (Yaw)")
    ("elevation,e", po::value<double>(&elevation)->default_value(0.0), "Translation in Y");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Rotation tool to translate one point cloud with regard to one reference point cloud." << std::endl
                << std::endl;
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

    b_accumulated_file = vm.count("accumulated");
    if (b_accumulated_file) {
      accumulated_file_name = vm["accumulated"].as<std::string>();
    }

    roll = vm["roll"].as<double>();
    pitch = vm["pitch"].as<double>();
    yaw = vm["yaw"].as<double>();
    elevation = vm["elevation"].as<double>();

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

    Eigen::Affine3f transform;

    // Translate point cloud centroid to origin
    Eigen::Vector4f src_centroid;
    pcl::compute3DCentroid(*cloud_src, src_centroid);
    transform = Eigen::Affine3f::Identity();
    transform.translation() << -src_centroid[0], -src_centroid[1], -src_centroid[2];
    pcl::transformPointCloudWithNormals(*cloud_src, *cloud_src, transform);

    // Rotation
    transform = Eigen::Affine3f::Identity();
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(DEG2RAD(roll), Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(DEG2RAD(pitch), Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(DEG2RAD(yaw), Eigen::Vector3f::UnitZ());
    transform.rotate(rotation);
    pcl::transformPointCloudWithNormals(*cloud_src, *cloud_src, transform);

    // Translate src point cloud centroid to tgt centroid
    Eigen::Vector4f tgt_centroid;
    pcl::compute3DCentroid(*cloud_tgt, tgt_centroid);
    transform = Eigen::Affine3f::Identity();
    transform.translation() << tgt_centroid[0], tgt_centroid[1], tgt_centroid[2];
    pcl::transformPointCloudWithNormals(*cloud_src, *cloud_src, transform);

    // Translate src point cloud in Y axis
    transform = Eigen::Affine3f::Identity();
    transform.translation() << 0, elevation, 0;
    pcl::transformPointCloudWithNormals(*cloud_src, *cloud_src, transform);

    pcl::io::savePLYFileBinary(output_file_name, *cloud_src);

    if (b_accumulated_file) {
      PointC::Ptr accumulated(new PointC);
      pcl::copyPointCloud(*cloud_src, *accumulated);
      *accumulated += *cloud_tgt;
      pcl::io::savePLYFileBinary(accumulated_file_name, *accumulated);
    }

    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "An unknown error has occurred." << std::endl;
  }

  return -1;
}
