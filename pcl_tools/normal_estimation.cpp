/*
 * Copyright (c) 2020, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// Boost
#include <boost/program_options.hpp>

// Typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef pcl::PointCloud<pcl::Normal> PointN;

int main(int argc, char* argv[]) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  try {
    // Declaration of variables
    std::string input_file_name;
    std::string output_file_name;
    uint neighbors;
    bool reverse_normals;
    bool centroid;
    bool origin;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&input_file_name)->required(), "Input file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("neighbors,n", po::value<uint>(&neighbors)->default_value(50), "N. of neighbors to analyze for each point")
    ("reverse_normals,r", "Reverse normals' direction")
    ("centroid,c", "Use the centroid as defined view point")
    ("origin,z", "Use the origin as defined view point");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Estimate a set of normals for all the points in the input dataset." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("output")) {
      input_file_name = vm["input"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw std::string("Correct mode of use: " + std::string(argv[0]) + " -i input.ply -o output.ply [opts]");
    }

    neighbors = vm["neighbors"].as<uint>();
    reverse_normals = vm.count("reverse_normals");
    centroid = vm.count("centroid");
    origin = vm.count("origin");

    if (origin && centroid) {
      throw std::string("It is not possible to use the centroid and origin as viewpoint at the same time");
    }

    // Load point cloud
    PointC::Ptr cloud(new PointC);
    if (pcl::io::loadPLYFile<PointT>(input_file_name, *cloud) == -1) {
      throw std::string("Couldn't load input file");
    }

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset.
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);

    // Output datasets
    PointN::Ptr normals(new PointN);

    // Use a certain number of neighbors
    ne.setKSearch(neighbors);

    if (centroid) {
      // Compute 3D centroid
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud, centroid);
      ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    } else if (origin) {
      ne.useSensorOriginAsViewPoint();
    }

    // Compute the normals
    ne.compute(*normals);

    // The setViewPoint function will flip the normals of the whole point cloud.
    // Once use the centroid of the set of points as view point, we need to invert the normals to outside the object.
    if ((centroid && !reverse_normals) || (!centroid && reverse_normals)) {
      for (size_t i = 0; i < normals->size(); i++) {
        normals->points[i].normal_x *= -1;
        normals->points[i].normal_y *= -1;
        normals->points[i].normal_z *= -1;
      }
    }

    // Concatenate cloud and normals and save
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_normals);
    pcl::io::savePLYFileBinary(output_file_name, *cloud_normals);

    return 0;
  } catch (boost::program_options::error& msg) {
    std::cout << "ERROR: " << msg.what() << std::endl;
  } catch (std::string msg) {
    std::cout << msg << std::endl;
  }

  return -1;
}
