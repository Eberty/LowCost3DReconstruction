/*
 * Copyright (c) 2019, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// Boost library
#include <boost/program_options.hpp>

// Typedefs
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;

int main(int argc, char* argv[]) {
  try {
    // Declaration of variables
    std::vector<std::string> clouds_file_names;
    std::string output_file_name;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()("help,h", "Print help message")(
        "inputs,i", po::value<std::vector<std::string>>(&clouds_file_names)->multitoken(), "Input clouds files (.ply)")(
        "output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }

    if (!vm["inputs"].empty() && vm.count("output")) {
      clouds_file_names = vm["inputs"].as<std::vector<std::string>>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw std::string("Correct mode of use: " + std::string(argv[0]) + " -i [*.ply] -o output.ply");
    }

    PointC::Ptr cloud(new PointC);
    PointC::Ptr accumulated(new PointC);

    // Load point clouds data from disk
    for (size_t i = 0; i < clouds_file_names.size(); i++) {
      if (pcl::io::loadPLYFile<PointT>(clouds_file_names[i], *cloud) == -1) {
        throw std::string("Couldn't load input cloud file");
      }
      std::cout << "Loaded " << cloud->size() << " data points from " << clouds_file_names[i] << std::endl;
      *accumulated = *accumulated + *cloud;
    }

    pcl::io::savePLYFileBinary(output_file_name, *accumulated);
    return 0;
  } catch (boost::program_options::error& msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (std::string msg) {
    std::cerr << msg << std::endl;
  }

  return -1;
}
