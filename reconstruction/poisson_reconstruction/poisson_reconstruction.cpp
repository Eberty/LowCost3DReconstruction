/*
 * Copyright (c) 2019, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

// Boost
#include <boost/program_options.hpp>

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;

int main(int argc, char* argv[]) {
  try {
    // Declaration of variables
    std::string input_file_name;
    std::string output_file_name;
    int depth;
    int min_depth;
    float point_weight;
    float scale;
    int solver_divide;
    int iso_divide;
    float samples_per_node;
    bool confidence;
    bool output_polygons;
    bool manifold;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&input_file_name)->required(), "Input file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "Output file (.ply)")
    ("depth", po::value<int>(&depth)->default_value(8), "Set the maximum depth of the tree that will be used for surface reconstruction.")
    ("min_depth", po::value<int>(&min_depth)->default_value(4), "Set the minimum depth of the tree that will be used for surface reconstruction.")
    ("point_weight", po::value<float>(&point_weight)->default_value(4.0f), "Specifies the importance that interpolation of the point samples is given in the formulation of the screened Poisson equation. The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0.")
    ("scale", po::value<float>(&scale)->default_value(1.1f), "Set the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube.")
    ("solver_divide", po::value<int>(&solver_divide)->default_value(8), "Set the the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation.")
    ("iso_divide", po::value<int>(&iso_divide)->default_value(8), "Set the depth at which a block iso-surface extractor should be used to extract the iso-surface.")
    ("samples_per_node", po::value<float>(&samples_per_node)->default_value(1.0f), "Set the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density.")
    ("confidence", po::value<bool>(&confidence)->default_value(false), "Enabling this flag tells the reconstructor to use the size of the normals as confidence information. When the flag is not enabled, all normals are normalized to have unit-length prior to reconstruction.")
    ("output_polygons", po::value<bool>(&output_polygons)->default_value(false), "Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating the results of Marching Cubes).")
    ("manifold", po::value<bool>(&manifold)->default_value(true), "Enabling this flag tells the reconstructor to add the polygon barycenter when triangulating polygons with more than three vertices.");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Compute the surface reconstruction of a point cloud using the Poisson surface reconstruction (pcl::surface::Poisson)." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("output")) {
      input_file_name = vm["input"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw "Correct mode of use: normal_estimation -i input.ply -o output.ply [opts]";
    }

    depth = vm["depth"].as<int>();
    min_depth = vm["min_depth"].as<int>();
    point_weight = vm["point_weight"].as<float>();
    scale = vm["scale"].as<float>();
    solver_divide = vm["solver_divide"].as<int>();
    iso_divide = vm["iso_divide"].as<int>();
    samples_per_node = vm["samples_per_node"].as<float>();
    confidence = vm["confidence"].as<bool>();
    output_polygons = vm["output_polygons"].as<bool>();
    manifold = vm["manifold"].as<bool>();

    // Load point cloud
    PointC::Ptr cloud(new PointC);
    if (pcl::io::loadPLYFile<PointT>(input_file_name, *cloud) == -1) {
      throw "Couldn't load input file";
    }

    // Poisson reconstruction
    pcl::Poisson<PointT> poisson;
    poisson.setDepth(depth);
    poisson.setMinDepth(min_depth);
    poisson.setPointWeight(point_weight);
    poisson.setScale(scale);
    poisson.setSolverDivide(solver_divide);
    poisson.setIsoDivide(iso_divide);
    poisson.setSamplesPerNode(samples_per_node);
    poisson.setConfidence(confidence);
    poisson.setOutputPolygons(output_polygons);
    poisson.setManifold(manifold);

    poisson.setInputCloud(cloud);

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    poisson.reconstruct(*mesh);

    // Save
    pcl::io::savePLYFileBinary(output_file_name, *mesh);

    return 0;
  } catch (boost::program_options::error& msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (const char* msg) {
    std::cerr << msg << std::endl;
  }

  return -1;
}
