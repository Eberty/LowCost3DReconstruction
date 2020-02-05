/*
 * Copyright (c) 2020, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Point cloud library
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// TEASER-plusplus
#include <Eigen/Core>
#include <teaser/registration.h>

// Boost
#include <boost/program_options.hpp>

// Typedefs
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;

int main(int argc, char* argv[]) {
  try {
    // Declaration of variables
    std::string src_file_name;
    std::string tgt_file_name;
    std::string output_file_name;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&src_file_name)->required(), "Input cloud file (.ply)")
    ("target,t", po::value<std::string>(&tgt_file_name)->required(), "Input target file (.ply)")
    ("output,o", po::value<std::string>(&output_file_name)->required(), "TXT file containing a 4x4 transformation matrix");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("input") && vm.count("target") && vm.count("output")) {
      src_file_name = vm["input"].as<std::string>();
      tgt_file_name = vm["target"].as<std::string>();
      output_file_name = vm["output"].as<std::string>();
    } else {
      throw std::string("Correct mode of use: " + std::string(argv[0]) + " -i input.ply -t target.ply -o output.txt [opts]");
    }

    PointC::Ptr cloud_src(new PointC);
    PointC::Ptr cloud_tgt(new PointC);

    // Load point clouds data from disk
    if (pcl::io::loadPLYFile<PointT>(src_file_name, *cloud_src) == -1) {
      throw std::string("Couldn't load input cloud file");
    }
    std::cout << "Loaded " << cloud_src->size() << " data points from " << src_file_name << std::endl;

    if (pcl::io::loadPLYFile<PointT>(tgt_file_name, *cloud_tgt) == -1) {
      throw std::string("Couldn't load input target file");
    }
    std::cout << "Loaded " << cloud_tgt->size() << " data points from " << tgt_file_name << std::endl;

    // Populate src & dst with ---------- Correspondences TODO -------------
    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, cloud_src->size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> dst(3, cloud_tgt->size());

    for(size_t i = 0; i < cloud_src->size(); i++) {
      src.col(i) = Eigen::Vector3d(cloud_src->points[i].x, cloud_src->points[i].y, cloud_src->points[i].z);
    }

    for(size_t i = 0; i < cloud_tgt->size(); i++) {
      dst.col(i) = Eigen::Vector3d(cloud_tgt->points[i].x, cloud_tgt->points[i].y, cloud_tgt->points[i].z);
    }

    // Populate solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.cbar2 = 1;
    params.noise_bound = 0.01;
    params.estimate_scaling = false;
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_gnc_factor = 1.4;
    params.rotation_max_iterations = 100;
    params.rotation_cost_threshold = 1e-6;

    // Initialize solver
    teaser::RobustRegistrationSolver solver(params);

    // Solve
    solver.solve(src, dst);

    // Get solution
    teaser::RegistrationSolution solution = solver.getSolution();

    std::cout << "Computed Scale: " << std::endl << solution.scale << std::endl << std::endl;
    std::cout << "Computed Rotation:" << std::endl << solution.rotation << std::endl << std::endl;
    std::cout << "Computed Translation:" << std::endl << solution.translation.transpose() << std::endl;
    
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(solution.translation);
    transform.scale(solution.scale);
    transform.rotate(solution.rotation);
    
    // Computed Transformation (homogeneous matrix)
    std::ofstream file(output_file_name);
    if (file.is_open()) {
      file << transform.matrix();
      file.close();
    } else {
      throw std::string("Unable to open file: " + output_file_name);
    }
    
    return 0; 
   } catch (boost::program_options::error& msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (std::string msg) {
    std::cerr << msg << std::endl;
  }

  return -1;
}
