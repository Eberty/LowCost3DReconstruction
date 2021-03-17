/*
 * Copyright (c) 2020, Eberty Alves
 */

// C++ standard library
#include <bits/stdc++.h>

// Boost
#include <boost/program_options.hpp>

struct Cameras {
  float f, q[9], c[3], d[2];
};

bool bundlerTxt(const std::string& input_file_path, const std::string& output_file_path,
                const std::vector<std::string>& images_file_name) {
  std::ifstream input_file(input_file_path);
  std::ofstream output_file(output_file_path);

  if (input_file.is_open() && output_file.is_open()) {
    std::cout << "Saving: " << output_file_path << std::endl;
    std::string token;

    while (std::getline(input_file, token)) {
      output_file << token << '\n';
    }

    for (size_t i = 0; i < images_file_name.size(); i++) {
      output_file << images_file_name[i] << '\n';
    }

    input_file.close();
    output_file.close();
    return true;
  } else {
    std::cout << "Unable to open " << input_file_path << " or " << output_file_path << std::endl;
    return false;
  }
}

bool bundlerOut(const std::string& meshlab_file_path, const std::string& input_file_path,
                const std::string& output_file_path, std::vector<Cameras>& cameras) {  // NOLINT
  std::ofstream output_file(output_file_path);
  if (!output_file.is_open()) {
    std::cout << "Unable to open file: " << output_file_path << std::endl;
    return false;
  } else {
    std::cout << "Saving: " << output_file_path << std::endl;
  }

  {
    std::ifstream input_file(meshlab_file_path);
    if (input_file.is_open()) {
      std::string token;
      while (input_file.peek() == '#') {
        std::getline(input_file, token);
      }

      int ncam = 0, npoint = 0;
      input_file >> ncam >> npoint;

      cameras.resize(ncam);
      for (int i = 0; i < ncam; i++) {
        input_file >> cameras[i].f >> cameras[i].d[0] >> cameras[i].d[1];
        for (int j = 0; j < 9; j++) {
          input_file >> cameras[i].q[j];
        }
        input_file >> cameras[i].c[0] >> cameras[i].c[1] >> cameras[i].c[2];
      }

      input_file.close();
    } else {
      std::cout << "Unable to open file: " << meshlab_file_path << std::endl;
      return false;
    }
  }

  {
    std::ifstream input_file(input_file_path);
    if (input_file.is_open()) {
      std::string token;
      while (input_file.peek() == '#') {
        std::getline(input_file, token);
        output_file << token << '\n';
      }

      int ncam = 0, npoint = 0;
      input_file >> ncam >> npoint;
      input_file.ignore();

      output_file << ncam + cameras.size() << " " << npoint << '\n';
      for (int i = 0; i < ncam; i++) {
        for (int j = 0; j < 5; j++) {
          std::getline(input_file, token);
          output_file << token << '\n';
        }
      }

      for (size_t i = 0; i < cameras.size(); i++) {
        output_file << cameras[i].f << ' ' << cameras[i].d[0] << ' ' << cameras[i].d[1] << '\n';
        for (int j = 0; j < 9; j++) {
          output_file << cameras[i].q[j] << (((j % 3) == 2) ? '\n' : ' ');
        }
        output_file << cameras[i].c[0] << ' ' << cameras[i].c[1] << ' ' << cameras[i].c[2] << '\n';
      }

      while (std::getline(input_file, token)) {
        output_file << token << '\n';
      }

      input_file.close();
    } else {
      std::cout << "Unable to open file: " << input_file_path << std::endl;
      return false;
    }
  }

  output_file.close();
  return true;
}

int compare(std::string s1, std::string s2) {
  std::transform(s1.begin(), s1.end(), s1.begin(), ::tolower);
  std::transform(s2.begin(), s2.end(), s2.begin(), ::tolower);
  return (s1.compare(s2) == 0);
}

void getFileNameAndPath(const std::string& str, std::string& file_name, std::string& path) {  // NOLINT
  size_t pos = 0;
  if ((pos = str.find_last_of("/")) != std::string::npos) {
    path = str.substr(0, pos + 1);
    file_name = str.substr(pos + 1, str.length());
  } else {
    path = "";
    file_name = str;
  }
}

int main(int argc, char* argv[]) {
  try {
    // Declaration of variables
    std::string meshlab_bundle;
    std::vector<std::string> images;

    std::string bundle_file_name;
    std::string list_file_name;

    std::string output_prefix;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("meshlab_bundle,m", po::value<std::string>(&meshlab_bundle)->required(), "Meshlab bundle file (.out)")
    ("images,i", po::value<std::vector<std::string>>(&images)->multitoken(), "Images used on meshlab bundle file")
    ("bundle,b", po::value<std::string>(&bundle_file_name)->required(), "Original bundle file (.out)")
    ("list,l", po::value<std::string>(&list_file_name)->required(), "Original list file (.txt)")
    ("prefix,p", po::value<std::string>(&output_prefix)->default_value("merged"), "Prefix for output files");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Merge bundle files" << std::endl << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    if (vm.count("meshlab_bundle") && vm.count("images") && vm.count("bundle") && vm.count("list")) {
      meshlab_bundle = vm["meshlab_bundle"].as<std::string>();
      images = vm["images"].as<std::vector<std::string>>();
      bundle_file_name = vm["bundle"].as<std::string>();
      list_file_name = vm["list"].as<std::string>();
      output_prefix = vm["prefix"].as<std::string>();
    } else {
      throw std::string("Correct mode of use: " + std::string(argv[0]) +
                        " -i <vector_images.png> -m meshlab_bundle.out -b bundle.out -l bundle-list.txt");
    }

    if (!compare(meshlab_bundle.substr(meshlab_bundle.find_last_of(".") + 1), "out") ||
        !compare(bundle_file_name.substr(bundle_file_name.find_last_of(".") + 1), "out") ||
        !compare(list_file_name.substr(list_file_name.find_last_of(".") + 1), "txt")) {
      throw std::string("Extension on input files are incorrect");
    }

    std::vector<Cameras> new_cameras;
    std::string file_name, path;

    getFileNameAndPath(bundle_file_name, file_name, path);
    std::string output_bundler_path = path + output_prefix + "." + file_name;
    if (!bundlerOut(meshlab_bundle, bundle_file_name, output_bundler_path, new_cameras)) {
      throw std::string("Error while writing bundler *.out file");
    }

    if (new_cameras.size() != images.size()) {
      throw std::string("Image list must have the same number of cameras of " + meshlab_bundle);
    }

    getFileNameAndPath(list_file_name, file_name, path);
    std::string output_list_path = path + output_prefix + "." + file_name;
    if (!bundlerTxt(list_file_name, output_list_path, images)) {
      throw std::string("Error while writing bundler *.txt file");
    }

    return 0;
  } catch (boost::program_options::error& msg) {
    std::cout << "ERROR: " << msg.what() << std::endl;
  } catch (std::string msg) {
    std::cout << msg << std::endl;
  }

  return -1;
}
