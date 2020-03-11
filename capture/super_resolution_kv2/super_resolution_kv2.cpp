/*
 * Copyright (c) 2020, Eberty Alves
 */

// OpenMP
#include <omp.h>

// C++ standard library
#include <bits/stdc++.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

// Boost
#include <boost/program_options.hpp>

// Adapted from https://stackoverflow.com/questions/7616511/
float mean(std::vector<float> &v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();
  return mean;
}

int save_ply(cv::Mat depth_mat, std::string filename, float min_value = std::numeric_limits<float>::min(),
             float max_value = std::numeric_limits<float>::max()) {
  FILE *fp;

  if ((fp = fopen(filename.c_str(), "w")) == NULL) {
    printf("Error: while creating file %s\n", filename.c_str());
    return 0;
  }

  // Normals via cross-product
  cv::Mat normal_mat = cv::Mat(depth_mat.size(), CV_32FC3);
  for (int i = 0; i < depth_mat.size().height; i++) {
    for (int j = 0; j < depth_mat.size().width; j++) {
      float right = 0, left = 0, down = 0, up = 0;

      if (i > 0 && j > 0 && i < depth_mat.size().height - 1 && j < depth_mat.size().width - 1) {
        right = depth_mat.at<float>(i + 1, j);
        left = depth_mat.at<float>(i - 1, j);
        down = depth_mat.at<float>(i, j + 1);
        up = depth_mat.at<float>(i, j - 1);
      }

      // Threshold (as per Hinterstoisser et al. (2011))
      if (fabs(depth_mat.at<float>(i, j) - right) > 20) {
        right = 0;
      }
      if (fabs(depth_mat.at<float>(i, j) - left) > 20) {
        left = 0;
      }
      if (fabs(depth_mat.at<float>(i, j) - down) > 20) {
        down = 0;
      }
      if (fabs(depth_mat.at<float>(i, j) - up) > 20) {
        up = 0;
      }

      double dzdx = (right - left) / 2.0;
      double dzdy = (down - up) / 2.0;

      cv::Vec3d d(-dzdx, -dzdy, 1.0);
      cv::Vec3d n = normalize(d);

      normal_mat.at<cv::Vec3f>(i, j) = n;
    }
  }

  uint32_t num_vertices = 0;
  // Iterate to count the number of vertices
  for (int i = 0; i < depth_mat.size().height; i++) {
    for (int j = 0; j < depth_mat.size().width; j++) {
      float z_in_mm = depth_mat.at<float>(i, j);
      if (z_in_mm != 0 && z_in_mm >= min_value && z_in_mm <= max_value) {
        num_vertices++;
      }
    }
  }

  // Then a second time to write the .ply file
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %d\n", num_vertices);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property float nx\n");
  fprintf(fp, "property float ny\n");
  fprintf(fp, "property float nz\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  for (int i = 0; i < depth_mat.size().height; i++) {
    for (int j = 0; j < depth_mat.size().width; j++) {
      float z_in_mm = depth_mat.at<float>(i, j);
      double fx = 365.531799, fy = 365.531799, cx = 256.136810, cy = 206.013901;
      if (z_in_mm != 0 && z_in_mm >= min_value && z_in_mm <= max_value) {
        double vx = z_in_mm * (j - cx) * (1 / fx);
        double vy = -z_in_mm * (i - cy) * (1 / fy);
        cv::Vec3f normal = normal_mat.at<cv::Vec3f>(i, j);
        fprintf(fp, "%.6lf %.6lf %.6lf %.6lf %.6lf %.6lf %d %d %d\n", (double)vx, (double)vy, (double)-z_in_mm,
                (double)normal[0], (double)normal[1], (double)normal[2], 128, 128, 128);
      }
    }
  }

  fclose(fp);
  fflush(fp);

  printf("%s saved!\n", filename.c_str());
  fflush(stdout);

  return 1;
}

int main(int argc, char **argv) {
  try {
    // Declaration of variables
    std::string capture_name;
    uint capture_step;
    uint num_captures;
    uint sr_size = 16;
    int resample_factor = 4;

    uint view_angle;
    std::string view_name;
    bool single_view;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("capture_name,n", po::value<std::string>(&capture_name)->default_value("artefact"), "Prefix of saved files")
    ("capture_step,s", po::value<uint>(&capture_step)->default_value(20), "Angles (in degrees) for each capture")
    ("num_captures,c", po::value<uint>(&num_captures)->default_value(18), "Number of view points (anlges)")
    ("sr_size,z", po::value<uint>(&sr_size)->default_value(16), "Number of sr images for each capture")
    ("resample_factor,f", po::value<int>(&resample_factor)->default_value(4), "Resample factor")
    ("view_angle,a", po::value<uint>(&view_angle), "Make SR just for an unique view angle")
    ("top,t", "Make SR for top view")
    ("bottom,b", "Make SR for bottom view");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Super-resolution tool to reconstruct detail and obtain high-resolution "
                << "depth images from the acquired image bursts." << std::endl
                << std::endl;
      std::cout << desc << std::endl;
      return 0;
    }

    capture_name = vm["capture_name"].as<std::string>();
    capture_step = vm["capture_step"].as<uint>();
    num_captures = vm["num_captures"].as<uint>();
    sr_size = vm["sr_size"].as<uint>();
    resample_factor = vm["resample_factor"].as<int>();

    if (vm.count("view_angle") || vm.count("top") || vm.count("bottom")) {
      if ((vm.count("view_angle") && vm.count("top")) || (vm.count("view_angle") && vm.count("bottom")) ||
          (vm.count("top") && vm.count("bottom"))) {
        std::cout << "The command options: 'view_angle', 'top' and 'bottom'"
                  << " can not be used together. Choose only one of them" << std::endl;
        return -1;
      }

      std::cout << "When using the command options: 'view_angle', 'top' or 'bottom'"
                << ", the options 'capture_step' and 'num_captures' are ignored" << std::endl;
      capture_step = 0;
      num_captures = 1;

      if (vm.count("view_angle")) {
        view_name = std::to_string(vm["view_angle"].as<uint>() % 360);
      } else if (vm.count("top")) {
        view_name = "top";
      } else if (vm.count("bottom")) {
        view_name = "bottom";
      }

      single_view = true;
    } else {
      single_view = false;
    }

    std::cout << "Capture name: " << capture_name << std::endl;
    if (single_view) {
      std::cout << "View angle: " << view_name << std::endl;
    } else {
      std::cout << "Step: " << capture_step << std::endl;
      std::cout << "N. of captures: " << num_captures << std::endl;
    }
    std::cout << std::endl;

    for (size_t captures = 0; captures < num_captures; captures++) {
      if (!single_view) {
        view_name = std::to_string(captures * capture_step);
      }

      std::vector<cv::Mat> lr_images(sr_size);            // stored as float
      std::vector<cv::Mat> lr_images_upsampled(sr_size);  // stored as float
      std::vector<cv::Mat> alignment_matrices(sr_size);   // affine transform matrices
      cv::Mat hr_image;

      float global_min = std::numeric_limits<float>::max(), global_max = std::numeric_limits<float>::min();

      // Pre-processing Phase - get minimum and maximum values from the LR images
      // we use this information to remove extreme values from the HR image because
      // some non-linear noise might be added on the resampling and fusion phases
      for (size_t i = 0; i < sr_size; i++) {
        std::ostringstream oss_in;
        oss_in << capture_name << "_burst_" << view_name << "_" << i << ".png";
        cv::Mat depth = cv::imread(oss_in.str(), cv::IMREAD_UNCHANGED);
        if (depth.empty()) {
          std::cout << "Image can not be read: " << oss_in.str() << std::endl;
          return -1;
        }
        lr_images[i] = cv::Mat(depth.rows, depth.cols, CV_32FC1, depth.data).clone();
        alignment_matrices[i] = cv::Mat::eye(2, 3, CV_32F);

        double local_min, local_max;
        cv::Mat mask = lr_images[i] > 0;

        minMaxLoc(lr_images[i], &local_min, &local_max, NULL, NULL, mask);

        global_min = local_min < global_min ? local_min : global_min;
        global_max = local_max > global_max ? local_max : global_max;
      }

      printf("Observed Global Min: %.0lf\n", global_min);
      printf("Observed Global Max: %.0lf\n", global_max);

// Registration Phase - Subpixel registration of all LR images to the first
#pragma omp parallel for
      for (size_t i = 0; i < sr_size; i++) {
        cv::Mat template_image = lr_images[0];  // first LR image
        findTransformECC(template_image, lr_images[i], alignment_matrices[i], cv::MOTION_AFFINE,
                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1E-12));
        std::cout << "View(" << view_name << ") Alignment " << i << "-> 0 = " << alignment_matrices[i] << std::endl;
      }

      // Upsample + Warp Phase - Upsample all LR images and use registration
      // information technically the resize operation should happen before the
      // warpAffine, but this is not working in this version of the program, whilst
      // it worked on an older version, ignoring this for now since the results are
      // good
      for (size_t i = 0; i < sr_size; i++) {
        // Warp - Simplified to rigid transform because we aim to have as little
        // translation and rotation between the LR images as possible while still
        // modifying the intrinsics enough to have complementary data
        if (i > 0) {
          warpAffine(lr_images[i], lr_images[i], alignment_matrices[i], lr_images[i].size(), cv::WARP_INVERSE_MAP);
        }

        // Upsample - can use pyramids or perform a simple scale operation
        // results were exactly the same, still gotta find out why
        cv::resize(lr_images[i], lr_images_upsampled[i], lr_images[i].size() * resample_factor, 0, 0,
                   cv::INTER_NEAREST);
      }

      // Reconstruction Phase - For now, simply averaging the images, this reduces
      // the impact of the additive noise from the kinect sensor on the HR image
      // (Richardt). Nonlinear noise generated during the other phases will be
      // removed using the previously established limits of the depth info; i.e., we
      // can not generate or reconstruct information outside of the observed volume.
      // This implementation performs poorly in case the images are not very well
      // aligned. In fact, even changing the ECC epsilon from 1E-12 to 1E-06
      // adversely affected the results
      /*hr_image = cv::Mat(lr_images_upsampled[0].size(),CV_32FC1);
      for(size_t i = 0; i < sr_size; i++) {
        addWeighted(hr_image, 1.0, lr_images_upsampled[i], (1.0 / sr_size), 0, hr_image);
      }*/

      // New approach -- zero-elimination (ZE) averaging
      hr_image = cv::Mat(lr_images_upsampled[0].size(), CV_32FC1);
      for (int i = 0; i < hr_image.size().height; i++) {
        for (int j = 0; j < hr_image.size().width; j++) {
          std::vector<float> candidate_pixels;
          for (size_t k = 0; k < sr_size; k++) {
            float depth = lr_images_upsampled[k].at<float>(i, j);
            if (depth > 0) {
              candidate_pixels.push_back(depth);
            }
          }
          if (candidate_pixels.size() > 0) {
            float &pix = hr_image.at<float>(i, j);
            pix = mean(candidate_pixels);
          }
        }
      }

      // Downsample the HR image back to original size to keep it valid within the
      // coordinate system of the Microsoft Kinect sensor - lanczos removes some
      // artifacts with no apparent downsides
      cv::resize(hr_image, hr_image, hr_image.size() / resample_factor, 0, 0, cv::INTER_LANCZOS4);
      std::ostringstream oss_out;
      oss_out << capture_name << "_srdepth_" << view_name << ".png";
      imwrite(oss_out.str(), cv::Mat(hr_image.rows, hr_image.cols, CV_8UC4, hr_image.data));
      oss_out.str(std::string());
      oss_out << capture_name << "_srmesh_" << view_name << ".ply";
      save_ply(hr_image, oss_out.str(), global_min, global_max);
      std::cout << std::endl;
    }

    return 0;
  } catch (boost::program_options::error &msg) {
    std::cout << "ERROR: " << msg.what() << std::endl;
  } catch (cv::Exception &msg) {
    std::cout << "ERROR: " << msg.what() << std::endl;
  } catch (...) {
    std::cout << "Some error has occurred." << std::endl;
  }
  return -1;
}
