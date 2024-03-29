/*
 * Copyright (c) 2020, Eberty Alves
 */

// Kinect
#include "kinect2.h"

// C++ standard library
#include <bits/stdc++.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// Boost library
#include <boost/program_options.hpp>

// Point cloud library
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/impl/io.hpp>
#include <pcl/search/impl/search.hpp>

// Maximum value that a float pixel will take on in the buffer
#define KINECT2_DEPTH_MM_MAX_VALUE 4500.0

// Escape closes the application
#define CLOSE 27

// Image capture keys
#define DEPTH 'd'
#define COLOR 'c'
#define BURST 'b'

// Mesh capture key
#define DEPTH_MESH 'm'

// Perform all captures
#define ALL 'a'

// SR Size
static unsigned int sr_size;

// ENUM viewers
enum { TOP = 1, FRONT = 0, BOTTOM = -1 };
static int type_of_view = FRONT;

// Counters
static unsigned int count_depth = 0;
static unsigned int count_burst = 0;
static unsigned int count_color = 0;
static unsigned int count_depth_mesh = 0;

// Captures settings
static std::string capture_name;
static unsigned int capture_step;

// Depth thresholds in mm
int depth_min;
int depth_max;
int left_plane;
int right_plane;
int top_plane;
int bottom_plane;

// OpenCV variables
static cv::Size kinect_size;

// ---------------------------------------------------------
// Name: lerp
// ---------------------------------------------------------
// Description: Make a linear interpolation between two
//              variables a and b given fraction f
// ---------------------------------------------------------

inline double lerp(double a, double b, double f) {
  return (a * (1.0 - f)) + (b * f);
}

// ---------------------------------------------------------
// Name: view_type_str
// ---------------------------------------------------------
// Description: Convert the view type into a string
// ---------------------------------------------------------

std::string view_type_str(int value) {
  if (type_of_view == TOP) {
    return "top";
  }
  if (type_of_view == BOTTOM) {
    return "bottom";
  }
  return std::to_string(value % 360);
}

// ---------------------------------------------------------
// Name: show_depth
// ---------------------------------------------------------
// Description: Transform a depth frame information into color
//              and show as a video
// ---------------------------------------------------------

void show_depth(const cv::Mat &depth) {
  cv::Mat depth_mat_tmp = cv::Mat(depth.size(), CV_8UC3);
  int i, j;
  for (i = 0; i < depth_mat_tmp.size().height; i++) {
    for (j = 0; j < depth_mat_tmp.size().width; j++) {
      float depth_in_mm = depth.at<float>(i, j);
      if (depth_in_mm != 0 && depth_in_mm >= depth_min && depth_in_mm <= depth_max && i >= top_plane &&
          i <= bottom_plane && j >= left_plane && j <= right_plane) {
        uint16_t color = (uint16_t)lerp(32.0, 255.0, depth_in_mm / KINECT2_DEPTH_MM_MAX_VALUE);
        cv::Vec3b &pixel_color = depth_mat_tmp.at<cv::Vec3b>(i, j);
        pixel_color[0] = color;
        pixel_color[1] = color;
        pixel_color[2] = color;
      } else {
        cv::Vec3b &pixel_color = depth_mat_tmp.at<cv::Vec3b>(i, j);
        pixel_color[0] = 0;
        pixel_color[1] = 0;
        pixel_color[2] = 0;
      }
    }
  }
  cv::imshow("DEPTH", depth_mat_tmp);
}

// ---------------------------------------------------------
// Name: show_rgb
// ---------------------------------------------------------
// Description: show rgb frame as a video
// ---------------------------------------------------------

void show_rgb(const cv::Mat &rgb) {
  cv::Mat img_bgr_mat;
  cv::resize(rgb, img_bgr_mat, cv::Size(), 0.3, 0.3, cv::INTER_CUBIC);
  imshow("RGB", img_bgr_mat);
}

// ---------------------------------------------------------
// Name: clean_image
// ---------------------------------------------------------
// Description: manually treshold a CV_32FC1 image, since
// the chances of opencv working for this are kind hit-or-miss
// between versions and operating systems
// ---------------------------------------------------------

cv::Mat clean_image(cv::Mat input_image, int near, int far, int left, int right, int top, int bottom) {
  cv::Mat output_image;
  input_image.copyTo(output_image);
  for (int i = 0; i < output_image.size().height; i++) {
    for (int j = 0; j < output_image.size().width; j++) {
      float &pixel = output_image.at<float>(i, j);
      if (pixel > far || pixel < near || i < top || i > bottom || j < left || j > right) {
        pixel = 0;
      }
    }
  }
  return output_image;
}

// ---------------------------------------------------------
// Name: save_depth
// ---------------------------------------------------------
// Description: create a file and save one depth frame
// ---------------------------------------------------------

int save_depth(const cv::Mat &depth) {
  std::ostringstream oss;
  oss << capture_name << "_depth_" << view_type_str(count_depth * capture_step) << ".png";
  cv::Mat depth_mat = clean_image(depth, depth_min, depth_max, left_plane, right_plane, top_plane, bottom_plane);
  depth_mat /= KINECT2_DEPTH_MM_MAX_VALUE;
  imwrite(oss.str(), cv::Mat(depth_mat.rows, depth_mat.cols, CV_8UC4, depth_mat.data));
  printf("%s saved!\n", oss.str().c_str());
  fflush(stdout);
  return 1;
}

// ---------------------------------------------------------
// Name: save_depth_burst
// ---------------------------------------------------------
// Description: create a file and save one depth frame from
//              a burst sequence
// ---------------------------------------------------------

int save_depth_burst(const cv::Mat &depth, const unsigned int frame_count) {
  std::ostringstream oss;
  oss << capture_name << "_burst_" << view_type_str(count_burst * capture_step) << "_" << frame_count << ".png";
  cv::Mat depth_mat = clean_image(depth, depth_min, depth_max, left_plane, right_plane, top_plane, bottom_plane);
  depth_mat /= KINECT2_DEPTH_MM_MAX_VALUE;
  imwrite(oss.str(), cv::Mat(depth_mat.rows, depth_mat.cols, CV_8UC4, depth_mat.data));
  printf("%s saved!\n", oss.str().c_str());
  fflush(stdout);
  return 1;
}

// ---------------------------------------------------------
// Name: save_rgb
// ---------------------------------------------------------
// Description: create a file and save one rgb frame
// ---------------------------------------------------------

int save_rgb(const cv::Mat &rgb) {
  std::ostringstream oss;
  oss << capture_name << "_color_" << view_type_str(count_color * capture_step) << ".png";
  imwrite(oss.str(), rgb);
  printf("%s saved!\n", oss.str().c_str());
  fflush(stdout);
  return 1;
}

// ---------------------------------------------------------
// Name: save_ply
// ---------------------------------------------------------
// Description: create a file and save a .ply point cloud
//              with both depth and color information.
// ---------------------------------------------------------

int save_ply(const PointC::Ptr cloud, int count, Kinect2 &kinect) {
  std::ostringstream oss;
  oss << capture_name << "_mesh_" << view_type_str(count * capture_step) << ".ply";

  PointC::Ptr output_cloud(new PointC());
  pcl::copyPointCloud<PointT, PointT>(*cloud, *output_cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*output_cloud, *output_cloud, indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  libfreenect2::Freenect2Device::IrCameraParams ir = kinect.getIrParameters();

  for (std::size_t i = 0; i < output_cloud->points.size(); i++) {
    float x = (((output_cloud->points[i].x) / (-output_cloud->points[i].z)) * ir.fx) - 0.5 + ir.cx;
    float y = (((-output_cloud->points[i].y) / (-output_cloud->points[i].z)) * ir.fy) - 0.5 + ir.cy;
    float depth_value = (-output_cloud->points[i].z) * 1000.0;
    if (depth_value != 0 && depth_value >= depth_min && depth_value <= depth_max && y >= top_plane &&
        y <= bottom_plane && x >= left_plane && x <= right_plane) {
      inliers->indices.push_back(i);
    }
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(output_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*output_cloud);

  double scale = 1000.0 / KINECT2_DEPTH_MM_MAX_VALUE;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform(0, 0) = transform(1, 1) = transform(2, 2) = scale;
  pcl::transformPointCloud(*output_cloud, *output_cloud, transform);

  typedef pcl::PointCloud<pcl::Normal> PointN;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(output_cloud);
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);
  PointN::Ptr normals(new PointN);
  ne.setKSearch(50);
  ne.useSensorOriginAsViewPoint();
  ne.compute(*normals);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*output_cloud, *normals, *cloud_normals);
  pcl::io::savePLYFileBinary(oss.str().c_str(), *cloud_normals);

  printf("%s saved!\n", oss.str().c_str());
  fflush(stdout);

  return 1;
}

int main(int argc, char *argv[]) {
  try {
    // Declaration of variables
    unsigned int processor_int;

    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("capture_name,n", po::value<std::string>(&capture_name)->default_value("artefact"), "Name for saved files")
    ("capture_step,s", po::value<unsigned int>(&capture_step)->default_value(20), "Angles (degrees) for each capture")
    ("sr_size,z", po::value<unsigned int>(&sr_size)->default_value(16), "Number of sr images for each capture")
    ("depth_min,m", po::value<int>(&depth_min)->default_value(0), "Minimum value for depth (threshold)")
    ("depth_max,M", po::value<int>(&depth_max)->default_value(790), "Maximum value for depth (threshold)")
    ("left_plane,l", po::value<int>(&left_plane)->default_value(0), "Left plane threshold in mm")
    ("right_plane,r", po::value<int>(&right_plane)->default_value(640), "Right plane threshold in mm")
    ("top_plane,t", po::value<int>(&top_plane)->default_value(0), "Top plane threshold in mm")
    ("bottom_plane,b", po::value<int>(&bottom_plane)->default_value(480), "Bottom plane threshold in mm")
    ("processor,p", po::value<unsigned int>(&processor_int)->default_value(2), "Processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout
          << "Connect the Kinect on the laptop, launch this depth acquisition tool and configure the bounding box of "
          << "the reconstruction volume to minimize the amount of extraneous information captured. Acquire the depth "
          << "and color images corresponding to each lateral view. Manually position the Kinect to "
          << "acquire views from the top and bottom of the object, when necessary." << std::endl
          << std::endl;
      std::cout << desc << std::endl;
      std::cout << std::endl << "Values: " << std::endl;
      std::cout << "  Image capture keys:    " << std::endl;
      std::cout << "              Depth      " << DEPTH << std::endl;
      std::cout << "              Color      " << COLOR << std::endl;
      std::cout << "              Burst      " << BURST << std::endl;
      std::cout << "              Depth mesh " << DEPTH_MESH << std::endl;
      std::cout << "  Perform all captures:  " << ALL << std::endl;
      std::cout << "  View type:             "
                << "Top(8), Front(*), Bottom(2)" << std::endl;
      std::cout << "  Close the application: " << CLOSE << std::endl;
      return 0;
    }

    capture_name = vm["capture_name"].as<std::string>();

    capture_step = vm["capture_step"].as<unsigned int>();
    if (capture_step <= 0 || capture_step >= 360) {
      std::cout << "capture_step must be in (0, 360) range, the value will be setting to 1." << std::endl;
      capture_step = 1;
    }

    sr_size = vm["sr_size"].as<unsigned int>();
    depth_min = vm["depth_min"].as<int>();
    depth_max = vm["depth_max"].as<int>();
    left_plane = vm["left_plane"].as<int>();
    right_plane = vm["right_plane"].as<int>();
    top_plane = vm["top_plane"].as<int>();
    bottom_plane = vm["bottom_plane"].as<int>();

    processor_int = vm["processor"].as<unsigned int>();
    if (processor_int < Kinect2::Processor::CPU || processor_int > Kinect2::Processor::OPENGL) {
      std::cout << "Processor must be 0 [CPU], 1 [OPENCL] or 2 [OPENGL], the value will be setting to 2." << std::endl;
      processor_int = 2;
    }
    Kinect2::Processor freenectprocessor = static_cast<Kinect2::Processor>(processor_int);

    std::cout << "Capture name: " << capture_name << " | Steps: " << capture_step << std::endl;

    // Set Windows
    cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("DEPTH", cv::WINDOW_AUTOSIZE);

    // Set kinect settings
    kinect_size.width = 512;
    kinect_size.height = 424;

    // Trackbar
    cv::createTrackbar("Near Plane (mm)\t", "DEPTH", &depth_min, 4500, NULL);
    cv::createTrackbar("Far Plane (mm)\t", "DEPTH", &depth_max, 4500, NULL);
    cv::createTrackbar("Left Plane (px)\t", "DEPTH", &left_plane, kinect_size.width - 1, NULL);
    cv::createTrackbar("Right Plane (px)\t", "DEPTH", &right_plane, kinect_size.width - 1, NULL);
    cv::createTrackbar("Top Plane (px)\t", "DEPTH", &top_plane, kinect_size.height - 1, NULL);
    cv::createTrackbar("Bottom Plane (px)", "DEPTH", &bottom_plane, kinect_size.height - 1, NULL);

    Kinect2 kinect(freenectprocessor);
    PointC::Ptr cloud = kinect.getCloud();

    // Loop to get captures
    int key = 0;
    while ((key = cv::waitKey(1))) {
      // Opencv apparently cant decide whether 'no key' is -1 or 255.
      if (key == 255 || key == -1) {
        show_rgb(kinect.getColor());
        show_depth(kinect.getDepth());
      } else {
        if (key == CLOSE) {
          break;
        }

        if (key >= '0' && key <= '9') {
          if (key == '8') {
            type_of_view = TOP;
            std::cout << "View: TOP" << std::endl;
          } else if (key == '2') {
            type_of_view = BOTTOM;
            std::cout << "View: BOTTOM" << std::endl;
          } else {
            type_of_view = FRONT;
            std::cout << "View: FRONT " << (count_depth * capture_step) << std::endl;
          }
        }

        if (key == ALL && type_of_view == FRONT) {
          std::vector<unsigned int> v = {count_depth, count_burst, count_color, count_depth_mesh};
          unsigned int min_value = *std::min_element(v.begin(), v.end());
          count_depth = count_burst = count_color = count_depth_mesh = min_value;
        }

        if (key == DEPTH || key == ALL) {
          save_depth(kinect.getDepth());
          if (type_of_view == FRONT) count_depth++;
        }

        if (key == BURST || key == ALL) {
          for (unsigned int i = 0; i < sr_size; i++) {
            save_depth_burst(kinect.getDepth(), i);
          }
          if (type_of_view == FRONT) count_burst++;
        }

        if (key == COLOR || key == ALL) {
          save_rgb(kinect.getColor());
          if (type_of_view == FRONT) count_color++;
        }

        if (key == DEPTH_MESH || key == ALL) {
          cloud = kinect.updateCloud(cloud);
          save_ply(cloud, count_depth_mesh, kinect);
          if (type_of_view == FRONT) count_depth_mesh++;
        }
      }
    }

    kinect.shutDown();

    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "An unknown error has occurred." << std::endl;
  }

  return -1;
}
