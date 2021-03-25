/*
 * Copyright (c) 2018-2019, Eberty Alves, Pedro Raimundo
 */

// C++ standard library
#include <bits/stdc++.h>

// Kinect
#include <libfreenect/libfreenect_sync.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// Boost library
#include <boost/program_options.hpp>

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

void show_depth(uint16_t *depth) {
  cv::Mat depth_mat = cv::Mat(kinect_size, CV_16UC1, depth);
  cv::Mat depth_mat_tmp = cv::Mat(kinect_size, CV_8UC3);
  int i, j;
  for (i = 0; i < kinect_size.height; i++) {
    for (j = 0; j < kinect_size.width; j++) {
      uint16_t depth_in_mm = depth_mat.at<uint16_t>(i, j);
      if (depth_in_mm != 0 && depth_in_mm >= depth_min && depth_in_mm <= depth_max && i >= top_plane &&
          i <= bottom_plane && j >= left_plane && j <= right_plane) {
        uint16_t color = (uint16_t)lerp(255.0, 32.0, (double)depth_in_mm / FREENECT_DEPTH_MM_MAX_VALUE);
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

void show_rgb(uchar *rgb) {
  cv::Mat img_bgr_mat;
  cv::Mat img_rgb_mat = cv::Mat(kinect_size, CV_8UC3, rgb);
  cvtColor(img_rgb_mat, img_bgr_mat, cv::COLOR_RGB2BGR);
  imshow("RGB", img_bgr_mat);
}

// ---------------------------------------------------------
// Name: clean_image
// ---------------------------------------------------------
// Description: manually treshold a CV_16UC1 image, since
// the chances of opencv working for this are kind hit-or-miss
// between versions and operating systems
// ---------------------------------------------------------

cv::Mat clean_image(cv::Mat input_image, int near, int far, int left, int right, int top, int bottom) {
  cv::Mat output_image;
  input_image.copyTo(output_image);
  for (int i = 0; i < output_image.size().height; i++) {
    for (int j = 0; j < output_image.size().width; j++) {
      uint16_t &pixel = output_image.at<uint16_t>(i, j);
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

int save_depth(uint16_t *depth) {
  cv::Mat depth_mat = cv::Mat(kinect_size, CV_16UC1, depth);
  std::ostringstream oss;
  oss << capture_name << "_depth_" << view_type_str(count_depth * capture_step) << ".png";
  depth_mat = clean_image(depth_mat, depth_min, depth_max, left_plane, right_plane, top_plane, bottom_plane);
  imwrite(oss.str(), depth_mat);
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

int save_depth_burst(uint16_t *depth, unsigned int frame_count) {
  cv::Mat depth_mat = cv::Mat(kinect_size, CV_16UC1, depth);
  std::ostringstream oss;
  oss << capture_name << "_burst_" << view_type_str(count_burst * capture_step) << "_" << frame_count << ".png";
  depth_mat = clean_image(depth_mat, depth_min, depth_max, left_plane, right_plane, top_plane, bottom_plane);
  imwrite(oss.str(), depth_mat);
  printf("%s saved!\n", oss.str().c_str());
  fflush(stdout);
  return 1;
}

// ---------------------------------------------------------
// Name: save_rgb
// ---------------------------------------------------------
// Description: create a file and save one rgb frame
// ---------------------------------------------------------

int save_rgb(uchar *rgb) {
  cv::Mat img_rgb_mat = cv::Mat(kinect_size, CV_8UC3, rgb);
  cv::Mat img_bgr_mat;
  std::ostringstream oss;
  oss << capture_name << "_color_" << view_type_str(count_color * capture_step) << ".png";
  cvtColor(img_rgb_mat, img_bgr_mat, cv::COLOR_RGB2BGR);
  imwrite(oss.str(), img_bgr_mat);
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

int save_ply(cv::Mat depth_mat, cv::Mat color_mat, int count) {
  std::ostringstream oss;
  oss << capture_name << "_mesh_" << view_type_str(count * capture_step) << ".ply";

  FILE *fp;

  if ((fp = fopen(oss.str().c_str(), "w")) == NULL) {
    printf("Error: while creating file %s\n", oss.str().c_str());
    return 0;
  }

  // Bilateral filtering of the depth image disabled for now
  // because super resolution ends up smoothing out the noise

  // Normals via cross-product
  cv::Mat normal_mat = cv::Mat(depth_mat.size(), CV_32FC3);
  for (int i = 0; i < depth_mat.size().height; i++) {
    for (int j = 0; j < depth_mat.size().width; j++) {
      uint16_t right = 0, left = 0, down = 0, up = 0;

      if (i > 0 && j > 0 && i < depth_mat.size().height - 1 && j < depth_mat.size().width - 1) {
        right = depth_mat.at<uint16_t>(i + 1, j);
        left = depth_mat.at<uint16_t>(i - 1, j);
        down = depth_mat.at<uint16_t>(i, j + 1);
        up = depth_mat.at<uint16_t>(i, j - 1);
      }

      // Threshold (as per Hinterstoisser et al (2011))
      if (abs(depth_mat.at<uint16_t>(i, j) - right) > 20) {
        right = 0;
      }
      if (abs(depth_mat.at<uint16_t>(i, j) - left) > 20) {
        left = 0;
      }
      if (abs(depth_mat.at<uint16_t>(i, j) - down) > 20) {
        down = 0;
      }
      if (abs(depth_mat.at<uint16_t>(i, j) - up) > 20) {
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
  // Iterate to count the number of vertices, needed for .ply header
  for (int i = 0; i < depth_mat.size().height; i++) {
    for (int j = 0; j < depth_mat.size().width; j++) {
      uint16_t z_in_mm = depth_mat.at<uint16_t>(i, j);

      if (z_in_mm != 0 && z_in_mm >= depth_min && z_in_mm <= depth_max && i >= top_plane && i <= bottom_plane &&
          j >= left_plane && j <= right_plane) {
        num_vertices++;
      }
    }
  }

  // then a second time to write the geometry into the .ply file
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
      uint16_t z_in_mm = depth_mat.at<uint16_t>(i, j);
      double fx = 572.882768, fy = 542.739980, cx = 314.649173, cy = 240.160459;
      if (z_in_mm != 0 && z_in_mm >= depth_min && z_in_mm <= depth_max && i >= top_plane && i <= bottom_plane &&
          j >= left_plane && j <= right_plane) {
        double vx = z_in_mm * (j - cx) * (1 / fx);
        double vy = -z_in_mm * (i - cy) * (1 / fy);
        cv::Vec3b color = color_mat.at<cv::Vec3b>(i, j);
        cv::Vec3f normal = normal_mat.at<cv::Vec3f>(i, j);
        fprintf(fp, "%.6lf %.6lf %.6lf %.6lf %.6lf %.6lf %d %d %d\n", (double)vx, (double)vy, (double)-z_in_mm,
                (double)normal[0], (double)normal[1], (double)normal[2], color[0], color[1], color[2]);
      }
    }
  }

  fclose(fp);
  fflush(fp);

  printf("%s saved!\n", oss.str().c_str());
  fflush(stdout);

  return 1;
}

int main(int argc, char **argv) {
  try {
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
    ("bottom_plane,b", po::value<int>(&bottom_plane)->default_value(480), "Bottom plane threshold in mm");

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

    std::cout << "Capture name: " << capture_name << " | Steps: " << capture_step << std::endl;

    // Set Windows
    cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("DEPTH", cv::WINDOW_AUTOSIZE);

    // Set kinect_size settings
    kinect_size.width = 640;
    kinect_size.height = 480;

    // Trackbar
    cv::createTrackbar("Near Plane (mm)\t", "DEPTH", &depth_min, 4000, NULL);
    cv::createTrackbar("Far Plane (mm)\t", "DEPTH", &depth_max, 4000, NULL);
    cv::createTrackbar("Left Plane (px)\t", "DEPTH", &left_plane, kinect_size.width - 1, NULL);
    cv::createTrackbar("Right Plane (px)\t", "DEPTH", &right_plane, kinect_size.width - 1, NULL);
    cv::createTrackbar("Top Plane (px)\t", "DEPTH", &top_plane, kinect_size.height - 1, NULL);
    cv::createTrackbar("Bottom Plane (px)", "DEPTH", &bottom_plane, kinect_size.height - 1, NULL);

    int key = 0;
    if (freenect_sync_set_tilt_degs(0, 0)) {  // Returns nonzero on error
      throw "No device found";
    }

    // Loop to get captures
    while ((key = cv::waitKey(1))) {
      // Opencv apparently cant decide whether 'no key' is -1 or 255.
      if (key == 255 || key == -1) {
        uint32_t timestamp;
        uint32_t timestamp_depth;
        unsigned char *data;
        uint16_t *depth_data;

        freenect_sync_get_video((void **)(&data), &timestamp, 0, FREENECT_VIDEO_RGB);
        show_rgb(data);
        freenect_sync_get_depth((void **)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
        show_depth(depth_data);
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
          uint32_t timestamp_depth;
          uint16_t *depth_data;
          freenect_sync_get_depth((void **)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
          save_depth(depth_data);
          if (type_of_view == FRONT) count_depth++;
        }

        if (key == BURST || key == ALL) {
          uint32_t timestamp_depth;
          uint16_t *depth_data;
          for (unsigned int i = 0; i < sr_size; i++) {
            freenect_sync_get_depth((void **)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
            save_depth_burst(depth_data, i);
          }
          if (type_of_view == FRONT) count_burst++;
        }

        if (key == COLOR || key == ALL) {
          uint32_t timestamp;
          unsigned char *data;
          freenect_sync_get_video((void **)(&data), &timestamp, 0, FREENECT_VIDEO_RGB);
          save_rgb(data);
          if (type_of_view == FRONT) count_color++;
        }

        if (key == DEPTH_MESH || key == ALL) {
          uint32_t timestamp_depth;
          uint16_t *depth_data;
          freenect_sync_get_depth((void **)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
          uint32_t timestamp_color;
          unsigned char *color_data;
          freenect_sync_get_video((void **)(&color_data), &timestamp_color, 0, FREENECT_VIDEO_RGB);
          save_ply(cv::Mat(kinect_size, CV_16UC1, depth_data), cv::Mat(kinect_size, CV_8UC3, color_data), count_depth_mesh);
          if (type_of_view == FRONT) count_depth_mesh++;
        }
      }
    }

    return 0;
  } catch (boost::program_options::error &msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (...) {
    std::cerr << "Some error has occurred." << std::endl;
  }

  return -1;
}
