/*
 * Copyright 2015 (2020), Giacomo Dabisias, Eberty Alves
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @Author
 * Giacomo Dabisias, PhD Student
 * PERCRO, (Laboratory of Perceptual Robotics)
 * Scuola Superiore Sant'Anna
 * via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
 *
 * @Author
 * Eberty Alves, Msc Student
 * PGCOMP, (Programa de Pós-Graduação em Ciência da Computação)
 * Universidade Federal da Bahia
 * Av. Ademar de Barros, s/n - Ondina, Salvador - BA, Brazil
 */

#ifndef KINECT2_H_
#define KINECT2_H_

// C++ standard library
#include <signal.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <limits>
#include <utility>

// Kinect 2
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>

// Point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointC;

bool stop = false;

void sigint_handler(int s) { stop = true; }

class Kinect2 {
 public:
  enum Processor { CPU, OPENCL, OPENGL };

  explicit Kinect2(Processor p, bool mirror = 1)
      : mirror_(mirror),
        listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
        undistorted_(512, 424, 4),
        registered_(512, 424, 4),
        big_mat_(1920, 1082, 4),
        qnan_(std::numeric_limits<float>::quiet_NaN()) {
    signal(SIGINT, sigint_handler);

     libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));

    if (freenect2_.enumerateDevices() == 0) {
      std::cout << "No kinect2 connected!" << std::endl;
      exit(-1);
    }

    serial_ = freenect2_.getDefaultDeviceSerialNumber();
    switch (p) {
      case CPU:
        std::cout << "Creating CPU processor" << std::endl;
        pipeline_ = new libfreenect2::CpuPacketPipeline();
        break;
#ifdef HAVE_OPENCL
      case OPENCL:
        std::cout << "Creating OpenCL processor" << std::endl;
        pipeline_ = new libfreenect2::OpenCLPacketPipeline();
        break;
#endif
      case OPENGL:
        std::cout << "Creating OpenGL processor" << std::endl;
        pipeline_ = new libfreenect2::OpenGLPacketPipeline();
        break;
      default:
        std::cout << "Creating CPU processor" << std::endl;
        pipeline_ = new libfreenect2::CpuPacketPipeline();
        break;
    }

    dev_ = freenect2_.openDevice(serial_, pipeline_);
    dev_->setColorFrameListener(&listener_);
    dev_->setIrAndDepthFrameListener(&listener_);
    dev_->start();

    registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

    prepareMake3D(dev_->getIrCameraParams());
  }

  libfreenect2::Freenect2Device::IrCameraParams getIrParameters() {
    libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
    return ir;
  }

  libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters() {
    libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
    return rgb;
  }

  PointC::Ptr getCloud() {
    const int w = undistorted_.width;
    const int h = undistorted_.height;
    PointC::Ptr cloud(new PointC(w, h));

    return updateCloud(cloud);
  }

  PointC::Ptr updateCloud(PointC::Ptr &cloud) {
    listener_.waitForNewFrame(frames_);
    libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];

    registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_);
    const std::size_t w = undistorted_.width;
    const std::size_t h = undistorted_.height;

    cv::Mat tmp_itD0(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
    cv::Mat tmp_itRGB0(registered_.height, registered_.width, CV_8UC4, registered_.data);

    if (mirror_ == true) {
      cv::flip(tmp_itD0, tmp_itD0, 1);
      cv::flip(tmp_itRGB0, tmp_itRGB0, 1);
    }

    const float *itD0 = reinterpret_cast<float *>(tmp_itD0.ptr());
    const char *itRGB0 = reinterpret_cast<char *>(tmp_itRGB0.ptr());

    PointT *itP = &cloud->points[0];
    bool is_dense = true;

    for (std::size_t y = 0; y < h; ++y) {
      const unsigned int offset = y * w;
      const float *itD = itD0 + offset;
      const char *itRGB = itRGB0 + offset * 4;
      const float dy = rowmap(y);

      for (std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4) {
        const float depth_value = *itD / 1000.0f;

        if (!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)) {
          const float rx = colmap(x) * depth_value;
          const float ry = dy * depth_value;
          itP->z = depth_value;
          itP->x = rx;
          itP->y = ry;

          itP->b = itRGB[0];
          itP->g = itRGB[1];
          itP->r = itRGB[2];
        } else {
          itP->z = qnan_;
          itP->x = qnan_;
          itP->y = qnan_;

          itP->b = qnan_;
          itP->g = qnan_;
          itP->r = qnan_;
          is_dense = false;
        }
      }
    }
    cloud->is_dense = is_dense;
    listener_.release(frames_);
    return cloud;
  }

  void shutDown() {
    dev_->stop();
    dev_->close();
  }

  cv::Mat getColor() {
    listener_.waitForNewFrame(frames_);
    libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
    cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
    cv::Mat r;
    if (mirror_ == true) {
      cv::flip(tmp, r, 1);
    } else {
      r = tmp.clone();
    }

    listener_.release(frames_);
    return std::move(r);
  }

  cv::Mat getDepth() {
    listener_.waitForNewFrame(frames_);
    libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];
    cv::Mat tmp(depth->height, depth->width, CV_8UC4, depth->data);
    cv::Mat r;
    if (mirror_ == true) {
      cv::flip(tmp, r, 1);
    } else {
      r = tmp.clone();
    }

    listener_.release(frames_);
    return std::move(r);
  }

  std::pair<cv::Mat, cv::Mat> getDepthRgb() {
    listener_.waitForNewFrame(frames_);
    libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];
    libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
    registration_->apply(rgb, depth, &undistorted_, &registered_);
    cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
    cv::Mat tmp_color(registered_.height, registered_.width, CV_8UC4, registered_.data);
    cv::Mat r = tmp_color.clone();
    cv::Mat d = tmp_depth.clone();
    if (mirror_ == true) {
      cv::flip(tmp_depth, d, 1);
      cv::flip(tmp_color, r, 1);
    }
    listener_.release(frames_);
    return std::move(std::pair<cv::Mat, cv::Mat>(r, d));
  }

 private:
  void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams &depth_p) {
    const int w = 512;
    const int h = 424;
    float *pm1 = colmap.data();
    float *pm2 = rowmap.data();
    for (int i = 0; i < w; i++) {
      *pm1++ = (i - depth_p.cx + 0.5) / depth_p.fx;
    }
    for (int i = 0; i < h; i++) {
      *pm2++ = (i - depth_p.cy + 0.5) / depth_p.fy;
    }
  }

  bool mirror_;
  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device *dev_ = 0;
  libfreenect2::PacketPipeline *pipeline_ = 0;
  libfreenect2::Registration *registration_ = 0;
  libfreenect2::SyncMultiFrameListener listener_;
  libfreenect2::FrameMap frames_;
  libfreenect2::Frame undistorted_, registered_, big_mat_;
  Eigen::Matrix<float, 512, 1> colmap;
  Eigen::Matrix<float, 424, 1> rowmap;
  std::string serial_;
  int map_[512 * 424];  // will be used in the next libfreenect2 update
  float qnan_;
};

#endif  //  KINECT2_H_
