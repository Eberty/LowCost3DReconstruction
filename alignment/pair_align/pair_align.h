/*
 * Copyright (c) 2019, Eberty Alves
 */

#ifndef PAIR_ALIGN_H_
#define PAIR_ALIGN_H_

// C++ standard library
#include <bits/stdc++.h>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// UI file
#include "ui_pair_align.h"

// Typedefs
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PointCloudColorHandler;

class PairAlign : public QMainWindow {
  Q_OBJECT

 public:
  PairAlign(int argc, char *argv[], QWidget *parent = 0);
  ~PairAlign();
  bool userInterface();
  std::string eraseLastSubStr(std::string str, const std::string to_erase);

 public Q_SLOTS:
  void saveFiles();
  void xSliderValueChanged(int value);
  void ySliderValueChanged(int value);
  void zSliderValueChanged(int value);
  void rollSliderValueChanged(int value);
  void pitchSliderValueChanged(int value);
  void yawSliderValueChanged(int value);

 protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointC::Ptr point_cloud;
  PointC::Ptr point_cloud_ref;
  PointC::Ptr transformed_point_cloud;

 private:
  bool parserProgramOptions(int argc, char *argv[]);
  void performOperationWithoutGui();

  void readPointClouds();
  void updateView();

  void transformPointCloud();
  void addPointCloudToViewer(PointC::Ptr &pc, const std::string point_cloud_name, const bool is_tranformed);

  // Declaration of variables
  bool user_interface;
  bool b_accumulated_file;
  std::string accumulated_file_name;
  uint view_angle;
  std::string view_name;
  std::string point_cloud_file_name;
  std::string point_cloud_ref_file_name;
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  // Interface
  Ui::PairAlign *ui;
};

#endif  // PAIR_ALIGN_H_
