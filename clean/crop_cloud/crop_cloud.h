/*
 * Copyright (c) 2019, Eberty Alves
 */

#ifndef CROP_CLOUD_H_
#define CROP_CLOUD_H_

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
#include "ui_crop_cloud.h"

// Typedefs
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PointCloudColorHandler;

class CropCloud : public QMainWindow {
  Q_OBJECT

 public:
  CropCloud(int argc, char *argv[], QWidget *parent = 0);
  ~CropCloud();
  bool userInterface();
  std::string eraseLastSubStr(std::string str, const std::string to_erase);

 public Q_SLOTS:
  void cropAndSave();
  void minXSliderValueChanged(int value);
  void minYSliderValueChanged(int value);
  void minZSliderValueChanged(int value);
  void maxXSliderValueChanged(int value);
  void maxYSliderValueChanged(int value);
  void maxZSliderValueChanged(int value);

 protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointC::Ptr point_cloud;
  PointC::Ptr point_cloud_inside;
  PointC::Ptr point_cloud_outside;

 private:
  bool parserProgramOptions(int argc, char *argv[]);
  void performOperationWithoutGui();

  bool readPointCloud();
  void updateView();

  void drawPointCloud(PointC::Ptr &pc, const std::string point_cloud_name, const bool is_outside);
  void drawCube();

  // Declaration of variables
  std::string point_cloud_file;
  bool user_interface;
  bool removed_points_file;
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;

  // Interface
  Ui::CropCloud *ui;
};

#endif  // CROP_CLOUD_H_
