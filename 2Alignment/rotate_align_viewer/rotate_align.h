/*
 * Copyright (c) 2019, Eberty Alves
 */

#ifndef ROTATE_ALIGN_PCLVIEWER_H_
#define ROTATE_ALIGN_PCLVIEWER_H_

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
#include "ui_rotate_align.h"

// Typedef to short code
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> PointCloudColorHandler;

class RotateAlign : public QMainWindow {
  Q_OBJECT

 public:
  RotateAlign(int argc, char *argv[], QWidget *parent = 0);
  ~RotateAlign();
  bool userInterface();

 public Q_SLOTS:
  void saveFiles();
  void distanceXSliderValueChanged(int value);
  void distanceYSliderValueChanged(int value);
  void distanceZSliderValueChanged(int value);

 protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  std::vector<PointC::Ptr> point_clouds;
  std::vector<PointC::Ptr> transformed_point_clouds;

 private:
  bool parserProgramOptions(int argc, char *argv[]);
  void performOperationWithoutGui();

  void printAttributes();
  void readPointClouds();
  void updatePointClouds();

  void transformPointCloud(size_t index);
  void addPointCloudToViewer(std::string point_cloud_name, int index);

  // Declaration of variables
  bool user_interface;
  bool accumulated_file;
  std::string capture_name;
  uint capture_step;
  uint num_captures;
  double distance_x;
  double distance_y;
  double distance_z;

  // Interface
  Ui::RotateAlign *ui;
};

#endif  // ROTATE_ALIGN_PCLVIEWER_H_
