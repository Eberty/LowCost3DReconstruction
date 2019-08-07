/*
 * Copyright (c) 2019, Eberty Alves
 */

// Point cloud library
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>

// Boost library
#include <boost/program_options.hpp>

// Qt
#include <QApplication>

// Project files
#include "rotate_align.h"

#define NORMALIZE_VALUE 10.0
#define DECIMAL_PLACES 1

RotateAlign::RotateAlign(int argc, char *argv[], QWidget *parent) : QMainWindow(parent), ui(new Ui::RotateAlign) {
  ui->setupUi(this);
  this->setWindowTitle("Rotate align");

  // Verify if everything is ok
  bool ok = parserProgramOptions(argc, argv);
  if (!ok) {
    user_interface = false;
    return;
  }

  // Just save files with defined transformations and not show the GUI
  if (ok && !user_interface) {
    performOperationWithoutGui();
    return;
  }

  // Connect sliders, buttons and their functions
  connect(ui->save_button, SIGNAL(clicked()), this, SLOT(saveFiles()));
  connect(ui->slider_distance_x, SIGNAL(valueChanged(int)), this, SLOT(distanceXSliderValueChanged(int)));
  connect(ui->slider_distance_y, SIGNAL(valueChanged(int)), this, SLOT(distanceYSliderValueChanged(int)));
  connect(ui->slider_distance_z, SIGNAL(valueChanged(int)), this, SLOT(distanceZSliderValueChanged(int)));

  // Set up the QVTK window for point cloud visualization
  viewer.reset(new pcl::visualization::PCLVisualizer("Matrix transformation", false));
  ui->pcl_widget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->pcl_widget->GetInteractor(), ui->pcl_widget->GetRenderWindow());
  viewer->addCoordinateSystem(10.0);
  viewer->setBackgroundColor(0, 0, 0, 0);

  // Defines point clouds
  printAttributes();
  readPointClouds();

  // Set values of distances (x, y, z)
  ui->slider_distance_x->setValue(distance_x * NORMALIZE_VALUE);
  ui->slider_distance_y->setValue(distance_y * NORMALIZE_VALUE);
  ui->slider_distance_z->setValue(distance_z * NORMALIZE_VALUE);

  // Visualization
  viewer->resetCamera();
  ui->pcl_widget->update();
}

RotateAlign::~RotateAlign() {
  delete ui;
  point_clouds.clear();
  transformed_point_clouds.clear();
}

bool RotateAlign::userInterface() {
  return user_interface;
}

void RotateAlign::saveFiles() {
  // Save ply with transformed points for each point cloud read
  for (size_t i = 0; i < num_captures; i++) {
    pcl::io::savePLYFileBinary(capture_name + std::to_string(i * capture_step) + "_rotated.ply", *transformed_point_clouds[i]);
  }

  // Save aligned mesh and quit
  if (accumulated_file) {
    PointC::Ptr accumulated(new PointC);

    pcl::copyPointCloud(*transformed_point_clouds[0], *accumulated);
    for (size_t i = 1; i < num_captures; i++) {
      *accumulated = *accumulated + *transformed_point_clouds[i];
    }

    pcl::io::savePLYFileBinary("accumulated.ply", *accumulated);
  }

  if (userInterface()) {
    QApplication::quit();
  }
}

void RotateAlign::distanceXSliderValueChanged(int value) {
  this->distance_x = value / NORMALIZE_VALUE;
  ui->number_distance_x->display(QString::number(this->distance_x, 'f', DECIMAL_PLACES));
  updatePointClouds();
}

void RotateAlign::distanceYSliderValueChanged(int value) {
  this->distance_y = value / NORMALIZE_VALUE;
  ui->number_distance_y->display(QString::number(this->distance_y, 'f', DECIMAL_PLACES));
  updatePointClouds();
}

void RotateAlign::distanceZSliderValueChanged(int value) {
  this->distance_z = value / NORMALIZE_VALUE;
  ui->number_distance_z->display(QString::number(this->distance_z, 'f', DECIMAL_PLACES));
  updatePointClouds();
}

bool RotateAlign::parserProgramOptions(int argc, char *argv[]) {
  try {
    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("gui,g", "Show a user interface to edit the distances")
    ("accumulated_file,m", "Saves the accumulated mesh in a .ply file")
    ("capture_name,n", po::value<std::string>(&this->capture_name)->default_value("artefact_srmesh_"), "Prefix of saved ply files")
    ("capture_step,s", po::value<uint>(&this->capture_step)->default_value(20), "Angles (in degrees) for each capture")
    ("num_captures,c", po::value<uint>(&this->num_captures)->default_value(18), "Number of captures")
    ("distance_x,x", po::value<double>(&this->distance_x)->default_value(44.0), "Distance in X from kinect to the center of the table")
    ("distance_y,y", po::value<double>(&this->distance_y)->default_value(60.0), "Distance in Y from kinect to the center of the table")
    ("distance_z,z", po::value<double>(&this->distance_z)->default_value(632.5), "Distance in Z from kinect to the center of the table");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return false;
    }

    this->user_interface = vm.count("gui");
    this->accumulated_file = vm.count("accumulated_file");
    this->capture_name = vm["capture_name"].as<std::string>();
    this->capture_step = vm["capture_step"].as<uint>();
    this->num_captures = vm["num_captures"].as<uint>();
    this->distance_x = vm["distance_x"].as<double>();
    this->distance_y = vm["distance_y"].as<double>();
    this->distance_z = vm["distance_z"].as<double>();

    return true;
  } catch (boost::program_options::error &msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (...) {
    std::cerr << "Some error has occurred." << std::endl;
  }
  return false;
}

void RotateAlign::performOperationWithoutGui() {
  printAttributes();
  readPointClouds();
  for (size_t i = 0; i < num_captures; i++) {
    transformPointCloud(i);
  }
  saveFiles();
}

void RotateAlign::printAttributes() {
  std::cout << "Capture name: " << capture_name << std::endl;
  std::cout << "Step: " << capture_step << std::endl;
  std::cout << "N. of captures: " << num_captures << std::endl;
  std::cout << "Save accumulated file: " << (accumulated_file ? "Yes" : "No") << std::endl;
}

void RotateAlign::readPointClouds() {
  for (size_t i = 0; i < num_captures; i++) {
    point_clouds.push_back(PointC::Ptr(new PointC));
    transformed_point_clouds.push_back(PointC::Ptr(new PointC));
    std::string ply_file = std::string(capture_name) + std::to_string(i * capture_step) + ".ply";
    if (pcl::io::loadPLYFile<PointT>(ply_file, *point_clouds[i]) != -1) {
      std::cout << "MESH " << std::to_string(i * capture_step) << " LOADED ALRIGHT!" << std::endl;

      // Cleaning
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*point_clouds[i], *point_clouds[i], indices);

      // Copy to transformed
      pcl::copyPointCloud(*point_clouds[i], *transformed_point_clouds[i]);
      if (userInterface()) {
        addPointCloudToViewer(std::to_string(i * capture_step), i);
      }
    }
  }
}

void RotateAlign::updatePointClouds() {
  // Translation to origin and alignment according to our rotation parameters
  for (size_t i = 0; i < num_captures; i++) {
    transformPointCloud(i);
    PointCloudColorHandler color_hander(transformed_point_clouds[i]);
    viewer->updatePointCloud(transformed_point_clouds[i], color_hander, std::to_string(i * capture_step));
  }
  ui->pcl_widget->update();
}

void RotateAlign::transformPointCloud(size_t index) {
  // Translation to origin and alignment according to rotation parameters
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // Distance from kinect to the center of the table
  transform.translation() << distance_x, distance_y, distance_z;
  pcl::transformPointCloud(*point_clouds[index], *transformed_point_clouds[index], transform);
  // Rotate around center
  transform = Eigen::Affine3f::Identity();
  Eigen::Matrix3f rotation(Eigen::AngleAxisf(-DEG2RAD(index * capture_step + index * 1.17), Eigen::Vector3f::UnitY()));
  transform.rotate(rotation);
  pcl::transformPointCloud(*transformed_point_clouds[index], *transformed_point_clouds[index], transform);
}

void RotateAlign::addPointCloudToViewer(std::string point_cloud_name, int index) {
  PointCloudColorHandler color_hander(transformed_point_clouds[index]);
  viewer->addPointCloud(transformed_point_clouds[index], color_hander, point_cloud_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, point_cloud_name);
}
