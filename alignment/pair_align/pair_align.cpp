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
#include "pair_align.h"

#define NORMALIZE_SCALE 10000.0
#define SCALE_DECIMAL_PLACES 4

#define NORMALIZE_VALUE 10.0
#define DECIMAL_PLACES 1

PairAlign::PairAlign(int argc, char *argv[], QWidget *parent) : QMainWindow(parent), ui(new Ui::PairAlign) {
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
  connect(ui->slider_x, SIGNAL(valueChanged(int)), this, SLOT(xSliderValueChanged(int)));
  connect(ui->slider_y, SIGNAL(valueChanged(int)), this, SLOT(ySliderValueChanged(int)));
  connect(ui->slider_z, SIGNAL(valueChanged(int)), this, SLOT(zSliderValueChanged(int)));
  connect(ui->slider_roll, SIGNAL(valueChanged(int)), this, SLOT(rollSliderValueChanged(int)));
  connect(ui->slider_pitch, SIGNAL(valueChanged(int)), this, SLOT(pitchSliderValueChanged(int)));
  connect(ui->slider_yaw, SIGNAL(valueChanged(int)), this, SLOT(yawSliderValueChanged(int)));
  connect(ui->slider_scale, SIGNAL(valueChanged(int)), this, SLOT(scaleSliderValueChanged(int)));

  // Set up the QVTK window for point cloud visualization
  viewer.reset(new pcl::visualization::PCLVisualizer("Matrix transformation", false));
  ui->pcl_widget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->pcl_widget->GetInteractor(), ui->pcl_widget->GetRenderWindow());
  viewer->addCoordinateSystem(1.0);
  viewer->setBackgroundColor(0, 0, 0, 0);

  // Defines point clouds
  if (!readPointClouds()) {
    user_interface = false;
    return;
  }

  // Set values of distances (x, y, z) and rotation (roll, pitch, yaw)
  ui->slider_x->setValue(x * NORMALIZE_VALUE);
  ui->slider_y->setValue(y * NORMALIZE_VALUE);
  ui->slider_z->setValue(z * NORMALIZE_VALUE);
  ui->slider_roll->setValue(roll * NORMALIZE_VALUE);
  ui->slider_pitch->setValue(pitch * NORMALIZE_VALUE);
  ui->slider_yaw->setValue(yaw * NORMALIZE_VALUE);

  ui->slider_scale->setValue(scale * NORMALIZE_SCALE);

  // Visualization
  viewer->resetCamera();
  ui->pcl_widget->update();
}

PairAlign::~PairAlign() {
  delete ui;
}

bool PairAlign::userInterface() {
  return user_interface;
}

void PairAlign::saveFiles() {
  // Remove .ply extention
  std::string point_cloud_name = eraseLastSubStr(point_cloud_file_name, ".ply");

  // Save ply with transformed points
  pcl::io::savePLYFileBinary(point_cloud_name + "_rotated.ply", *transformed_point_cloud);

  // Save aligned mesh
  if (b_accumulated_file) {
    PointC::Ptr accumulated(new PointC);

    pcl::copyPointCloud(*transformed_point_cloud, *accumulated);
    *accumulated = *accumulated + *point_cloud_ref;

    pcl::io::savePLYFileBinary(accumulated_file_name, *accumulated);
  }

  // Quit
  if (userInterface()) {
    QApplication::quit();
  }
}

void PairAlign::xSliderValueChanged(int value) {
  this->x = value / NORMALIZE_VALUE;
  ui->number_x->display(QString::number(this->x, 'f', DECIMAL_PLACES));
  updateView();
}

void PairAlign::ySliderValueChanged(int value) {
  this->y = value / NORMALIZE_VALUE;
  ui->number_y->display(QString::number(this->y, 'f', DECIMAL_PLACES));
  updateView();
}

void PairAlign::zSliderValueChanged(int value) {
  this->z = value / NORMALIZE_VALUE;
  ui->number_z->display(QString::number(this->z, 'f', DECIMAL_PLACES));
  updateView();
}

void PairAlign::rollSliderValueChanged(int value) {
  this->roll = value / NORMALIZE_VALUE;
  ui->number_roll->display(QString::number(this->roll, 'f', DECIMAL_PLACES));
  updateView();
}

void PairAlign::pitchSliderValueChanged(int value) {
  this->pitch = value / NORMALIZE_VALUE;
  ui->number_pitch->display(QString::number(this->pitch, 'f', DECIMAL_PLACES));
  updateView();
}

void PairAlign::yawSliderValueChanged(int value) {
  this->yaw = value / NORMALIZE_VALUE;
  ui->number_yaw->display(QString::number(this->yaw, 'f', DECIMAL_PLACES));
  updateView();
}

void PairAlign::scaleSliderValueChanged(int value) {
  this->scale = value / NORMALIZE_SCALE;
  ui->number_scale->display(QString::number(this->scale, 'f', SCALE_DECIMAL_PLACES));
  updateView();
}

bool PairAlign::parserProgramOptions(int argc, char *argv[]) {
  try {
    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("gui,g", "Show a user interface to the alignment")
    ("save_accumulated,s", po::value<std::string>(&accumulated_file_name), "Saves the accumulated mesh in a .ply file")
    ("point_cloud,c", po::value<std::string>(&this->point_cloud_file_name)->required(), "Point cloud input file (.ply)")
    ("ref_point_cloud,f", po::value<std::string>(&this->point_cloud_ref_file_name)->required(), "Point cloud reference file (.ply)")
    ("x", po::value<double>(&this->x)->default_value(0.0), "Translation in X")
    ("y", po::value<double>(&this->y)->default_value(0.0), "Translation in Y")
    ("z", po::value<double>(&this->z)->default_value(0.0), "Translation in Z")
    ("roll", po::value<double>(&this->roll)->default_value(0.0), "Rotation in X (Roll)")
    ("pitch", po::value<double>(&this->pitch)->default_value(0.0), "Rotation in Y (Pitch)")
    ("yaw", po::value<double>(&this->yaw)->default_value(0.0), "Rotation in Z (Yaw)")
    ("scale", po::value<double>(&this->scale)->default_value(1.0), "Scales the point cloud");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << "Rotation tool to orient the views with regard to one reference point cloud." << std::endl << std::endl;
      std::cout << desc << std::endl;
      return false;
    }

    if (vm.count("point_cloud") && vm.count("ref_point_cloud")) {
      this->point_cloud_file_name = vm["point_cloud"].as<std::string>();
      this->point_cloud_ref_file_name = vm["ref_point_cloud"].as<std::string>();
    } else {
      std::cout << "Correct mode of use: " << argv[0] << " -c pc_input.ply -f pc_ref.ply [opts]" << std::endl;
      return false;
    }

    this->user_interface = vm.count("gui");

    this->b_accumulated_file = vm.count("save_accumulated");
    if (this->b_accumulated_file) {
      this->accumulated_file_name = vm["save_accumulated"].as<std::string>();
    }

    this->x = vm["x"].as<double>();
    this->y = vm["y"].as<double>();
    this->z = vm["z"].as<double>();
    this->roll = vm["roll"].as<double>();
    this->pitch = vm["pitch"].as<double>();
    this->yaw = vm["yaw"].as<double>();
    this->scale = vm["scale"].as<double>();

    return true;
  } catch (boost::program_options::error &msg) {
    std::cout << "ERROR: " << msg.what() << std::endl;
  } catch (...) {
    std::cout << "Some error has occurred." << std::endl;
  }
  return false;
}

void PairAlign::performOperationWithoutGui() {
  if (readPointClouds()) {
    transformPointCloud();
    saveFiles();
  }
}

bool PairAlign::readPointClouds() {
  // Alocate point clouds
  point_cloud.reset(new PointC);
  point_cloud_ref.reset(new PointC);
  transformed_point_cloud.reset(new PointC);

  // Ref point cloud
  if (pcl::io::loadPLYFile<PointT>(point_cloud_ref_file_name, *point_cloud_ref) != -1) {
    std::cout << "MESH " << point_cloud_ref_file_name << " LOADED ALRIGHT!" << std::endl;
    // Cleaning
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*point_cloud_ref, *point_cloud_ref, indices);

    if (userInterface()) {
      addPointCloudToViewer(point_cloud_ref, point_cloud_ref_file_name, false);
      addPointCloudToViewer(transformed_point_cloud, point_cloud_file_name, true);
    }
  } else {
    std::cout << "Couldn't read input ref file" << std::endl;
    return false;
  }

  // Point cloud to tarnslate
  if (pcl::io::loadPLYFile<PointT>(point_cloud_file_name, *point_cloud) != -1) {
    std::cout << "MESH " << point_cloud_file_name << " LOADED ALRIGHT!" << std::endl;
    // Cleaning
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*point_cloud, *point_cloud, indices);
    // Copy
    pcl::copyPointCloud(*point_cloud, *transformed_point_cloud);

    if (userInterface()) {
      addPointCloudToViewer(transformed_point_cloud, point_cloud_file_name, true);
    }
  } else {
    std::cout << "Couldn't read input file" << std::endl;
    return false;
  }

  return true;
}

void PairAlign::updateView() {
  transformPointCloud();
  PointCloudColorHandler color_hander(transformed_point_cloud, 0, 255, 0);
  viewer->updatePointCloud(transformed_point_cloud, color_hander, point_cloud_file_name);
  ui->pcl_widget->update();
}

void PairAlign::transformPointCloud() {
  // Alignment according to transformation values
  // Translate
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << this->x, this->y, this->z;
  pcl::transformPointCloud(*point_cloud, *transformed_point_cloud, transform);
  // Rotate
  transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf((this->roll * M_PI) / 180, Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf((this->pitch * M_PI) / 180, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf((this->yaw * M_PI) / 180, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*transformed_point_cloud, *transformed_point_cloud, transform);
  // Scale
  transform = Eigen::Affine3f::Identity();
  transform(0, 0) = transform(1, 1) = transform(2, 2) = this->scale;
  pcl::transformPointCloud(*transformed_point_cloud, *transformed_point_cloud, transform);
}

void PairAlign::addPointCloudToViewer(PointC::Ptr &pc, const std::string point_cloud_name, const bool is_tranformed) {
  PointCloudColorHandler color_hander(pc, 255 * !is_tranformed, 255 * is_tranformed, 0);
  viewer->addPointCloud(pc, color_hander, point_cloud_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, point_cloud_name);
}

std::string PairAlign::eraseLastSubStr(std::string str, const std::string to_erase) {
  // Search for the last substring in string
  size_t pos = str.rfind(to_erase);

  // If found then erase it from string
  if (pos != std::string::npos) {
    str.erase(pos, to_erase.length());
  }

  return str;
}
