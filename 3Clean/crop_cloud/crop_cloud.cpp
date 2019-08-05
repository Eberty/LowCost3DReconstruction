/*
 * Copyright (c) 2019, Eberty Alves
 */

// Point cloud library
#include <pcl/filters/crop_box.h>
#include <pcl/io/ply_io.h>

// Boost library
#include <boost/program_options.hpp>

// Qt
#include <QApplication>

// Project files
#include "crop_cloud.h"

#define NORMALIZE_VALUE 100.0
#define DECIMAL_PLACES 2

CropCloud::CropCloud(int argc, char *argv[], QWidget *parent) : QMainWindow(parent), ui(new Ui::CropCloud) {
  ui->setupUi(this);
  this->setWindowTitle("Remove points outside box");

  // Verify if everything is ok
  bool ok = parserProgramOptions(argc, argv);
  if (!ok) {
    user_interface = false;
    return;
  }

  // Just save files with defined values and not show the GUI
  if (ok && !user_interface) {
    performOperationWithoutGui();
    return;
  }

  // Connect sliders, buttons and their functions
  connect(ui->save_button, SIGNAL(clicked()), this, SLOT(saveFiles()));
  connect(ui->slider_min_x, SIGNAL(valueChanged(int)), this, SLOT(minXSliderValueChanged(int)));
  connect(ui->slider_min_y, SIGNAL(valueChanged(int)), this, SLOT(minYSliderValueChanged(int)));
  connect(ui->slider_min_z, SIGNAL(valueChanged(int)), this, SLOT(minZSliderValueChanged(int)));
  connect(ui->slider_max_x, SIGNAL(valueChanged(int)), this, SLOT(maxXSliderValueChanged(int)));
  connect(ui->slider_max_y, SIGNAL(valueChanged(int)), this, SLOT(maxYSliderValueChanged(int)));
  connect(ui->slider_max_z, SIGNAL(valueChanged(int)), this, SLOT(maxZSliderValueChanged(int)));

  // Set up the QVTK window for point cloud visualization
  viewer.reset(new pcl::visualization::PCLVisualizer("PCL Visualizer", false));
  ui->pcl_widget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->pcl_widget->GetInteractor(), ui->pcl_widget->GetRenderWindow());

  // Defines point clouds
  readPointCloud();

  // Set values for crop box
  ui->slider_min_x->setValue(min_x * NORMALIZE_VALUE);
  ui->slider_min_y->setValue(min_y * NORMALIZE_VALUE);
  ui->slider_min_z->setValue(min_z * NORMALIZE_VALUE);
  ui->slider_max_x->setValue(max_x * NORMALIZE_VALUE);
  ui->slider_max_y->setValue(max_y * NORMALIZE_VALUE);
  ui->slider_max_z->setValue(max_z * NORMALIZE_VALUE);

  // Visualization
  viewer->resetCamera();
  ui->pcl_widget->update();
}

CropCloud::~CropCloud() {
  delete ui;
}

bool CropCloud::userInterface() {
  return user_interface;
}

void CropCloud::saveFiles() {
  // Remove .ply extention
  std::string point_cloud_name = eraseLastSubStr(point_cloud_file, ".ply");

  // Points that will be filtered
  PointC::Ptr cloud_filtered(new PointC);

  // CropBox is a filter that allows the user to filter all the data inside of a given box
  pcl::CropBox<PointT> crop_box;
  crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
  crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
  crop_box.setInputCloud(point_cloud);
  crop_box.filter(*cloud_filtered);

  // Save ply with points croped
  pcl::io::savePLYFileBinary(point_cloud_name + "_croped.ply", *cloud_filtered);

  // Save points outside the crop box
  if (removed_points_file) {
    crop_box.setNegative(true);
    crop_box.filter(*cloud_filtered);
    pcl::io::savePLYFileBinary(point_cloud_name + "_unused_points.ply", *cloud_filtered);
  }
  
  // Quit
  if (userInterface()) {
    QApplication::quit();
  }
}

void CropCloud::minXSliderValueChanged(int value) {
  if ((value / NORMALIZE_VALUE) <= this->max_x) {
    this->min_x = value / NORMALIZE_VALUE;
    ui->number_min_x->display(QString::number(this->min_x, 'f', DECIMAL_PLACES));
    updateView();
  } else {
    ui->slider_min_x->setValue(this->max_x * NORMALIZE_VALUE);
  }
}

void CropCloud::minYSliderValueChanged(int value) {
  if ((value / NORMALIZE_VALUE) <= this->max_y) {
    this->min_y = value / NORMALIZE_VALUE;
    ui->number_min_y->display(QString::number(this->min_y, 'f', DECIMAL_PLACES));
    updateView();
  } else {
    ui->slider_min_y->setValue(this->max_y * NORMALIZE_VALUE);
  }
}

void CropCloud::minZSliderValueChanged(int value) {
  if ((value / NORMALIZE_VALUE) <= this->max_z) {
    this->min_z = value / NORMALIZE_VALUE;
    ui->number_min_z->display(QString::number(this->min_z, 'f', DECIMAL_PLACES));
    updateView();
  } else {
    ui->slider_min_z->setValue(this->max_z * NORMALIZE_VALUE);
  }
}

void CropCloud::maxXSliderValueChanged(int value) {
  if ((value / NORMALIZE_VALUE) >= this->min_x) {
    this->max_x = value / NORMALIZE_VALUE;
    ui->number_max_x->display(QString::number(this->max_x, 'f', DECIMAL_PLACES));
    updateView();
  } else {
    ui->slider_max_x->setValue(this->min_x * NORMALIZE_VALUE);
  }
}

void CropCloud::maxYSliderValueChanged(int value) {
  if ((value / NORMALIZE_VALUE) >= this->min_y) {
    this->max_y = value / NORMALIZE_VALUE;
    ui->number_max_y->display(QString::number(this->max_y, 'f', DECIMAL_PLACES));
    updateView();
  } else {
    ui->slider_max_y->setValue(this->min_y * NORMALIZE_VALUE);
  }
}

void CropCloud::maxZSliderValueChanged(int value) {
  if ((value / NORMALIZE_VALUE) >= this->min_z) {
    this->max_z = value / NORMALIZE_VALUE;
    ui->number_max_z->display(QString::number(this->max_z, 'f', DECIMAL_PLACES));
    updateView();
  } else {
    ui->slider_max_z->setValue(this->min_z * NORMALIZE_VALUE);
  }
}

bool CropCloud::parserProgramOptions(int argc, char *argv[]) {
  try {
    // Parse command-line options
    namespace po = boost::program_options;

    // Define command-line options
    po::options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help message")
    ("input,i", po::value<std::string>(&this->point_cloud_file)->required(), "Point cloud input file (.ply)")
    ("gui,g", "Show a user interface to the alignment")
    ("save_removed,s", "Saves the removed points in a .ply file")
    ("min_x,x", po::value<double>(&this->min_x)->default_value(-1.0), "The min X coordinate to crop")
    ("max_x,X", po::value<double>(&this->max_x)->default_value( 1.0), "The max X coordinate to crop")
    ("min_y,y", po::value<double>(&this->min_y)->default_value(-1.0), "The min Y coordinate to crop")
    ("max_y,Y", po::value<double>(&this->max_y)->default_value( 1.0), "The max Y coordinate to crop")
    ("min_z,z", po::value<double>(&this->min_z)->default_value(-1.0), "The min Z coordinate to crop")
    ("max_z,Z", po::value<double>(&this->max_z)->default_value( 1.0), "The max Z coordinate to crop");

    // Use a parser to evaluate the command line
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // Store the command-line options evaluated by the parser
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return false;
    }

    if (vm.count("input")) {
      this->point_cloud_file = vm["input"].as<std::string>();
    } else {
      std::cout << "Correct mode of use: " << argv[0] << " -i input.ply [opts]" << std::endl;
      return false;
    }

    this->user_interface = vm.count("gui");
    this->removed_points_file = vm.count("save_removed");
    this->min_x = vm["min_x"].as<double>();
    this->min_y = vm["min_y"].as<double>();
    this->min_z = vm["min_z"].as<double>();
    this->max_x = vm["max_x"].as<double>();
    this->max_y = vm["max_y"].as<double>();
    this->max_z = vm["max_z"].as<double>();

    return true;
  } catch (boost::program_options::error &msg) {
    std::cerr << "ERROR: " << msg.what() << std::endl;
  } catch (...) {
    std::cerr << "Some error has occurred." << std::endl;
  }
  return false;
}

void CropCloud::performOperationWithoutGui() {
  readPointCloud();
  saveFiles();
}

void CropCloud::readPointCloud() {
  // Alocate point clouds
  point_cloud.reset(new PointC);
  point_cloud_inside.reset(new PointC);
  point_cloud_outside.reset(new PointC);

  // Load
  if (pcl::io::loadPLYFile<PointT>(point_cloud_file, *point_cloud) != -1) {
    std::cout << "MESH " << point_cloud_file << " LOADED ALRIGHT!" << std::endl;
    // Cleaning
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*point_cloud, *point_cloud, indices);
  } else {
    throw "Couldn't open point cloud file";
  }
}

void CropCloud::updateView() {
  drawCube();

  pcl::CropBox<PointT> crop_box;
  crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
  crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
  crop_box.setInputCloud(point_cloud);
  crop_box.filter(*point_cloud_inside);

  drawPointCloud(point_cloud_inside, "inside", false);

  crop_box.setNegative(true);
  crop_box.filter(*point_cloud_outside);
  drawPointCloud(point_cloud_outside, "outside", true);

  ui->pcl_widget->update();
}

void CropCloud::drawCube() {
  viewer->removeShape("cube");
  viewer->addCube(min_x, max_x, min_y, max_y, min_z, max_z, 1.0, 1.0, 1.0, "cube");
  viewer->setRepresentationToWireframeForAllActors();
}

void CropCloud::drawPointCloud(PointC::Ptr &pc, const std::string point_cloud_name, const bool is_outside) {
  viewer->removePointCloud(point_cloud_name);

  PointCloudColorHandler color_hander(pc, 255*is_outside, 0, 255*!is_outside);
  viewer->addPointCloud(pc, color_hander, point_cloud_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, point_cloud_name);
}

std::string CropCloud::eraseLastSubStr(std::string str, const std::string to_erase) {
  // Search for the last substring in string
  size_t pos = str.rfind(to_erase);

  // If found then erase it from string
  if (pos != std::string::npos) {
    str.erase(pos, to_erase.length());
  }

  return str;
}
