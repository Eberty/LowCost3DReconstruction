/*
 * Copyright (c) 2019, Eberty Alves
 */

// Qt library
#include <QApplication>

// Project files
#include "crop_cloud.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  CropCloud viewer(argc, argv);
  if (viewer.userInterface()) {
    viewer.show();
    return app.exec();
  }
  return 0;
}
