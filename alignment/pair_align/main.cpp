/*
 * Copyright (c) 2019, Eberty Alves
 */

// Qt library
#include <QApplication>

// Project files
#include "pair_align.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  PairAlign viewer(argc, argv);
  if (viewer.userInterface()) {
    viewer.show();
    return app.exec();
  }
  return 0;
}
