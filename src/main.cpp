//============================================================================
// Name        : main.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 21.06.2018
// Copyright   : BSD 3-Clause
// Description : Qt based GUI.
//============================================================================
#include <QtGui>
#include <QApplication>
#include "../include/kinematicsgui/mainwindow.hpp"

int main(int argc, char **argv) {

    //Starting Qt
    QApplication app(argc, argv);
    kinematicsgui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
