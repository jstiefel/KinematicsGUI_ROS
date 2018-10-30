//============================================================================
// Name        : mainwindow.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 21.06.2018
// Copyright   : BSD 3-Clause
// Description : Qt based GUI
//============================================================================

#ifndef kinematicsgui_MAINWINDOW_H
#define kinematicsgui_MAINWINDOW_H

#include <QtGui/QMainWindow>
#include "ui_mainwindow.h"
#include "qnode.hpp"

namespace kinematicsgui {

//Qt central, all operations relating to the view part here.

class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
  void on_actionAbout_triggered();

  void on_actionManual_triggered();

  void on_actionQuit_triggered();

  void updateLoggingView(); //manual connection

  void updateStateView(); //manual connection

private Q_SLOTS:
  void on_ee_homing_clicked();

  void on_q_homing_clicked();

  void on_ee_start_clicked();

  void on_q_start_clicked();

  void on_q_injection_clicked();

  void on_ee_injection_clicked();

  void on_copy_to_control_clicked();

private:
  Ui::MainWindowDesign ui; //can pick any element in our .ui file with this
  QNode qnode; //create ROS node
};

}  // namespace kinematicsgui

#endif // kinematicsgui_MAINWINDOW_H
