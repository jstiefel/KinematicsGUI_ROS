//============================================================================
// Name        : qnode.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 21.06.2018
// Copyright   : BSD 3-Clause
// Description : ROS communication node
//============================================================================

#ifndef kinematicsgui_QNODE_HPP_
#define kinematicsgui_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "kinematics_controller/global_state_info.h"
#include "kinematics_controller/joint_pos_service.h"
#include "kinematics_controller/ee_pos_service.h"
#include <rosgraph_msgs/Log.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

namespace kinematicsgui {


class QNode : public QThread {
    Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  void run();
  bool homing();
  bool ee_target_pos(float ee_y, float ee_x, float ee_z,  float ee_rho, float ee_theta);
  bool joint_target_pos(float q_y, float q_x, float q_alpha, float q_beta, float q_d);
  bool make_injection();

  void stateTopicCallback(const kinematics_controller::global_state_info::ConstPtr& state);

  // State:
  double x_value = 0.0;
  double y_value = 0.0;
  double alpha_value = 0.0;
  double beta_value = 0.0;
  double d_value = 0.0;
  bool homing_completed_status = 0;
  bool last_position_reached_status = 0;
  double x_ee_value = 0.0;
  double y_ee_value = 0.0;
  double z_ee_value = 0.0;

  // Logging:

  QStringListModel* loggingModel() { return &logging_model; }
  void logTopicCallback(const rosgraph_msgs::Log::ConstPtr& log);


Q_SIGNALS:
    void rosShutdown();
    void loggingUpdated();
    void stateUpdated();

private:
  int init_argc;
  char** init_argv;
  ros::Subscriber log_subscriber;
  ros::Subscriber global_state_subscriber;
  ros::ServiceClient global_homing_client;
  ros::ServiceClient joint_pos_client;
  ros::ServiceClient ee_pos_client;
  ros::ServiceClient injection_client;
  std_srvs::Trigger homing_client;
  std_srvs::Trigger injection;
  kinematics_controller::joint_pos_service joint_client;
  kinematics_controller::ee_pos_service ee_client;

  // Logging:
  QStringListModel logging_model;
};

}  // namespace kinematicsgui

#endif /* kinematicsgui_QNODE_HPP_ */
