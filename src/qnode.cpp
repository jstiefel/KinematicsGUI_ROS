//============================================================================
// Name        : qnode.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 21.06.2018
// Copyright   : BSD 3-Clause
// Description : ROS communication node
//============================================================================

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "../include/kinematicsgui/qnode.hpp"
#include <string>

namespace kinematicsgui {

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
  {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"kinematicsgui");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;
  // Add your ros communications here.
  log_subscriber = nh.subscribe("/rosout_agg", 5, &QNode::logTopicCallback, this);
  global_state_subscriber = nh.subscribe("/kinematics_controller/global_state_topic", 1, &QNode::stateTopicCallback, this);
  global_homing_client = nh.serviceClient<std_srvs::Trigger>("/kinematics_controller/global_homing_service");
  joint_pos_client = nh.serviceClient<kinematics_controller::joint_pos_service>("/kinematics_controller/joint_pos_service");
  ee_pos_client = nh.serviceClient<kinematics_controller::ee_pos_service>("/kinematics_controller/ee_pos_service");
  injection_client = nh.serviceClient<std_srvs::Trigger>("/kinematics_controller/injection_service");

//  sleep(15); //wait 7 seconds before starting that whole log is not shown at startup
  start(); //start a Qthread, which calls run()
  std::cout << "Successfully initialized node." << std::endl;
  return true;
}

void QNode::run() {
  //QThread function
  ros::Rate loop_rate(1); //too fast loop rate crashes the GUI
  std::cout << "Node is running." << std::endl;
  while ( ros::ok() ) {

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool QNode::homing(){
  if (global_homing_client.call(homing_client)){
    ROS_INFO_STREAM("global homing client successfully called. Success: " << (int)homing_client.response.success);
  } else {
    ROS_ERROR("Failed to call service global_homing_client");
    return false;
  }
  return true;
}

bool QNode::ee_target_pos(float ee_y, float ee_x, float ee_z,  float ee_phi, float ee_theta){
  ee_client.request.x_ee = ee_x;
  ee_client.request.y_ee = ee_y;
  ee_client.request.z_ee = ee_z;
  ee_client.request.phi_ee = ee_phi;
  ee_client.request.theta_ee = ee_theta;

  if (ee_pos_client.call(ee_client)){
    ROS_INFO_STREAM("ee pos client service successfully called. Success: " << (int)ee_client.response.success);
  } else {
    ROS_ERROR("Failed to call service ee_pos_client");
    return false;
  }
  return true;
}

bool QNode::joint_target_pos(float q_y, float q_x, float q_alpha, float q_beta, float q_d){
  joint_client.request.y_joint = q_y;
  joint_client.request.x_joint = q_x;
  joint_client.request.alpha_joint = q_alpha;
  joint_client.request.beta_joint = q_beta;
  joint_client.request.d_joint = q_d;

  if (joint_pos_client.call(joint_client)){
    ROS_INFO_STREAM("joint pos client service successfully called. Success: " << (int)joint_client.response.success);
  } else {
    ROS_ERROR("Failed to call service joint_pos_client");
    return false;
  }
  return true;
}

bool QNode::make_injection(){
  if (injection_client.call(injection)){
    ROS_INFO_STREAM("injection client successfully called. Success: " << (int)injection.response.success);
  } else {
    ROS_ERROR("Failed to call service injection_client");
    return false;
  }
  return true;
}

void QNode::stateTopicCallback(const kinematics_controller::global_state_info::ConstPtr& state){
    x_value = (float)state->x_joint;
    y_value = (float)state->y_joint;
    alpha_value = (float)state->alpha_joint;
    beta_value = (float)state->beta_joint;
    d_value = (float)state->d_joint;
    x_ee_value = (float)state->x_ee;
    y_ee_value = (float)state->y_ee;
    z_ee_value = (float)state->z_ee;

    if(state->homing_completed){
      homing_completed_status = 1;
    }
    else {
      homing_completed_status = 0;
    }
    if(state->last_position_reached){
      last_position_reached_status = 1;
    }
    else {
      last_position_reached_status = 0;
    }

    Q_EMIT stateUpdated(); //to set the number new
}

//Logging:
void QNode::logTopicCallback(const rosgraph_msgs::Log::ConstPtr& log){
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  int level = log->level;
  switch (level){
    case(1) : {
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(2) : {
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(4) : {
        logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(8) : {
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(16) : {
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


}  // namespace kinematicsgui
