#include "ros/ros.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/Circle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "Eigen/Dense"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <boost/units/systems/si.hpp>
#include <torque_control/torque_trajectoryAction.h>
#include <torque_control/step.h>
#include <actionlib/client/simple_action_client.h>
#include "rpg_youbot_common/rpg_youbot_common.h"
using namespace std;


ros::NodeHandle nh;
ros::Rate lr(50);
  ros::ServiceClient cs2cs_client = nh.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
  ros::Publisher arm_pub_pos = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",
                                                                            2);
  actionlib::SimpleActionClient<torque_control::torque_trajectoryAction> ac("torque_control", true);

