
#include "ros/ros.h"
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_tester");
  ros::NodeHandle nh;
  ros::Rate lr(500);
  ros::ServiceClient circle_client = nh.serviceClient<trajectory_generator::Circle>("Circular_Trajectory");
  ros::Publisher arm_pub_pos = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",
                                                                            1);
  actionlib::SimpleActionClient<torque_control::torque_trajectoryAction> ac("torque_control", true);

  double first_pt[5];
  bool feasible = false;
  trajectory_msgs::JointTrajectory traj, temp;
  trajectory_msgs::JointTrajectoryPoint point;

  trajectory_generator::Circle circle;
/*  
  circle.request.radius = 0.1;//半径
  circle.request.omega = 2 * circle.request.radius * M_PI;
  circle.request.rpy[0] = M_PI / 2;
  circle.request.rpy[1] = 0;
  circle.request.rpy[2] = 0;
  circle.request.center[0] = 0.35;
  circle.request.center[1] = 0;
  circle.request.center[2] = 0.3;
  circle.request.center[0] = 0.35;
  circle.request.center[1] = 0;
  circle.request.center[2] = 0.3;
*/
circle.request.radius = 0.1;
  circle.request.omega = 2 * circle.request.radius * M_PI;
/*
  circle.request.rpy[0] = M_PI/2;
   circle.request.rpy[1] = 0;
   circle.request.rpy[2] = 0;
   circle.request.center[0] = 0.35;
   circle.request.center[1] = 0;
   circle.request.center[2] = 0.3;
*/
  circle.request.rpy[0] = M_PI / 2;
  circle.request.rpy[1] = -M_PI / 4;
  circle.request.rpy[2] = M_PI / 10;
  circle.request.center[0] = 0.35;
  circle.request.center[1] = 0.0;
  circle.request.center[2] = 0.5;


 if (circle_client.call(circle))
  {
    if (circle.response.feasible)
    {
      feasible = true;
      traj = circle.response.trajectory;
    }
    else
    {
      cout << "CS 2 CS not feasible" << endl;
    }

  }
  else
  {
    ROS_ERROR("Could not call service.");
    return 1;
  }

  //torque control

  if (feasible)
  {
    point = traj.points.back();
    int i = 0;
    while (!point.positions.empty())
    {
      first_pt[i] = point.positions.back();
      i++;
      point.positions.pop_back();
    }
   cout << "Publishing arm cmd" << endl;
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    lr.sleep();
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    lr.sleep();
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    lr.sleep();
    sleep(1);

if (1)
  {
    point = traj.points.back();
    int i = 0;
    while (!point.positions.empty())
    {
      first_pt[i] = point.positions.back();
      i++;
      point.positions.pop_back();
    }
    cout << "Publishing arm cmd" << endl;
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
   
    
    //ROS_INFO("READY FOR TORQUE?");

  }

  /*  //ROS_INFO("READY FOR TORQUE?");
   // int x;
   // cin >> x;
   // ac.waitForServer(); //will wait for infinite time
   // ROS_INFO("Action server started, sending goal.");

    torque_control::torque_trajectoryGoal goal;
    goal.trajectory = traj;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
	cout << "Publishing arm cmd" << endl;
    //arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
*/
  }

  return 0;
}

