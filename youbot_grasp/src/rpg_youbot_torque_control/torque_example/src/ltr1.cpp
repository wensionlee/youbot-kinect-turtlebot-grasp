/*
// along with RPG-YTC.  If not, see <http://www.gnu.org/licenses/>.

 * ltr.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: keiserb
 */
#include <string.h>  
#include <stdlib.h>  
#include <unistd.h>  
#include <arpa/inet.h>  
#include <sys/socket.h>
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

int ik(float x,float y,float z,float a,float b,float c,float d)

{
cout<<"twst3"<<endl;

  ros::NodeHandle nh;
  ros::Rate lr(10);
  ros::ServiceClient cs2cs_client = nh.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
  ros::Publisher arm_pub_pos = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",
                                                                            2);
  actionlib::SimpleActionClient<torque_control::torque_trajectoryAction> ac("torque_control", true);

  double max_vel = 0.05;
  double max_acc = 0.5;
  ROS_INFO("max_vel: %f \t max_acc :%f", max_vel, max_acc);
  double first_pt[5];
  bool feasible = false;
  trajectory_msgs::JointTrajectory traj, temp;
  trajectory_msgs::JointTrajectoryPoint point;
  geometry_msgs::Pose start_p, low_p1, low_p2, end_p;
  trajectory_generator::CStoCS cs2cs;
  start_p.position.x = x;
  start_p.position.y = y;
  start_p.position.z = z;
  Eigen::Quaterniond grip(a, b, c, d);
  start_p.orientation.x = grip.x();
  start_p.orientation.y = grip.y();
  start_p.orientation.z = grip.z();
  start_p.orientation.w = grip.w();
  low_p1.position.x = 0;
  low_p1.position.y = 0.23;
  low_p1.position.z = 0;
  low_p1.orientation = start_p.orientation;
  low_p2.position.x = -0.05;
  low_p2.position.y = 0.25;
  low_p2.position.z = -0.1;
  low_p2.orientation = start_p.orientation;
  end_p.position.x = -0.12;
  end_p.position.y = 0.25;
  end_p.position.z = 0.00;
  end_p.orientation = start_p.orientation;

  cs2cs.request.start_pos = start_p;
  cs2cs.request.end_pos = low_p1;
  cs2cs.request.start_vel = 0.0;
  cs2cs.request.end_vel = max_vel;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;


cout<<"test"<<endl;
  if (cs2cs_client.call(cs2cs))
  {
    if (cs2cs.response.feasible)
    {
      cout << "feasible" << endl;
      temp = cs2cs.response.trajectory;
      while (!temp.points.empty())
      {
        point = temp.points.back();
        temp.points.pop_back();
        traj.points.insert(traj.points.begin(), point);
      }
      traj.joint_names = temp.joint_names;
    }
    else
    {
      cout << "First Half Not Feasible" << endl;
      feasible = false;
    }
  }
  else
  {
    ROS_ERROR("Could not call service.");
    return 1;
  }

  cs2cs.request.start_pos = low_p1;
  cs2cs.request.end_pos = low_p2;
  cs2cs.request.start_vel = max_vel;
  cs2cs.request.end_vel = max_vel;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;

  if (cs2cs_client.call(cs2cs))
  {
    if (cs2cs.response.feasible)
    {
      cout << "feasible" << endl;
      temp = cs2cs.response.trajectory;
      while (!temp.points.empty())
      {
        point = temp.points.back();
        temp.points.pop_back();
        traj.points.insert(traj.points.begin(), point);
      }
      traj.joint_names = temp.joint_names;
    }
    else
    {
      cout << "First Half Not Feasible" << endl;
      feasible = false;
    }
  }
  else
  {
    ROS_ERROR("Could not call service.");
    return 1;
  }

  cs2cs.request.start_pos = low_p2;
  cs2cs.request.end_pos = end_p;
  cs2cs.request.start_vel = max_vel;
  cs2cs.request.end_vel = 0.0;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;

  if (cs2cs_client.call(cs2cs))
  {
    if (cs2cs.response.feasible)
    {
      cout << "feasible" << endl;
      temp = cs2cs.response.trajectory;
      while (!temp.points.empty())
      {
        point = temp.points.back();
        temp.points.pop_back();
        traj.points.insert(traj.points.begin(), point);
      }
      traj.joint_names = temp.joint_names;
    }
    else
    {
      cout << "Second Half Not Feasible" << endl;
      feasible = false;
    }
  }
  else
  {
    ROS_ERROR("Could not call service.");
    return 1;
  }

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
   
    
    ROS_INFO("READY FOR TORQUE?");
 
   
   

  }
}

int gripper(float x,float y)
{	ros::NodeHandle n;
	
	ros::Publisher gripperPositionPublisher;

	
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(20); //Hz
	
	
	static const int numberOfGripperJoints = 2;

	 
	brics_actuator::JointPositions command;
	
	vector <brics_actuator::JointValue> gripperJointPositions;

	
	gripperJointPositions.resize(numberOfGripperJoints);

	gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
	gripperJointPositions[0].value = x;
	gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

	cout << "Please type in value for a right jaw of the gripper " << endl;
	
	gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
	gripperJointPositions[1].value = y;
	gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

	cout << "sending command ..." << endl;

	command.positions = gripperJointPositions;
	
for(int i=0;i<3;i++)
{
    gripperPositionPublisher.publish(command);
    sleep(1);
}	
       

	cout << "--------------------" << endl;
	ros::spinOnce();
	rate.sleep();


	return 0;

}






int main(int argc, char **argv)
{  
   
ros::init(argc, argv, "traj_tester");
//float a=0.313651;
//float b=0.309381;
//float *str=socket();
//float a=str[0]*0.001;
//float b=str[1]*0.001;
float a=0.259164;
float b=0.277459;
cout<<a<<endl;
cout<<b<<endl;


//ik(0.260,0.061,-0.01, 0.707106781187,0, 0.707106781187, 0);//graps right


 gripper(0.01,0.01);
 
ik(a-0.248,b-0.061,0.01,0.6851, 0.1749, 0.6851, -0.1749);
 


 //ik(a-0.248,b-0.061,-0.01,0.6851, 0.1749, 0.6851, -0.1749); //x-0.26 y+0.061
 
ik(a-0.248,b-0.061,-0.07,0.6851, 0.1749, 0.6851, -0.1749);
 cout<<"test1"<<endl;

gripper(0.001,0.001);

 ik(a-0.248,b-0.061,-0.02,0.6851, 0.1749, 0.6851, -0.1749);
 //cout<<"test2"<<endl;


ik(0.26,0.061,-0.01,0.6851, 0.1749, 0.6851, -0.1749);
ik(0.26,0.061,-0.07,0.6851, 0.1749, 0.6851, -0.1749);
 gripper(0.01,0.01);

ik(0.26,0.061,-0.01,0.6851, 0.1749, 0.6851, -0.1749);



 
  return 0;
}

