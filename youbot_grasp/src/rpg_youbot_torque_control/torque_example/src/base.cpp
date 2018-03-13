
// Simple demo program that calls the youBot ROS wrapper
//

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include<iostream>

#include<math.h>
using namespace std;
ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

/*
// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}

void moveGripper() {
	brics_actuator::JointPositions msg;
	
	// open gripper
	msg = createGripperPositionCommand(0.011);
	gripperPublisher.publish(msg);

	ros::Duration(3).sleep();

	// close gripper
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
}
*/




// move platform a little bit back- and forward and to the left and right
void movePlatform(float a,float b) {
	geometry_msgs::Twist twist;

	// forward
	twist.linear.x = 0.05;  // with 0.05 m per sec
	twist.linear.y =0;
	platformPublisher.publish(twist);
	ros::Duration(a/abs(twist.linear.x )).sleep();

	cout<<a/abs(twist.linear.x )<<endl;

	// to the left
	twist.linear.x = 0;
	twist.linear.y =0.05;
	platformPublisher.publish(twist);
	ros::Duration(b/abs(twist.linear.y)).sleep();
	// stop
        twist.linear.x = 0;
	twist.linear.y = 0;
	platformPublisher.publish(twist);
}




int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::NodeHandle n;
	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	sleep(1);
		
	
       float a=0.30,b=0.50;
 	movePlatform(a,b);
                        



	sleep(1);
	
	ros::shutdown();
	return 0;
}
