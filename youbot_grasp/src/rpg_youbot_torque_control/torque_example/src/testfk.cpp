#include <iostream>
#include <assert.h>


#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>





#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>


#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include "trajectory_msgs/JointTrajectoryPoint.h"

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>

#include <boost/units/systems/si.hpp>


#include <actionlib/client/simple_action_client.h>


using namespace std;

float fk(float a,float b,float c,float d,float e){
         
        
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;


	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfArmJoints = 5;
	

/*

		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfArmJoints; ++i) {
			jointName.str("");
		
			armJointPositions[i].joint_uri = jointName.str();
		}
			armJointPositions[0].value =a;
			armJointPositions[0].unit = boost::units::to_string(boost::units::si::radians);
			
			armJointPositions[1].value =b;
			armJointPositions[1].unit = boost::units::to_string(boost::units::si::radians);
			
			armJointPositions[2].value =c;
			armJointPositions[2].unit = boost::units::to_string(boost::units::si::radians);
			
			armJointPositions[3].value =d;
			armJointPositions[3].unit = boost::units::to_string(boost::units::si::radians);
			
			armJointPositions[4].value =e;
			armJointPositions[4].unit = boost::units::to_string(boost::units::si::radians);
			
			cout<<"test"<<endl;
			//armJointPositions[0].value = a;
			//armJointPositions[1].value = b;
			//armJointPositions[2].value = c;
			//armJointPositions[3].value = d;
			//armJointPositions[4].value = e;
	

*/


for(int j=0;j<10;j++){
	
		brics_actuator::JointPositions command;
		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

		armJointPositions.resize(numberOfArmJoints); //TODO:change that
		
		std::stringstream jointName;
	//for (int i = 0;  i< 1; ++i){
			jointName.str("");
			jointName << "arm_joint_" << (1);
			armJointPositions[0].joint_uri = jointName.str();
			armJointPositions[0].value = a;
			armJointPositions[0].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[0].joint_uri << " = " << armJointPositions[0].value << " " << armJointPositions[0].unit << endl;
//};
			jointName.str("");
			jointName << "arm_joint_" << (2);
armJointPositions[1].joint_uri = jointName.str();
			armJointPositions[1].value = b;
			armJointPositions[1].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[1].joint_uri << " = " << armJointPositions[1].value << " " << armJointPositions[1].unit << endl;
			jointName.str("");
			jointName << "arm_joint_" << (3);
armJointPositions[2].joint_uri = jointName.str();
			armJointPositions[2].value = c;
			armJointPositions[2].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[2].joint_uri << " = " << armJointPositions[2].value << " " << armJointPositions[2].unit << endl;
			jointName.str("");
			jointName << "arm_joint_" << (4);
armJointPositions[3].joint_uri = jointName.str();
			armJointPositions[3].value = d;
			armJointPositions[3].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[3].joint_uri << " = " << armJointPositions[3].value << " " << armJointPositions[3].unit << endl;
			jointName.str("");
			jointName << "arm_joint_" << (5);
armJointPositions[4].joint_uri = jointName.str();
			armJointPositions[4].value =e;
			armJointPositions[4].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[4].joint_uri << " = " << armJointPositions[4].value << " " << armJointPositions[4].unit << endl;
		



		cout << "sending command ..." << endl;

		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);



		cout << "--------------------" << endl;
		ros::spinOnce();
		rate.sleep();
                cout<<"a="<<a<<endl;
		cout<<"b="<<b<<endl;
                cout<<armJointPositions[0];
		cout<<armJointPositions[1];
	}
	

	return 0;
}	


int main(int argc, char **argv) {

	ros::init(argc, argv, "fk");
	
	//fk(1.2,0.8, -3.38, 1.3, 2.9);
	fk(1.2,1.3, -2.7, 1.6, 2.8);
	//fk(4.5,0.8, -3.3, 1.3, 2.9);
	return 0;
}


