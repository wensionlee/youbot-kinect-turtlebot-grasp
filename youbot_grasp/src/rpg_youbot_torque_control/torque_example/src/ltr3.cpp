/*
// along with RPG-YTC.  If not, see <http://www.gnu.org/licenses/>.

 * ltr.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: wensionlee
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
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "geometry_msgs/Twist.h"


#include<iostream>
#include<math.h>
#include <assert.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include <boost/units/io.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include "control_msgs/FollowJointTrajectoryAction.h"




ros::Publisher platformPublisher;

using namespace std;


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





int ik(float x,float y,float z,float a,float b,float c,float d)

{
cout<<"twst"<<endl;

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
	for(int i=0;i<3;i++){
    gripperPositionPublisher.publish(command);
    sleep(1);
}	
       

	cout << "--------------------" << endl;
	ros::spinOnce();
	rate.sleep();


	return 0;

}



float *socket1()
{
//创建套接字  
    int sock = socket(AF_INET, SOCK_STREAM, 0);  
  
    //向服务器（特定的IP和端口）发起请求  
    struct sockaddr_in serv_addr;  
    memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充  
    serv_addr.sin_family = AF_INET;  //使用IPv4地址  
    serv_addr.sin_addr.s_addr = inet_addr("192.168.1.106");  //具体的IP地址  
    serv_addr.sin_port = htons(1234);  //端口  
    connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));  

    

   
    //读取客户端传回的数据  
    char buffer[3][100]; 
    
    read(sock, buffer[0], sizeof(buffer[0]));  
    read(sock, buffer[1], sizeof(buffer[1]));  
    read(sock, buffer[2], sizeof(buffer[2]));  
    float bu[3];
    bu[0]=atof(buffer[0]);
    bu[1]=atof(buffer[1]);
    bu[2]=atof(buffer[2]);
    //printf("Message form server: %s\n", buffer);  
    printf("%f\n",bu[0]);
    printf("%f\n",bu[1]);
    printf("%f\n",bu[2]);

   return bu;


  printf("==============================\n");
    char str[40];  
    read(sock,str, sizeof(str)-1);  
     
    printf("Message form server: %s\n", str);

/*

   #define BUF_SIZE 100
   char bufSend[BUF_SIZE] = "a";  
     char bufRecv[BUF_SIZE] = {0};
    send(sock, bufSend, strlen(bufSend), 0);  
    //接收服务器传回的数据  
   
    recv(sock, bufRecv, BUF_SIZE, 0);  
  
    //输出接收到的数据  
    printf("Message form server: %s\n", bufRecv);  
 memset(bufSend, 0, BUF_SIZE);  //重置缓冲区  
        memset(bufRecv, 0, BUF_SIZE);  //重置缓冲区 
   
    //关闭套接字  
*/
 close(sock);

}










int main(int argc, char **argv)
{  
   
ros::init(argc, argv, "traj_tester");
/*底盘运动
//float a=0.313651;
//float b=0.309381;
float *str=socket1();
float baseX=str[0]*0.001;
float baseY=str[1]*0.001;

cout<<"basex="<<baseX<<endl;
cout<<"basey="<<baseY<<endl;

//ik(0.260,0.061,-0.01, 0.707106781187,0, 0.707106781187, 0);//graps right
ros::NodeHandle n;
platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
sleep(1);
movePlatform((baseX-0.25),(baseY-0.065-0.25));

sleep(1);
*/ 


//float *str1=socket1();
//float a=str1[0]*0.001;
//float b=str1[1]*0.001;

  //float a=0.259164;
  //float b=0.307459;
float a=0.439164;
float b=0.06;
cout<<"a="<<a<<endl;
cout<<"b="<<b<<endl;
gripper(0.01,0.01);

//ik(a-0.248,b-0.085,0.01,0.6851, 0.1749, 0.6851, -0.1749);
 

ik(a-0.248,b-0.085,-0.01,0.6851, 0.1749, 0.6851, -0.1749); //x-0.26 y+0.061

sleep(1);

ik(a-0.248,b-0.085,-0.08,0.6851, 0.1749, 0.6851, -0.1749);

gripper(0.001,0.001);


sleep(3);

//ik(a-0.248,b-0.085,-0.07,0.6851, 0.1749, 0.6851, -0.1749);
 
	
fk(1.2,0.8, -3.38, 1.3, 2.9);
//fk(4.5,0.8, -3.3, 1.3, 2.9);

sleep(1);
gripper(0.01,0.01);
sleep(3);
fk(1.2,1.3, -2.7, 1.6, 2.8);

return 0;
}

