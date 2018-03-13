
#include <moveit/move_group_interface/move_group.h>






#include <string.h>
#include "boost/units/io.hpp"	//must---
#include <geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>

#include <std_msgs/Int32.h>
/*             position.x  position.x     position.z    orientation.x   orientation.y   orientation.z    orientation.w
//低点垂直     0.4439      -0.03          0.0847046      0.0166193      0.999848        -0.000555996     0.00524638
  低点垂直     0.413999   -0.036004       0.0928915      0              1                0                0
*/


using namespace std;






void mark_pose_callback()

{
//    ROS_INFO(" comein");
    moveit::planning_interface::MoveGroup group("arm_1");//right_arm   arm_1
    moveit::planning_interface::MoveGroup::Plan my_plan;
//    string reference_frame = "base_link";
//    string eef_link = "end_point_link";
    group.setGoalJointTolerance(0.01);
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.01);
//    group.setPoseReferenceFrame(reference_frame);
//    group.setEndEffectorLink(eef_link);
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    group.setNumPlanningAttempts(5);
    group.allowReplanning(false);
    group.setPlanningTime(60);


    geometry_msgs::Pose target_pose1;
    geometry_msgs::Pose target_pose2;
//    target_pose1 = msg.pose;  //暂存接收到的mark坐标值

  target_pose1.position.x= 0.355864;
  target_pose1.position.y = 0.146411;
  target_pose1.position.z = 0.066093;
  target_pose1.orientation.x= 0;
  target_pose1.orientation.y = 1;
  target_pose1.orientation.z = 0;
  target_pose1.orientation.w = 0;
  target_pose2.position.x= 0.355864;
  target_pose2.position.y = -0.146411;
  target_pose2.position.z = 0.066093;
  target_pose2.orientation.x= 0;
  target_pose2.orientation.y = 1;
  target_pose2.orientation.z = 0;
  target_pose2.orientation.w = 0;


//每次加载初始化当前手臂位置
    std::vector<double>group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),group_variable_values);
    group.setStartState(*group.getCurrentState());
   group.setStartState(*group.getCurrentState());
group.setStartState(*group.getCurrentState());
   group.setStartState(*group.getCurrentState());



  group.setPoseTarget(target_pose1);
//    group.setStartState(*group.getCurrentState());

  bool success = group.plan(my_plan);

//  ROS_INFO("Visualizing plan 1 (pose goal) :%s",success?"success":"FAILED");   

   if(success)
   {
     success=0;
    ROS_INFO("action position.x %f",target_pose1.position.x);  //注：%f可以，%d提示警告
    ROS_INFO("action position.y %f",target_pose1.position.y);
    ROS_INFO("action position.z %f",target_pose1.position.z);
    ROS_INFO("action orientation.x %f",target_pose1.orientation.x);
    ROS_INFO("action orientation.y %f",target_pose1.orientation.y);
    ROS_INFO("action orientation.z %f",target_pose1.orientation.z);
    ROS_INFO("action orientation.w %f",target_pose1.orientation.w);

    	/*open the grasp*/
    	//execute the arm
    	//group.execute(my_plan);
	group.execute(my_plan);
    	ROS_INFO("plan1 is success");
    	/*close the grasp*/
    	sleep(7);
        ROS_INFO("action1 is success");

    }



    group.setStartState(*group.getCurrentState());
    group.setStartState(*group.getCurrentState());
   group.setStartState(*group.getCurrentState());
group.setStartState(*group.getCurrentState());
   group.setStartState(*group.getCurrentState());
  group.setPoseTarget(target_pose2);
  success = group.plan(my_plan);

//  ROS_INFO("Visualizing plan 1 (pose goal) :%s",success?"success":"FAILED");   

   if(success)
   {
	group.execute(my_plan);
    	ROS_INFO("plan2 is success");
    	/*close the grasp*/
    	sleep(7);
        ROS_INFO("action2 is success");
    }


}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial_test_custom", ros::init_options::AnonymousName);
  ros::NodeHandle nh;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


mark_pose_callback();
  sleep(6.0);

//  ros::shutdown();  
  ros::spin();

  return 0;
}
