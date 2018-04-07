#include "ros/ros.h"
#include "../cfg/ros-roomba/roomba_500_series/msg_gen/cpp/include/roomba_500_series/Bumper.h"
#include "cmath.h"
#include "geometry_msgs/Twist.h"


bool left_bumper, right_bumper;
void chatterCallback(const std_msgs::Bumper::ConstPtr& msg)
	{
	
		left_bumper  = msg->left.state;
		right_bumper = msg->right.state;
	//	ROS_INFO("New : %d", msg->header.seq);
	//	for(int i = 0; i < msg->state.size(); i++)
	//	{
	//		if(msg->state[i].bumper_state)
	//			{							}
	//	}
	}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "collide");

	ros::NodeHandle n;

	ros::Publisher vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1)
  geometry_msgs::Twist cmd_vel;


if (left_bumper ==1){
bool judge_left = true;
   while(judge_left == true){
    if(judge_left == false){ break; }

    cmd_vel.linear.x = 0.1;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);

    while(left_bumper == 1){
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z += 3.14; //turn right while left bumper is true
	vel_pub_.publish(cmd_vel);
	
	ros::Subcriber sub= n.subscribe("bumper",1000,chatterCallback());
    			   }

    while(right_bumper == 0 || left_bumper == 0){
     if(judge_left == false){ break; }

     cmd_vel.linear.x = 0.2;
     cmd_vel.angular.z = 0.0; 
     vel_pub_.publish(cmd_vel);  


	ros::Subcriber sub= n.subscribe("bumper",1000,chatterCallback());

     if(right_bumper == 1){ judge_left = false; }
     if(left_bumper == 1){ break; }
    }
   }
   return;
  }

if (right_bumper ==1){
bool judge_right = true;
   while(judge_right == true){
    if(judge_right == false){ break; }

    cmd_vel.linear.x = 0.1;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);

    while(right_bumper == 1){
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z += 3.14; //turn right while left bumper is true
	vel_pub_.publish(cmd_vel);
	
	ros::Subcriber sub= n.subscribe("bumper",1000,chatterCallback());
    			   }

    while(right_bumper == 0 || left_bumper == 0){
     if(judge_right == false){ break; }

     cmd_vel.linear.x = 0.2;
     cmd_vel.angular.z = 0.0; 
     vel_pub_.publish(cmd_vel);  


	ros::Subcriber sub= n.subscribe("bumper",1000,chatterCallback());

     if(left_bumper == 1){ judge_right = false; }
     if(right_bumper == 1){ break; }
    }
   }
   return;
  }
	ros::spin();
}




 
