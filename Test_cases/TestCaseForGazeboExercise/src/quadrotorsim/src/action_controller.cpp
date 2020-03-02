/************************************************************
 * Author: zenglei                                          *
 * Date:  2019-12-03                                        *
 * Description: send the takeoff command to the uav by ros  *
 * Modified: None                                           *
 * **********************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  std::string topicName = "/TakeOff/cmd_vel";

  // n.getParam("topicname", topicName);
  ros::param::get("~topicname", topicName);
  
  // std::cout << "****************************TopicName: " << topicName.c_str() << "*****************************" << std::endl;

  ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>(topicName, 1);

  ros::Rate loop_rate(100);

  int nCount = 0;

  while (ros::ok())
  {

    // std::cout << "Publish Cmd Vel Topic!!" << std::endl;
    geometry_msgs::Twist cmd_vel_msg;
    
    /*
    if(nCount % 3 == 0)
    {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 1;
    }else if(nCount % 3 == 1)
    {
        cmd_vel_msg.linear.x = 1;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 0;
    }else if(nCount % 3 == 2)
    {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.linear.y = 1;
        cmd_vel_msg.linear.z = 0;
    }
    */
    // if(nCount % 5000 == 1)
    {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 1;
    }
   
    
    nCount ++;
    if(nCount == 50000) nCount = 0;
    // ROS_INFO("%s", msg.data.c_str());

    cmd_vel_pub_.publish(cmd_vel_msg);

    // ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}