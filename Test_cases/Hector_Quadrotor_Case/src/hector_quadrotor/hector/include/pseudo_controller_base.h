/**
 *
 * Refer autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 *pseudo fix-wing 四旋翼模拟固定翼飞行，订阅当前实际及命令的速度、高度和角度，发布向前，向上下速度和角度
 * @author AIRC-DA group,questions contack kobe.
 */

#ifndef PSEUDO_CONTROLLER_BASE_H
#define PSEUDO_CONTROLLER_BASE_H

#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <geometry_msgs/Twist.h>
#include <hector_msgs/State.h>
#include <hector_msgs/Controller_Commands.h>
#include <hector_msgs/Controller_Internals.h>

#include <dynamic_reconfigure/server.h>
#include <hector/ControllerConfig.h>
#include <nav_msgs/Odometry.h>

namespace hector
{

enum class alt_zones
{
  PREPARE,
  TAKE_OFF,
  CLIMB,
  DESCEND,
  ALTITUDE_HOLD,
  LANDING
};

class pseudo_controller_base
{
public:
  pseudo_controller_base();
  float spin();

protected:

  struct input_s
  {
    float Ts;               /** time step */
    float va;               /** airspeed */
    float h;                /** altitude */
    float chi;              /** course angle */
    float Va_c;             /** commanded airspeed (m/s) */
    float h_c;              /** commanded altitude (m) */
    float chi_c;            /** commanded course (north)(-pi to pi,clockwise:-) */
    bool takeoff;
    bool landing;
  };

  struct output_s
  {
    //float theta_c; //俯仰角--即平行于机身轴线并指向飞行器前方的向量与地面的夹角
    //float phi_c;   //滚转角--机体坐标系OZb轴与通过机体OXb轴的铅垂面间的夹角，机体向右滚为正，反之为负
    //float delta_a; //副翼偏移量-转弯用-副翼跳变造成油门和升降舵数值的波动
    //float delta_e; //升降舵偏移量-拉高降低
    //float delta_r; //方向舵偏移量-保持稳定，消除测滑
    //float delta_t; //油门偏移量
    float forward_trans;  //前进力度
    float up_down_trans;  //向上、向下力度,上正下负
    float left_right_trans;  //向左右力度,左正右负
    float roll_trans;  //左右滚转，x轴,逆时针为正，顺时针为负，[-pi/2,pi/2]
    float rotate_trans;   //z旋转,逆时针为正，顺时针为负，[-pi/2,pi/2]
    float rotate_value;
    alt_zones current_zone;
  };

  struct params_s
  {
    double alt_hz;           /**< altitude hold zone */
    double alt_toz;          /**< altitude takeoff zone */
    double tau;
    double c_kp;
    //double c_kd;
    double c_ki;

    double a_p_kp;
    double a_p_kd;
    double a_p_ki;

    double a_t_ki;
    double a_t_kp;
    double a_t_kd;

    double a_kp;
    double a_kd;
    double a_ki;

    double trim_t;
    //double max_a;y
    double max_t;
  };
  bool reached;
  virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;
  ros::Subscriber bebop1_state_sub_;

  ros::Subscriber controller_commands_sub_;
  ros::Publisher pesudors_pub_;//actuators
  ros::Publisher internals_pub_;
  ros::Timer act_pub_timer_;

  struct params_s params_;            /**< params */
  hector_msgs::Controller_Commands controller_commands_;
  hector_msgs::State vehicle_state_;
  nav_msgs::Odometry bebop1_state_;
  
  void vehicle_state_callback(const hector_msgs::StateConstPtr &msg);
  void bebop1_state_callback(const nav_msgs::Odometry::ConstPtr &msg);
  float Quaternion_to_Euler(float,float,float,float);

  void controller_commands_callback(const hector_msgs::Controller_CommandsConstPtr &msg);
  bool command_recieved_;

  dynamic_reconfigure::Server<hector::ControllerConfig> server_;
  dynamic_reconfigure::Server<hector::ControllerConfig>::CallbackType func_;

  void reconfigure_callback(hector::ControllerConfig &config, uint32_t level);

  /*Convert from deflection angle to pwm*/
  //void convert_to_pwm(struct output_s &output);

  /* Publish the outputs  */
  void actuator_controls_publish(const ros::TimerEvent &);
};
} //end namespace

#endif // PSEUDO_CONTROLLER_BASE_H
