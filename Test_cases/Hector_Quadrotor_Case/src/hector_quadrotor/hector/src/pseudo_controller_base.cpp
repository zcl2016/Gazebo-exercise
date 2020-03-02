#include "pseudo_controller_base.h"
#include "pseudo_controller_example.h"
// modified by kobe
namespace hector
{

pseudo_controller_base::pseudo_controller_base():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle())
{
  //ROS_INFO("hello");
  vehicle_state_sub_ = nh_.subscribe("state", 10, &pseudo_controller_base::vehicle_state_callback, this);
  bebop1_state_sub_ = nh_.subscribe("ground_truth/state", 10, &pseudo_controller_base::bebop1_state_callback, this);
  //bebop1_state_sub_ = nh_.subscribe("/test/odom", 10, &pseudo_controller_base::bebop1_state_callback, this);
  


  controller_commands_sub_ = nh_.subscribe("controller_commands", 10, &pseudo_controller_base::controller_commands_callback,this);

  memset(&vehicle_state_, 0, sizeof(vehicle_state_));
  memset(&bebop1_state_, 0, sizeof(bebop1_state_));

  memset(&controller_commands_, 0, sizeof(controller_commands_));

  nh_private_.param<double>("TRIM_T", params_.trim_t, 0.3);

  nh_private_.param<double>("ALT_TOZ", params_.alt_toz, 1.5);
  nh_private_.param<double>("ALT_HZ", params_.alt_hz, 0.2);
  nh_private_.param<double>("TAU", params_.tau, 5.0);
  nh_private_.param<double>("COURSE_KP", params_.c_kp, 0.7329);
  //nh_private_.param<double>("COURSE_KD", params_.c_kd, 0.0);
  nh_private_.param<double>("COURSE_KI", params_.c_ki, 0.0);

  nh_private_.param<double>("AS_PITCH_KI", params_.a_p_ki, 0.0);
  nh_private_.param<double>("AS_THR_KP", params_.a_t_kp, 0.9);
  nh_private_.param<double>("AS_THR_KD", params_.a_t_kd, 0.0);
  nh_private_.param<double>("AS_THR_KI", params_.a_t_ki, 0.0);
  nh_private_.param<double>("ALT_KP", params_.a_kp, 0.045);
  nh_private_.param<double>("ALT_KD", params_.a_kd, 0.0);
  nh_private_.param<double>("ALT_KI", params_.a_ki, 0.0);

  //nh_private_.param<double>("MAX_A", params_.max_a, 0.523);//rotate_trans
  //nh_private_.param<double>("MAX_T", params_.max_t, 1.0);
  nh_private_.param<double>("MAX_T", params_.max_t, 200.0);

  func_ = boost::bind(&pseudo_controller_base::reconfigure_callback, this, _1, _2);
  server_.setCallback(func_);

  //pesudors_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 10);
  pesudors_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  internals_pub_ = nh_.advertise<hector_msgs::Controller_Internals>("controller_inners", 10);
  act_pub_timer_ = nh_.createTimer(ros::Duration(1.0/100.0), &pseudo_controller_base::actuator_controls_publish, this);

  command_recieved_ = false;
}

void pseudo_controller_base::vehicle_state_callback(const hector_msgs::StateConstPtr &msg)
{
  vehicle_state_ = *msg;
}

void pseudo_controller_base::bebop1_state_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  bebop1_state_ = *msg;
}

void pseudo_controller_base::controller_commands_callback(const hector_msgs::Controller_CommandsConstPtr &msg)
{
  command_recieved_ = true;
  controller_commands_ = *msg;
}

void pseudo_controller_base::reconfigure_callback(hector::ControllerConfig &config, uint32_t level)
{
  params_.trim_t = config.TRIM_T;

  params_.c_kp = config.COURSE_KP;
  //params_.c_kd = config.COURSE_KD;
  params_.c_ki = config.COURSE_KI;

  params_.a_p_ki = config.AS_PITCH_KI;

  params_.a_t_kp = config.AS_THR_KP;
  params_.a_t_kd = config.AS_THR_KD;
  params_.a_t_ki = config.AS_THR_KI;

  params_.a_kp = config.ALT_KP;
  params_.a_kd = config.ALT_KD;
  params_.a_ki = config.ALT_KI;

}


void pseudo_controller_base::actuator_controls_publish(const ros::TimerEvent &)
{
  struct input_s input;
  //input.h = -vehicle_state_.position[2];
  //input.va = vehicle_state_.Va;
  //input.chi = vehicle_state_.chi;
  input.h =  bebop1_state_.pose.pose.position.z;                /** altitude */
  input.va = sqrt(pow(bebop1_state_.twist.twist.linear.x,2)+pow(bebop1_state_.twist.twist.linear.y,2));
  input.chi =pseudo_controller_base::Quaternion_to_Euler(bebop1_state_.pose.pose.orientation.x,bebop1_state_.pose.pose.orientation.y,bebop1_state_.pose.pose.orientation.z,bebop1_state_.pose.pose.orientation.w);
  // ROS_INFO("LOOK here1:  %f",input.chi);
  input.Va_c = controller_commands_.Va_c;
  input.h_c = controller_commands_.h_c;//3>controller_commands_.h_c)?3:controller_commands_.h_c;
  input.chi_c = controller_commands_.chi_c;
  input.takeoff = controller_commands_.takeoff;
  input.landing = controller_commands_.landing;
  input.Ts = 0.01f;
  if (controller_commands_.Va_c==0)
  {
    reached=1;
  }
  struct output_s output;
  
  if (command_recieved_ == true)
  {
    control(params_, input, output);

    //convert_to_pwm(output);
    geometry_msgs::Twist cmd_vel;
    //rosflight_msgs::Command pseudors;
    /* publish pseudors controls */
    cmd_vel.linear.x = output.forward_trans;//head vel
    cmd_vel.linear.y = output.left_right_trans;
    cmd_vel.linear.z = output.up_down_trans;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;//output.roll_trans;
if (input.va > 0.1)
    cmd_vel.angular.z = output.rotate_trans;//output.rotate_trans
else
cmd_vel.angular.z = 0;

    pesudors_pub_.publish(cmd_vel);
    //ROS_INFO("hello %f",bebop1_state_.pose.pose.position.z);
    // ROS_INFO("x-y-z-w: %f %f %f %f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z,cmd_vel.angular.z);
    // ROS_INFO("six: %f %f %f %f %f %f\n",input.h,input.h_c,input.va,input.Va_c,input.chi,input.chi_c);
    // ROS_INFO("position: %f %f %f",bebop1_state_.pose.pose.position.x,bebop1_state_.pose.pose.position.y,bebop1_state_.pose.pose.position.z);
    if (internals_pub_.getNumSubscribers() > 0)
    {
      hector_msgs::Controller_Internals inners;
      //sinners.phi_c = output.phi_c;
      //inners.theta_c = output.theta_c;
      switch (output.current_zone)
      {
      case alt_zones::TAKE_OFF:
        inners.alt_zone = inners.ZONE_TAKE_OFF;
        break;
      case alt_zones::CLIMB:
        inners.alt_zone = inners.ZONE_CLIMB;
        break;
      case alt_zones::DESCEND:
        inners.alt_zone = inners.ZONE_DESEND;
        break;
      case alt_zones::ALTITUDE_HOLD:
        inners.alt_zone = inners.ZONE_ALTITUDE_HOLD;
        break;
      default:
        break;
      }
      inners.aux_valid = false;
      internals_pub_.publish(inners);
    }
  }
}

float pseudo_controller_base::Quaternion_to_Euler(float q0,float q1,float q2,float q3)
{
    float Radx;
    float sum;
    sum = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (sum==0)
    {
      sum=0.0001;
    }
    q0 = q0 / sum;
    q1 = q1 / sum;
    q2 = q2 / sum;
    q3 = q3 / sum;

    // ROS_INFO("LOOK here2:  %f",asin(2.0f*(q2*q3 + q0*q1)));
    if (fabs(q2)<fabs(q3))
    {
      Radx = asin(2.0f*(q2*q3 + q0*q1));
      return Radx;
    }
    else
    {
      if(q2*q3>0)
      {
         Radx = 3.14159-asin(2.0f*(q2*q3 + q0*q1));
         return Radx;
      }
      else
      {
         Radx = -3.14159-asin(2.0f*(q2*q3 + q0*q1));
         return Radx;
      }
    }
}

} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_controller");
  hector::pseudo_controller_base *cont = new hector::pseudo_controller_example();

  ros::spin();

  return 0;
}
