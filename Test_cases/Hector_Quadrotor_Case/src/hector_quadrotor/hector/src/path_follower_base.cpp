#include "path_follower_base.h"
#include "path_follower_example.h"

namespace hector
{

path_follower_base::path_follower_base() : nh_(ros::NodeHandle()),
                                           nh_private_(ros::NodeHandle("~"))
{
  vehicle_state_sub_ = nh_.subscribe<hector_msgs::State>("state", 1, &path_follower_base::vehicle_state_callback, this);
  bebop1_state_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &path_follower_base::bebop1_state_callback, this);
  //bebop1_state_sub_ = nh_.subscribe<nav_msgs::Odometry>("/test/odom", 1, &path_follower_base::bebop1_state_callback, this);

  current_path_sub_ = nh_.subscribe<hector_msgs::Current_Path>("current_path", 1,
                                                                 &path_follower_base::current_path_callback, this);

  nh_private_.param<double>("CHI_INFTY", params_.chi_infty, 1.0472);
  nh_private_.param<double>("K_PATH", params_.k_path, 0.025);
  nh_private_.param<double>("K_ORBIT", params_.k_orbit, 4.0);

  func_ = boost::bind(&path_follower_base::reconfigure_callback, this, _1, _2);
  server_.setCallback(func_);

  update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &path_follower_base::update, this);
  controller_commands_pub_ = nh_.advertise<hector_msgs::Controller_Commands>("controller_commands", 1);

  state_init_ = false;
  current_path_init_ = false;
}

void path_follower_base::update(const ros::TimerEvent &)
{

  struct output_s output;

  if (state_init_ == true && current_path_init_ == true)
  {
    follow(params_, input_, output);
    hector_msgs::Controller_Commands msg;
    msg.chi_c = output.chi_c;
    msg.landing=input_.landing;
    msg.takeoff=input_.takeoff;
    if (input_.Va_c_path == 0)
    {
      // ROS_INFO("Hello1");
      msg.Va_c = 0;
    }
    else
    {
      // ROS_INFO("Hello2");
      msg.Va_c = input_.Va_d;
    }

  // ROS_INFO("what %f",msg.Va_c);
  
    msg.h_c = output.h_c;
    msg.phi_ff = output.phi_ff;
    controller_commands_pub_.publish(msg);
  }
}

void path_follower_base::vehicle_state_callback(const hector_msgs::StateConstPtr &msg)
{
  // input_.pn = msg->position[0];               /** position north */
  // input_.pe = msg->position[1];               /** position east */
  // input_.h = -msg->position[2];                /** altitude */
  // input_.chi = msg->chi;
  // input_.Va = msg->Va;

  // state_init_ = true;
}
void path_follower_base::bebop1_state_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // ROS_INFO("%g", msg->pose.pose.position.x); /** position north */
  input_.pn = msg->pose.pose.position.x;     /** position north */
  input_.pe = msg->pose.pose.position.y;     /** position east */
  input_.h = msg->pose.pose.position.z;      /** altitude */
  input_.Va = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
  input_.chi = path_follower_base::Quaternion_to_Euler(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("%g %g %g %g %g",input_.pn,input_.pe,input_.h,input_.Va,input_.chi);
  state_init_ = true;
}

float path_follower_base::Quaternion_to_Euler(float q0, float q1, float q2, float q3)
{
  float Radx;
  float sum;
  sum = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (sum == 0)
  {
    sum = 0.0001;
  }
  q0 = q0 / sum;
  q1 = q1 / sum;
  q2 = q2 / sum;
  q3 = q3 / sum;
  if (fabs(q2) < fabs(q3))
  {
    Radx = asin(2.0f * (q2 * q3 + q0 * q1));
    return Radx;
  }
  else
  {
    if (q2 * q3 > 0)
    {
      Radx = 3.14159 - asin(2.0f * (q2 * q3 + q0 * q1));
      return Radx;
    }
    else
    {
      Radx = -3.14159 - asin(2.0f * (q2 * q3 + q0 * q1));
      return Radx;
    }
  }
  //Radx = asin(2.0f*(q2*q3 + q0*q1));
  //Rady = atan2(-2 * q1 * q3 + 2 * q0 * q2, q3*q3 - q2 * q2 - q1 * q1 + q0 * q0);
  //Radz = atan2(2 * q1*q2 - 2 * q0*q3, q2*q2 - q3*q3 + q0*q0 - q1*q1);
  //return Radx;
}
void path_follower_base::current_path_callback(const hector_msgs::Current_PathConstPtr &msg)
{
  if (msg->path_type == msg->LINE_PATH)
    input_.p_type = path_type::Line;
  else if (msg->path_type == msg->ORBIT_PATH)
    input_.p_type = path_type::Orbit;
  else if (msg->path_type == msg->STAR_PATH) // add by kobe
    input_.p_type = path_type::Star;

  input_.Va_d = msg->Va_d;
  // ROS_INFO("hello %f",input_.Va_d);
  for (int i = 0; i < 3; i++)
  {
    input_.r_path[i] = msg->r[i];
    input_.q_path[i] = msg->q[i];
    input_.c_orbit[i] = msg->c[i];
  }
  input_.rho_orbit = msg->rho;
  input_.lam_orbit = msg->lambda;
  input_.h_c_path = msg->h_c; //add by kobe
  input_.Va_c_path = msg->Va_d;
  input_.landing = msg->landing;
  input_.takeoff = msg->takeoff;
  current_path_init_ = true;
}

void path_follower_base::reconfigure_callback(hector::FollowerConfig &config, uint32_t level)
{
  params_.chi_infty = config.CHI_INFTY;
  params_.k_path = config.K_PATH;
  params_.k_orbit = config.K_ORBIT;
}
} // namespace hector

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_path_follower");
  hector::path_follower_base *path = new hector::path_follower_example();

  ros::spin();

  return 0;
}
