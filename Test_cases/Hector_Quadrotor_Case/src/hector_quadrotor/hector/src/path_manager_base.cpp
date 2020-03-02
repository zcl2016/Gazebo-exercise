#include "path_manager_base.h"
#include "path_manager_example.h"

namespace hector
{

path_manager_base::path_manager_base() : nh_(ros::NodeHandle()), /** nh_ stuff added here */
                                         nh_private_(ros::NodeHandle("~"))
{
  nh_private_.param<double>("R_min", params_.R_min, 25); //25
  nh_private_.param<double>("update_rate", update_rate_, 10.0);

  vehicle_state_sub_ = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);
  bebop1_state_sub_ = nh_.subscribe("ground_truth/state", 10, &path_manager_base::bebop1_state_callback, this);
  //bebop1_state_sub_ = nh_.subscribe("/test/odom", 10, &path_manager_base::bebop1_state_callback, this);

  new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);
  
  current_path_pub_ = nh_.advertise<hector_msgs::Current_Path>("current_path", 10);
  goal_info_pub_ = nh_.advertise<hector_msgs::Goal_Info>("Goal_Info", 10);
  down_data_pub_ = nh_.advertise<hector_msgs::Down_Data_New>("serial_commu_down", 10);

  waypoint_received_sub_ = nh_.subscribe("new_waypoint", 10, &path_manager_base::waypoint_received_callback, this);

  update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &path_manager_base::current_path_publish, this);

  num_waypoints_ = 0;

  state_init_ = false;

  waypoint_received_ = false;
}

void path_manager_base::waypoint_received_callback(const hector_msgs::Waypoint &msg)
{
  float ell = sqrtf((waypoint_end_.w[0] - msg.w[0]) * (waypoint_end_.w[0] - msg.w[0]) +
                    (waypoint_end_.w[1] - msg.w[1]) * (waypoint_end_.w[1] - msg.w[1]));
  if (ell < 2.0 * params_.R_min)
  {
    ROS_ERROR("The distance between nodes must be larger than 2R.");
    return;
  }
  waypoint_start_ = waypoint_end_;

  waypoint_end_.w[0] = msg.w[0];
  waypoint_end_.w[1] = msg.w[1];
  waypoint_end_.w[2] = msg.w[2];
  waypoint_end_.chi_d = msg.chi_d;
  waypoint_end_.chi_valid = msg.chi_valid;
  waypoint_end_.Va_d = msg.Va_d;

  waypoint_received_ = true;
}

void path_manager_base::vehicle_state_callback(const hector_msgs::StateConstPtr &msg)
{
  vehicle_state_ = *msg;
  //state_init_ = true;
}

void path_manager_base::bebop1_state_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  bebop1_state_ = *msg;
  state_init_ = true;
}

void path_manager_base::new_waypoint_callback(const hector_msgs::Waypoint &msg)
{
  if (msg.clear_wp_list == true)
  {
    waypoints_.clear();
    num_waypoints_ = 0;
    idx_a_ = 0;
    min_distance_ = 1000000000; //add by kobe, just need to be larger than the large-enter-ball's radius.
    min_distance_titude = 10000000000; 
    //return;
  }
  if (msg.set_current || num_waypoints_ == 0)//mean to set the start point as the a goal-never use---kobe denote
  {
    waypoint_s currentwp;
    currentwp.w[0] = bebop1_state_.pose.pose.position.x;
    currentwp.w[1] = bebop1_state_.pose.pose.position.y;
    currentwp.w[2] = (-bebop1_state_.pose.pose.position.z > -25 ? msg.w[2] : -bebop1_state_.pose.pose.position.z);
    currentwp.chi_d= path_manager_base::Quaternion_to_Euler(bebop1_state_.pose.pose.orientation.x,bebop1_state_.pose.pose.orientation.y,bebop1_state_.pose.pose.orientation.z,bebop1_state_.pose.pose.orientation.w);
    // currentwp.w[0] = vehicle_state_.position[0];
    // currentwp.w[1] = vehicle_state_.position[1];
    // currentwp.w[2] = (vehicle_state_.position[2] > -25 ? msg.w[2] : vehicle_state_.position[2]);
    // currentwp.chi_d = vehicle_state_.chi;
    // add by kobe
    currentwp.lat = vehicle_state_.Wd;
    currentwp.lon = vehicle_state_.Jd;

    currentwp.chi_valid = msg.chi_valid;
    currentwp.Va_d = msg.Va_d;
    currentwp.landing=msg.landing;
    currentwp.takeoff=msg.takeoff;
    waypoints_.clear();
    waypoints_.push_back(currentwp);
    num_waypoints_ = 1;
    idx_a_ = 0;
    min_distance_ = 10000000000; //add by kobe, just need to be larger than the large-enter-ball's radius.
    min_distance_titude = 10000000000; 
    show = 1;
    show2=1;
  }
  waypoint_s nextwp;
  nextwp.w[0] = msg.w[0];
  nextwp.w[1] = msg.w[1];
  nextwp.w[2] = msg.w[2];
  // add by kobe
  nextwp.lat=msg.lat;
  nextwp.lon=msg.lon;

  nextwp.chi_d = msg.chi_d;
  nextwp.chi_valid = msg.chi_valid;
  nextwp.Va_d = msg.Va_d;
  nextwp.landing=msg.landing;
  nextwp.takeoff=msg.takeoff;
  waypoints_.push_back(nextwp);
  num_waypoints_++;
  // ROS_INFO("num_waypoints_: %d", num_waypoints_);
}

void path_manager_base::current_path_publish(const ros::TimerEvent &)
{
  struct input_s input;
  input.pn = bebop1_state_.pose.pose.position.x;               /** position north */
  input.pe = bebop1_state_.pose.pose.position.y;              /** position east */
  input.h =  bebop1_state_.pose.pose.position.z;                /** altitude */
  input.chi =path_manager_base::Quaternion_to_Euler(bebop1_state_.pose.pose.orientation.x,bebop1_state_.pose.pose.orientation.y,bebop1_state_.pose.pose.orientation.z,bebop1_state_.pose.pose.orientation.w);
  input.va = sqrt(pow(bebop1_state_.twist.twist.linear.x,2)+pow(bebop1_state_.twist.twist.linear.y,2));
  // input.pn = vehicle_state_.position[0]; /** position north */
  // input.pe = vehicle_state_.position[1]; /** position east */
  // input.h = -vehicle_state_.position[2]; /** altitude */
  // input.chi = vehicle_state_.chi;
  // input.va = vehicle_state_.Va;

  //add by kobe
  input.lat=vehicle_state_.Wd;
  input.lon=vehicle_state_.Jd;

  struct output_s output;

  if (state_init_ == true)
  {
    manage(params_, input, output);
  }

  hector_msgs::Current_Path current_path;

  hector_msgs::Goal_Info goal_info;
  hector_msgs::Down_Data_New down_data;
  // modified by kobe
  if (output.flag == 1)
    current_path.path_type = current_path.LINE_PATH;
  else if (output.flag == 0)
    current_path.path_type = current_path.ORBIT_PATH;
  else if (output.flag == 2)
    current_path.path_type = current_path.STAR_PATH;

  for (int i = 0; i < 3; i++)
  {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  current_path.rho = output.rho;
  current_path.lambda = output.lambda;
  current_path.h_c = output.h_c;
  current_path.Va_d = output.Va_c; //new speed
  current_path.takeoff=output.takeoff;
  current_path.landing=output.landing;
  current_path_pub_.publish(current_path);

  goal_info.altitude=-output.h_c;
  goal_info.v_d=output.Va_d;
  goal_info.action=1;//desired action move
  for (int i = 0; i < 2; i++)
  {
    goal_info.xy[i] = output.gxy[i];
    goal_info.ll[i] = output.gll[i];
  }
  goal_info_pub_.publish(goal_info); // add by kobe

  down_data.latitude   = output.gll[0];
  down_data.longitude  = output.gll[1];
  down_data.altitude   = -output.h_c;
  down_data.plane_speed= output.Va_d;
  down_data.status_word= 1;
  //ROS_INFO("Look here %f %f %f %f",down_data.latitude,down_data.longitude,down_data.altitude,output.Va_d); 
  down_data_pub_.publish(down_data); // add by kobe
}

//add by kobe
float path_manager_base::Quaternion_to_Euler(float q0, float q1, float q2, float q3)
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
}

} // namespace hector

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_path_manager");
  hector::path_manager_base *est = new hector::path_manager_example();

  ros::spin();

  return 0;
}
