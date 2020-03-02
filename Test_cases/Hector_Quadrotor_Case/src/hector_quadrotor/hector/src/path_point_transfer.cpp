#include "path_point_transfer.h"
// #include "path_point_transfer.h"
#define EARTH_RADIUS 6378137.0f //6378145.0f

namespace hector
{

float init_lat_ = 41.71671500; /**< Initial latitude in degrees */
float init_lon_ = 87.53306850; /**< Initial longitude in degrees */
float init_alt_ = 1180;        //>0  1180

float latitude2pn(float latitude, float init_lat_)
{
  return EARTH_RADIUS * (latitude - init_lat_) * M_PI / 180.0;
}

float longtitude2pe(float longitude, float init_lat_, float init_lon_)
{
  return EARTH_RADIUS * cos(init_lat_ * M_PI / 180.0) * (longitude - init_lon_) * M_PI / 180.0;
}

path_point_transfer::path_point_transfer() : nh_(ros::NodeHandle()), /** nh_ stuff added here */
                                             nh_private_(ros::NodeHandle("~"))
{
  nh_private_.param<double>("update_rate", update_rate_, 10.0);

  // vehicle_state_sub_ = nh_.subscribe("state", 1000, &path_point_transfer::vehicle_state_callback, this);
  bebop1_state_sub_ = nh_.subscribe("ground_truth/state", 10, &path_point_transfer::bebop1_state_callback, this);
  new_waypoint_sub_ = nh_.subscribe("serial_commu_down", 1000, &path_point_transfer::new_waypoint_callback, this);

  current_path_pub_ = nh_.advertise<hector_msgs::Current_Path>("current_path", 1000);

  update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &path_point_transfer::current_path_publish, this);

  state_init_ = false;

  waypoint_received_ = false;
}

path_point_transfer::~path_point_transfer()
{
  // stop();
}

// void path_point_transfer::vehicle_state_callback(const hector_msgs::StateConstPtr &msg)
// {
//   vehicle_state_ = *msg;
//   state_init_ = true;
// }

void path_point_transfer::bebop1_state_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  bebop1_state_ = *msg;
  state_init_ = true;
}

void path_point_transfer::new_waypoint_callback(const hector_msgs::Down_Data_NewConstPtr &msg)
{
  point_state_ = *msg;
  waypoint_received_ = true;
}

void path_point_transfer::current_path_publish(const ros::TimerEvent &)
{
  Eigen::Vector3f w_start;
  Eigen::Vector3f w_end;
  w_start << bebop1_state_.pose.pose.position.x, bebop1_state_.pose.pose.position.y, bebop1_state_.pose.pose.position.z; //aircraft's position
  w_end << latitude2pn(point_state_.latitude, init_lat_), longtitude2pe(point_state_.longitude, init_lat_, init_lon_), -point_state_.altitude;
  ROS_WARN("startpn,%g,startpe,%g,pn,%g,pe,%g",vehicle_state_.position[0], vehicle_state_.position[1],latitude2pn(point_state_.latitude, init_lat_), longtitude2pe(point_state_.longitude, init_lat_, init_lon_));
  Eigen::Vector3f q_1 = (w_end - w_start).normalized();
  hector_msgs::Current_Path current_path;
  current_path.path_type = current_path.LINE_PATH;
  current_path.Va_d = point_state_.plane_speed;
  for (int i = 0; i < 3; i++)
  {
    current_path.r[i] = w_start[i];
    current_path.q[i] = q_1[i];
  }
  current_path.h_c = -point_state_.altitude; //current_path.h_c<0 和state统一
  current_path_pub_.publish(current_path);
}

} // namespace hector

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_path_point_transfer");
  hector::path_point_transfer *cont = new hector::path_point_transfer();
  // hector::path_point_transfer pp;
  ros::spin();

  return 0;
}
