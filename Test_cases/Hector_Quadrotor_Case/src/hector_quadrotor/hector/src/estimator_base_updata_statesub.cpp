//* @author AIRC-DA group,questions contack kobe.

#include "estimator_base_updata_statesub.h"

namespace hector
{

estimator_base_updata_statesub::estimator_base_updata_statesub() : nh_(ros::NodeHandle()),
                                                     nh_private_(ros::NodeHandle("~"))
{
  state29_sub_ = nh_.subscribe("/serial_commu_up", 10, &estimator_base_updata_statesub::state99Callback, this);
  nh_private_.param<double>("update_rate", update_rate_, 100.0);
  update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &estimator_base_updata_statesub::update, this);
  vehicle_state_pub_ = nh_.advertise<hector_msgs::State>("state", 10);

  state_init_ = false;
  init_lat_=0;       /**< Initial latitude in degrees */
  init_lon_=0;       /**< Initial longitude in degrees */
  init_alt_=0;       //>0
}

void estimator_base_updata_statesub::update(const ros::TimerEvent &)
{

  hector_msgs::State msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = 1; // Denotes global frame
  if (state_init_)
  {
    msg.position[0] = EARTH_RADIUS*(state29_.latitude - init_lat_)*M_PI/180.0;
    // pn = EARTH_RADIUS*(msg.latitude - init_lat_)*M_PI/180.0;
    msg.position[1] = EARTH_RADIUS*cos(init_lat_*M_PI/180.0)*(state29_.longitude - init_lon_)*M_PI/180.0;
    // pe = EARTH_RADIUS*cos(init_lat_*M_PI/180.0)*(msg.longitude - init_lon_)*M_PI/180.0;
    msg.position[2] = -state29_.altitude- init_alt_;  //<0
    //input_.gps_h = msg.altitude - init_alt_;
    msg.Wd=state29_.latitude;
    msg.Jd=state29_.longitude;
    msg.Va=sqrt(pow(state29_.north_direction_speed, 2.0) + pow(state29_.east_direction_speed, 2.0) + pow(state29_.ground_direction_speed, 2.0));
    msg.Vg=msg.Va; //true for rosflight
    msg.phi=state29_.roll_angle;
    msg.theta=state29_.pitch_angle;
    msg.psi=state29_.yaw_angle;
    //msg.chi=msg.psi;//watch here
    msg.chi = atan2(msg.Va*sin(msg.psi), msg.Va*cos(msg.psi));
    msg.p=state29_.angular_rate_x;
    msg.q=state29_.angular_rate_y;
    msg.r=state29_.angular_rate_z;

    msg.initial_lat = init_lat_;
    msg.initial_lon = init_lon_;
    msg.initial_alt = init_alt_;
    // ROS_WARN("Recieved up-data data.");
    //ROS_INFO("Hello--- %f %f %f %f",state29_.latitude,state29_.longitude,msg.Vg,state29_.altitude);
  }
  else
  {
    // ROS_WARN("No up-data data.");
  }
  vehicle_state_pub_.publish(msg);
}

void estimator_base_updata_statesub::state99Callback(const hector_msgs::Up_Data_NewConstPtr &msg)
{
  state29_ = *msg;
  state_init_ = true;
}
} // namespace hector

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_estimator_updata_statesub");
  hector::estimator_base_updata_statesub *cont = new hector::estimator_base_updata_statesub();
  ros::spin();

  return 0;
}
