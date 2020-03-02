//* @author AIRC-DA group,questions contack kobe.

#include "estimator_base_99pub.h"

namespace hector
{

estimator_base_99pub::estimator_base_99pub():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{

  true_state_sub_ = nh_.subscribe("truth", 1, &estimator_base_99pub::truestateCallback, this);
  nh_private_.param<double>("update_rate", update_rate_, 100.0);
  update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &estimator_base_99pub::update, this);
  vehicle_state99_pub_ = nh_.advertise<hector_msgs::State29>("state29", 10);
  state_init_ = false;
}

void estimator_base_99pub::update(const ros::TimerEvent &)
{
  hector_msgs::State29 msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = 1; // Denotes global frame
  if (state_init_)
  {
    msg.position_gps[0]=(true_state_.position[0]*180)/(EARTH_RADIUS*M_PI)+true_state_.initial_lat;
    // pn = EARTH_RADIUS*(msg.latitude - init_lat_)*M_PI/180.0;
    msg.position_gps[1]=(true_state_.position[1]*180)/(EARTH_RADIUS*cos(true_state_.initial_lat*M_PI/180.0)*M_PI)+true_state_.initial_lon;
    // pe = EARTH_RADIUS*cos(init_lat_*M_PI/180.0)*(msg.longitude - init_lon_)*M_PI/180.0;
    // input_.gps_h = msg.altitude - init_alt_;
    msg.position_gps[2] =-true_state_.position[2]-true_state_.initial_alt; //>0
    msg.attitude_angle[0] = true_state_.phi;
    msg.attitude_angle[1] = true_state_.theta;
    msg.attitude_angle[2] = true_state_.psi;
    msg.velocity[0] = true_state_.Vn;
    msg.velocity[1] = true_state_.Ve;
    msg.velocity[2] = true_state_.Vd;
    msg.angular_velocity[0] = true_state_.p;
    msg.angular_velocity[1] = true_state_.q;
    msg.angular_velocity[2] = true_state_.r;
    msg.acceleration[0] = -1;
    msg.acceleration[1] = -1;
    msg.acceleration[2] = -1;
    msg.electric_quantity = -1;
    msg.state_word = -1;
    // ROS_INFO("GPS-position:  %g %g %g",msg.position_gps[0],msg.position_gps[1],msg.position_gps[2]);
    // ROS_INFO("xyz-position:  %g %g %g",true_state_.position[0],true_state_.position[1],true_state_.position[2]);
  }
  else
  {
    // ROS_WARN("No plane truth data.");
  }

  vehicle_state99_pub_.publish(msg);
}


void estimator_base_99pub::truestateCallback(const hector_msgs::StateConstPtr &msg)
{
  true_state_ = *msg;
  state_init_ = true;
}


} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_estimator_29pub");
  hector::estimator_base_99pub *cont = new hector::estimator_base_99pub();

  ros::spin();

  return 0;
}
