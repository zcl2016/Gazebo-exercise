/**
 * @file path_manager_base.h
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for RosPlane software
 */

#ifndef PATH_POINT_TRANSFER_H
#define PATH_POINT_TRANSFER_H

#include <ros/ros.h>
#include <hector_msgs/State.h>
#include <hector_msgs/Current_Path.h>
#include <hector_msgs/Down_Data_New.h>
// #include <hector_msgs/Waypoint.h>
// #include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/FluidPressure.h>
#include <math.h>
#include <Eigen/Eigen>
#include <hector/ControllerConfig.h>
#include <nav_msgs/Odometry.h>

namespace hector
{
class path_point_transfer
{
public:
  path_point_transfer();

  ~path_point_transfer();

protected:
  //modified by kobe

  struct waypoint_s
  {
    float w[3];
    float lat;
    float lon;
    float chi_d;
    bool chi_valid;
    float Va_d;
  };

  std::vector<waypoint_s> waypoints_;

  waypoint_s waypoint_start_;
  waypoint_s waypoint_end_;
  bool waypoint_received_;

  int num_waypoints_;
  int idx_a_; /** index to the waypoint that was most recently achieved */

  float min_distance_;
  double min_distance_titude;
  bool show; //add by kobe
  // struct input_s
  // {
  //   float pn;               /** position north */
  //   float pe;               /** position east */
  //   float lat;              //latitude
  //   float lon;              //lontitude
  //   float h;                /** altitude */
  //   float chi;              /** course angle */
  //   float va;               //add by kobe
  // };

  //   struct output_s
  //   {
  //     int  flag;             /** Inicates strait line or orbital path (1 is line, 0 is orbit, 2 is star) modified by kobe*/
  //     float Va_d;             /** Desired airspeed (m/s) */
  //     float r[3];             /** Vector to origin of straight line path (m) */
  //     float q[3];             /** Unit vector, desired direction of travel for line path */
  //     float c[3];             /** Center of orbital path (m) */
  //     float gxy[2];           /** x-y-coordinate (m) */
  //     float gll[2];           /** latitude-longtitude */
  //     float rho;              /** Radius of orbital path (m) */
  //     float h_c;
  //     int8_t lambda;          /** Direction of orbital path (cw is 1, ccw is -1) */
  // };

  struct params_s
  {
    double R_min;
  };

  // virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_; /**< vehicle state subscription */
  ros::Subscriber bebop1_state_sub_;

  ros::Subscriber new_waypoint_sub_; /**< new waypoint subscription */
  ros::Publisher current_path_pub_;  /**< controller commands publication */
  // ros::Publisher goal_info_pub_;     /**< goal info publication */
  // ros::Publisher down_data_pub_;
  ros::Subscriber waypoint_received_sub_;
  struct params_s params_;

  hector_msgs::State vehicle_state_; /**< vehicle state */
  hector_msgs::Down_Data_New point_state_;
  nav_msgs::Odometry bebop1_state_;

  double update_rate_;
  ros::Timer update_timer_;

  void vehicle_state_callback(const hector_msgs::StateConstPtr &msg);
  void bebop1_state_callback(const nav_msgs::Odometry::ConstPtr &msg);
  // float Quaternion_to_Euler(float, float, float, float);

  bool state_init_;
  void new_waypoint_callback(const hector_msgs::Down_Data_NewConstPtr &msg);
  void current_path_publish(const ros::TimerEvent &);
  // void waypoint_received_callback(const hector_msgs::Waypoint &msg);
};
} // namespace hector
#endif // PATH_MANAGER_BASE_H
