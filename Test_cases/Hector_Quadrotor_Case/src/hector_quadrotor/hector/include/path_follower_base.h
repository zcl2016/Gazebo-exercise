//* @author AIRC-DA group,questions contack kobe.

#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <ros/ros.h>
#include <hector_msgs/State.h>
#include <hector_msgs/Controller_Commands.h>
#include <dynamic_reconfigure/server.h>
#include <hector/FollowerConfig.h>
#include <hector_msgs/Current_Path.h>
#include <nav_msgs/Odometry.h>
#define EARTH_RADIUS 6378145.0f
namespace hector
{

enum class path_type
{
  Orbit,
  Line,
  Star  //add by kobe
};

class path_follower_base
{
public:
  path_follower_base();
  float spin();

protected:

  struct input_s
  {
    enum path_type p_type;
    float Va_d;
    float r_path[3];
    float q_path[3];
    float c_orbit[3];
    float rho_orbit;
    int lam_orbit;
    float pn;               /** position north */
    float pe;               /** position east */
    float h;                /** altitude */
    float Va;               /** airspeed */
    float chi;              /** course angle */
    float h_c_path;         /** goal attitude */
    float Va_c_path;         /** goal attitude */
    bool landing;
    bool takeoff;
  };

  struct output_s
  {
    double Va_c;             /** commanded airspeed (m/s) */
    double h_c;              /** commanded altitude (m) */
    double chi_c;            /** commanded course (rad) */
    double phi_ff;           /** feed forward term for orbits (rad) */
  };

  struct params_s
  {
    double chi_infty;
    double k_path;
    double k_orbit;
  };

  virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;
  ros::Subscriber bebop1_state_sub_;
  ros::Subscriber current_path_sub_;

  ros::Publisher controller_commands_pub_;

  double update_rate_ = 100.0;
  ros::Timer update_timer_;

  hector_msgs::Controller_Commands controller_commands_;
  struct params_s  params_;            /**< params */
  struct input_s input_;

  void vehicle_state_callback(const hector_msgs::StateConstPtr &msg);
  bool state_init_;

  void bebop1_state_callback(const nav_msgs::Odometry::ConstPtr &msg);
  float Quaternion_to_Euler(float,float,float,float);

  void current_path_callback(const hector_msgs::Current_PathConstPtr &msg);
  bool current_path_init_;

  dynamic_reconfigure::Server<hector::FollowerConfig> server_;
  dynamic_reconfigure::Server<hector::FollowerConfig>::CallbackType func_;
  void reconfigure_callback(hector::FollowerConfig &config, uint32_t level);

  void update(const ros::TimerEvent &);
};

} // end namespace

#endif // PATH_FOLLOWER_BASE_H
