
// modified by kobe and zhangshuai

#ifndef ESTIMATOR_BASE_PUB_H
#define ESTIMATOR_BASE_PUB_H

#include <ros/ros.h>
#include <hector_msgs/State.h>
#include <hector_msgs/State29.h>
#include <math.h>
#include <Eigen/Eigen>

#define EARTH_RADIUS 6378145.0f

namespace hector
{
class estimator_base_99pub
{
public:
  estimator_base_99pub();

protected:

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher vehicle_state99_pub_;
  ros::Subscriber true_state_sub_;

  void update(const ros::TimerEvent &);
  void truestateCallback(const hector_msgs::StateConstPtr &msg);
  
  double update_rate_;
  ros::Timer update_timer_;

  std::string truth_topic_;

  bool state_init_;
  hector_msgs::State true_state_;

};

} //end namespace

#endif // 
