// modified by kobe

#ifndef ESTIMATOR_BASE_UPDATA_SUB_H
#define ESTIMATOR_BASE_UPDATA_SUB_H

#include <ros/ros.h>
#include <hector_msgs/State.h>
#include <hector_msgs/Up_Data_New.h> 

#include <math.h>
#include <Eigen/Eigen>

#define EARTH_RADIUS 6378145.0f

namespace hector
{
class estimator_base_updata_statesub
{
public:
  estimator_base_updata_statesub();

protected:

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher vehicle_state_pub_;
  ros::Subscriber state29_sub_;
  void update(const ros::TimerEvent &);
  void state99Callback(const hector_msgs::Up_Data_NewConstPtr &msg);

  double update_rate_;
  ros::Timer update_timer_;
  std::string state29_topic_;

  double init_lat_;       /**< Initial latitude in degrees */
  double init_lon_;       /**< Initial longitude in degrees */
  float init_alt_;        /**< Initial altitude in meters above MSL  */

  bool state_init_;
  hector_msgs::Up_Data_New state29_;
};

} //end namespace

#endif //
