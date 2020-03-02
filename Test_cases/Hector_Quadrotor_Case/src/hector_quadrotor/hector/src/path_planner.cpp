#include <ros/ros.h>
#include <hector_msgs/Waypoint.h>
#define EARTH_RADIUS 6378145.0f
#define num_waypoints 3
float pn2latitude(float pn,float init_lat_)
{
  return (pn*180)/(EARTH_RADIUS*M_PI)+init_lat_;
}

float pe2longtitude(float pe,float init_lat_,float init_lon_)
{
  return (pe*180)/(EARTH_RADIUS*cos(init_lat_*M_PI/180.0)*M_PI)+init_lon_;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_simple_path_planner");

  ros::NodeHandle nh_;
  ros::Publisher waypointPublisher = nh_.advertise<hector_msgs::Waypoint>("waypoint_path", 10);

  float Va = 3;
  int tmp;
  tmp = 7; //notice here
  float wps[tmp * num_waypoints] =
      {
          // pn,pe,altitude,-,va,lat,lon
          // 20, 10,          -10, 0, 3, pn2latitude(20,0),       pe2longtitude(0,0,0),
          // 30, 20,         -10, 0, Va,  pn2latitude(30,0),       pe2longtitude(30,0,0),
          // 0,   20,        -12, 0, 5,  pn2latitude(0,0),         pe2longtitude(20,0,0),
          0, 10,          -10, 0, 3, pn2latitude(20,0),       pe2longtitude(0,0,0),
      };

  for (int i(0); i < num_waypoints; i++)
  {
    ros::Duration(0.5).sleep();

    hector_msgs::Waypoint new_waypoint;

    new_waypoint.w[0] = wps[i * tmp + 0];
    new_waypoint.w[1] = wps[i * tmp + 1];
    new_waypoint.w[2] = wps[i * tmp + 2];
    new_waypoint.chi_d = wps[i * tmp + 3];
    new_waypoint.Va_d = wps[i * tmp + 4];
    // add by kobe
    new_waypoint.lat = wps[i * tmp + 5];
    new_waypoint.lon = wps[i * tmp + 6];

    new_waypoint.chi_valid = false; //kobe

    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;

    new_waypoint.clear_wp_list = false;

    waypointPublisher.publish(new_waypoint);
  }
  ros::Duration(1.5).sleep();

  return 0;
}
