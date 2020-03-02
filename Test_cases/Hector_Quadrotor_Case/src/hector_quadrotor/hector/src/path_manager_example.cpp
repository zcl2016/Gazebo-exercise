#include "path_manager_example.h"
#include "ros/ros.h"
#include <cmath>

namespace hector
{

path_manager_example::path_manager_example() : path_manager_base()
{
  fil_state_ = fillet_state::STRAIGHT;
  // use this !
  dub_state_ = dubin_state::FIRST;
}
// done
void path_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
{

  //ROS_INFO("num_waypoints_: %d",num_waypoints_);
  if (num_waypoints_ < 2)
  {
    ROS_WARN_THROTTLE(4, "No waypoints received! Loitering about origin at 50m");
    output.flag = 0; //modified by kobe
    output.Va_d = 12;
    output.c[0] = 0.0f;
    output.c[1] = 0.0f;
    output.c[2] = -50.0f;
    // Radius of orbital path (m)
    output.rho = params.R_min;
    // Direction of orbital path (clockwise is 1, counterclockwise is -1)
    output.lambda = 1;
  }
  else
  {
    // do this, idx_a_ is initialized zero
    if (waypoints_[idx_a_].chi_valid)
    {
      manage_dubins(params, input, output);
    }
    else
    {
      /** Switch the following for flying directly to waypoints, or filleting corners */
      manage_line(params, input, output);
      //manage_fillet(params, input, output);
      // manage_star(params, input, output); //add by kobe
    }
  }
}

// add by kobe
void path_manager_example::manage_star(const params_s &params, const input_s &input, output_s &output)
{
  Eigen::Vector3f w_start;
  w_start << input.pn, input.pe, -input.h; //aircraft's position

  int idx_b;
  int idx_c;
  float the_distance_;
  double the_distance_titude; //for latitude and longtitude
  idx_b = idx_a_ + 1;

  if (idx_a_ < num_waypoints_ - 2)
  {
    idx_c = idx_b + 1;
  }
  else
  {
    idx_c = idx_b;
  }
  Eigen::Vector3f w_ip1(waypoints_[idx_c].w);

  Eigen::Vector3f w_im1(waypoints_[idx_a_].w);
  Eigen::Vector3f w_i(waypoints_[idx_b].w); //the goal

  the_distance_ = sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2));
  the_distance_titude = earth_distance(input.lat, input.lon, waypoints_[idx_b].lat, waypoints_[idx_b].lon);

  output.flag = 1;                      //modified by kobe
  output.Va_d = waypoints_[idx_b].Va_d; //modified by kobe

  //ROS_INFO("position:  %g %g %g",w_start(0),w_start(1),w_start(2));
  //ROS_INFO("Current info are:  %f %f %f %f", input.va, -input.h,w_start(0),w_start(1));

  output.r[0] = w_start(0);
  output.r[1] = w_start(1);
  output.r[2] = w_start(2);
  Eigen::Vector3f q_im1 = (w_i - w_start).normalized();
  output.q[0] = q_im1(0);
  output.q[1] = q_im1(1);
  output.q[2] = q_im1(2);

  output.gxy[0] = w_i(0);
  output.gxy[1] = w_i(1);
  output.gll[0] = waypoints_[idx_b].lat;
  output.gll[1] = waypoints_[idx_b].lon;
  output.h_c = w_i(2); //altitude of the goal

  if (idx_a_ == 0&&show2 == 1)
  {
    ROS_INFO("First goal info are:               %f %f", w_i(0), w_i(1));
    ROS_INFO("Desired fly speed and height are:  %f %f", output.Va_d, -w_i(2));
    show2=0;
  }
  //enter the slow-down area,recompute the speed
  if (sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2)) < output.Va_d * 2 && idx_a_ == num_waypoints_ - 2) //enter the slowdown-ball and the goal is the last goal
  {
    output.Va_c = sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2))/2;
    if (sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2)) < 0.5 )
    {
      output.Va_c = 0;
    }
    // ROS_INFO("Now the speed is: %f",output.Va_c);
  }
  else
  {
    output.Va_c = output.Va_d;
  }
  output.landing=waypoints_[idx_b].landing;
  output.takeoff=waypoints_[idx_b].takeoff;
  /* //1.enter a tiny ball and switch goal
  if (sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2)) < 5) //enter a tiny ball and switch goal
  {
    //ROS_INFO("00000o- %g %g %g", w_start(2), w_i(2), sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2)));
    //ROS_INFO("last goal is: %f %f", w_i(0), w_i(1));
    //ROS_INFO("Va- %f/n", input.va);
    if (idx_a_ == num_waypoints_ - 2)
    {
      idx_a_ = idx_a_; //do not update goal
      //ROS_INFO("idx_a_- %f/n", idx_a_);
    }
    else
    {
      idx_a_++;
    } 
  }
  */
  /*
  //2.enter a half-plane and switch,ahead 5 m from the goal point
  Eigen::Vector3f w_i_plane;
  w_i_plane=w_i;
  w_i_plane(0)=w_i_plane(0)-5*(  ( w_i(0)-w_im1(0) ) / ( 0.001+fabs(w_i(0)-w_im1(0)) )  );
  w_i_plane(1)=w_i_plane(1)-5*(  ( w_i(1)-w_im1(1) ) / ( 0.001+fabs(w_i(1)-w_im1(1)) )  );
  if ((w_i_plane - w_start).normalized().dot((w_i_plane - w_im1).normalized())<0)//plane-method
  {
    if (idx_a_ == num_waypoints_ - 2)
      idx_a_ = idx_a_;//do not update goal
    else
      idx_a_++;
  }
  */
  //3.Enter a large ball(large-enter-ball) first. Anyway, when the distance from the aircraft to goal become larger,switch to the next goal.
  if (the_distance_ < min_distance_)
  {
    min_distance_ = the_distance_;
  }
  if (the_distance_titude < min_distance_titude)
  {
    min_distance_titude = the_distance_titude;
  }

  // ROS_INFO("distance is :  %.20f %f",  earth_distance(input.lat, input.lon, waypoints_[idx_b].lat, waypoints_[idx_b].lon), sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2)));

  //ROS_INFO("000distance- %f %f", the_distance_, min_distance_);
  //1.进入大圈且最小距离开始增大时;2.进入一个小圈时�?分别针对北东坐标和经纬度坐标
  judge_condition1 = (sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2)) < 3 && the_distance_ > min_distance_ + 1);
  judge_condition2 = (earth_distance(input.lat, input.lon, waypoints_[idx_b].lat, waypoints_[idx_b].lon) < 3 && the_distance_titude > min_distance_titude + 1);
  judge_condition3 = (sqrt(pow(w_start(0) - w_i(0), 2) + pow(w_start(1) - w_i(1), 2)) < 0.5);
  judge_condition4 = (earth_distance(input.lat, input.lon, waypoints_[idx_b].lat, waypoints_[idx_b].lon) < 0.5);
  // height judge
  judge_condition5 = (sqrt(pow(w_start(2) - w_i(2), 2)) < 10);   
  
  //notice here
  judge_condition2=0;
  judge_condition4=0;

  if ((judge_condition1 || judge_condition2 || judge_condition3 || judge_condition4) && judge_condition5) //switch
  {
    //ROS_INFO("distance- %f %f", the_distance_, min_distance_);
    if (idx_a_ == num_waypoints_ - 2)
    {
      if (show == 1)
      { 
        ROS_INFO("This goal is reached:          %f %f", w_i(0), w_i(1));
        ROS_INFO("Current position are:          %f %f", input.pn, input.pe);
        ROS_INFO("No more goal, just hover and wait for the next goal.");
        ROS_INFO("Desired speed and height are:  %f %f", output.Va_d, -w_i(2));
        ROS_INFO("Current speed and height are:  %f %f\n\n", input.va, input.h);
        show = 0;
      }
      idx_a_ = idx_a_; //do not update goal
    }
    else
    {
      ROS_INFO("This goal is reached:          %f %f", w_i(0), w_i(1));
      ROS_INFO("Current position are:          %f %f", input.pn, input.pe);
      ROS_INFO("Desired speed and height are:  %f %f", output.Va_d, -w_i(2));
      ROS_INFO("Current speed and height are:  %f %f", input.va, input.h);
      ROS_INFO("New goal info are:             %f %f\n\n", w_ip1(0), w_ip1(1));
      min_distance_ = sqrt(pow(w_i(0) - w_ip1(0), 2) + pow(w_i(1) - w_ip1(1), 2));
      idx_a_++;
    }
  }
  //
}

double path_manager_example::earth_distance(double latitude1, double longitude1, double latitude2, double longitude2)
{
  double Lat1 = latitude1 * M_PI_F / 180.00; // 纬度
  double Lat2 = latitude2 * M_PI_F / 180.00;
  double a = Lat1 - Lat2;                                                                     //两点纬度之差
  double b = longitude1 * M_PI_F / 180.00 - longitude2 * M_PI_F / 180.00;                     //经度之差
  double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(Lat1) * cos(Lat2) * pow(sin(b / 2), 2))); //计算两点距离的公�?/better when value is very samll
  //double s2 = acos(cos(Lat1)*cos(Lat2)*cos(b)+sin(Lat1)*sin(Lat2));//计算两点距离的公�?
  s = s * EARTH_RADIUS; //弧长乘地球半径（半径为米�?
  //s2 = s2 * EARTH_RADIUS;//弧长乘地球半径（半径为米�?
  return s;
}

void path_manager_example::manage_line(const params_s &params, const input_s &input, output_s &output)
{
  Eigen::Vector3f p;
  p << input.pn, input.pe, -input.h;

  int idx_b;
  int idx_c;
  if (idx_a_ == num_waypoints_ - 1)
  {
    idx_b = 0;
    idx_c = 1;
  }
  else if (idx_a_ == num_waypoints_ - 2)
  {
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    idx_b = idx_a_ + 1;
    idx_c = idx_b + 1;
  }

  Eigen::Vector3f w_im1(waypoints_[idx_a_].w);
  Eigen::Vector3f w_i(waypoints_[idx_b].w);
  Eigen::Vector3f w_ip1(waypoints_[idx_c].w);

  output.flag = true;
  output.Va_d = waypoints_[idx_a_].Va_d;
  output.r[0] = w_im1(0);
  output.r[1] = w_im1(1);
  output.r[2] = w_im1(2);
  Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
  Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
  output.q[0] = q_im1(0);
  output.q[1] = q_im1(1);
  output.q[2] = q_im1(2);

  Eigen::Vector3f n_i = (q_im1 + q_i).normalized();
  if ((p - w_i).dot(n_i) > 0.0f)
  {
    if (idx_a_ == num_waypoints_ - 1)
      idx_a_ = 0;
    else
      idx_a_++;
  }
}
void path_manager_example::manage_fillet(const params_s &params, const input_s &input, output_s &output)
{
  if (num_waypoints_ < 3) //since it fillets don't make sense between just two points
  {
    manage_line(params, input, output);
    return;
  }

  Eigen::Vector3f p;
  p << input.pn, input.pe, -input.h;

  int idx_b;
  int idx_c;
  if (idx_a_ == num_waypoints_ - 1)
  {
    idx_b = 0;
    idx_c = 1;
  }
  else if (idx_a_ == num_waypoints_ - 2)
  {
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    idx_b = idx_a_ + 1;
    idx_c = idx_b + 1;
  }

  Eigen::Vector3f w_im1(waypoints_[idx_a_].w);
  Eigen::Vector3f w_i(waypoints_[idx_b].w);
  Eigen::Vector3f w_ip1(waypoints_[idx_c].w);

  float R_min = params.R_min;

  output.Va_d = waypoints_[idx_a_].Va_d;
  output.r[0] = w_im1(0);
  output.r[1] = w_im1(1);
  output.r[2] = w_im1(2);
  Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
  Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
  float beta = acosf(-q_im1.dot(q_i));

  Eigen::Vector3f z;
  switch (fil_state_)
  {
  case fillet_state::STRAIGHT:
    output.flag = 1; //modified by kobe
    output.q[0] = q_im1(0);
    output.q[1] = q_im1(1);
    output.q[2] = q_im1(2);
    output.c[0] = 1;
    output.c[1] = 1;
    output.c[2] = 1;
    output.rho = 1;
    output.lambda = 1;
    z = w_i - q_im1 * (R_min / tanf(beta / 2.0));
    if ((p - z).dot(q_im1) > 0)
      fil_state_ = fillet_state::ORBIT;
    break;
  case fillet_state::ORBIT:
    output.flag = 0; //modified by kobe
    output.q[0] = q_i(0);
    output.q[1] = q_i(1);
    output.q[2] = q_i(2);
    Eigen::Vector3f c = w_i - (q_im1 - q_i).normalized() * (R_min / sinf(beta / 2.0));
    output.c[0] = c(0);
    output.c[1] = c(1);
    output.c[2] = c(2);
    output.rho = R_min;
    output.lambda = ((q_im1(0) * q_i(1) - q_im1(1) * q_i(0)) > 0 ? 1 : -1);
    z = w_i + q_i * (R_min / tanf(beta / 2.0));
    if ((p - z).dot(q_i) > 0)
    {
      if (idx_a_ == num_waypoints_ - 1)
        idx_a_ = 0;
      else
        idx_a_++;
      fil_state_ = fillet_state::STRAIGHT;
    }
    break;
  }
}

void path_manager_example::manage_dubins(const params_s &params, const input_s &input, output_s &output)
{
  Eigen::Vector3f p;
  //ROS_INFO("input.pn: %f, input.pe: %f, -input.h: %f",input.pn, input.pe, -input.h);
  p << input.pn, input.pe, -input.h;

  /*  uint8 path_type		# Indicates strait line or orbital path
      float32 Va_d		# Desired airspeed (m/s)
      float32[3] r		# Vector to origin of straight line path (m)
      float32[3] q		# Unit vector, desired direction of travel for line path
      float32[3] c		# Center of orbital path (m)
      float32 rho		# Radius of orbital path (m)
      int8 lambda		# Direction of orbital path (clockwise is 1, counterclockwise is -1)

      uint8 ORBIT_PATH = 0
      uint8 LINE_PATH = 1

      int8 CLOCKWISE = 1
      int8 COUNT_CLOCKWISE = -1 */

  output.Va_d = waypoints_[idx_a_].Va_d;
  output.r[0] = 0;
  output.r[1] = 0;
  output.r[2] = 0;
  output.q[0] = 0;
  output.q[1] = 0;
  output.q[2] = 0;
  output.c[0] = 0;
  output.c[1] = 0;
  output.c[2] = 0;
  //ROS_INFO("dub_state_: %d",dub_state_);
  switch (dub_state_)
  {
  case dubin_state::FIRST:
    // path planning
    // Algorithm11 P211
    dubinsParameters(waypoints_[0], waypoints_[1], params.R_min);

    waypoint_start_ = waypoints_[0];
    waypoint_end_ = waypoints_[1];

    output.flag = 0; //modified by kobe
    output.c[0] = dubinspath_.cs(0);
    output.c[1] = dubinspath_.cs(1);
    output.c[2] = dubinspath_.cs(2);
    output.rho = dubinspath_.R;
    output.lambda = dubinspath_.lams;
    // p: current pos
    // w1: waypoint 1
    // q1: unit vector of H1
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
    {
      dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
    }
    else
    {
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  case dubin_state::BEFORE_H1:
    //waypoint_received_ = false;
    output.flag = 0; //modified by kobe
    output.c[0] = dubinspath_.cs(0);
    output.c[1] = dubinspath_.cs(1);
    output.c[2] = dubinspath_.cs(2);
    output.rho = dubinspath_.R;
    output.lambda = dubinspath_.lams;
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // entering H1
    {
      dub_state_ = dubin_state::STRAIGHT;
    }
    break;
  case dubin_state::BEFORE_H1_WRONG_SIDE:
    //waypoint_received_ = false;
    output.flag = 0; //modified by kobe
    output.c[0] = dubinspath_.cs(0);
    output.c[1] = dubinspath_.cs(1);
    output.c[2] = dubinspath_.cs(2);
    output.rho = dubinspath_.R;
    output.lambda = dubinspath_.lams;
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) < 0) // exit H1
    {
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  case dubin_state::STRAIGHT:
    output.flag = 1; //modified by kobe
    output.r[0] = dubinspath_.w1(0);
    output.r[1] = dubinspath_.w1(1);
    output.r[2] = dubinspath_.w1(2);
    // output.r[0] = dubinspath_.z1(0);
    // output.r[1] = dubinspath_.z1(1);
    // output.r[2] = dubinspath_.z1(2);
    output.q[0] = dubinspath_.q1(0);
    output.q[1] = dubinspath_.q1(1);
    output.q[2] = dubinspath_.q1(2);
    output.rho = 1;
    output.lambda = 1;
    //ROS_INFO("pos: [%f,%f,%f],w2: [%f,%f,%f],q1: [%f,%f,%f]",p[0],p[1],p[2],dubinspath_.w2[0],dubinspath_.w2[1],dubinspath_.w2[2],dubinspath_.q1[0],dubinspath_.q1[1],dubinspath_.q1[2]);
    //ROS_INFO("w3: [%f,%f,%f]",dubinspath_.w3[0],dubinspath_.w3[1],dubinspath_.w3[2]);
    //ROS_INFO("w1: [%f,%f,%f]",dubinspath_.w1[0],dubinspath_.w1[1],dubinspath_.w1[2]);

    if (waypoint_received_)
    {
      //start new path
      // plan new Dubin's path to next waypoint configuration
      waypoint_start_.w[0] = p[0];
      waypoint_start_.w[1] = p[1];
      waypoint_start_.w[2] = p[2];
      waypoint_start_.chi_d = 0;
      waypoint_start_.chi_valid = true;
      waypoint_start_.Va_d = 12;
      dubinsParameters(waypoint_start_, waypoint_end_, params.R_min);
      waypoint_received_ = false;

      if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
      {
        dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
      }
      else
      {
        dub_state_ = dubin_state::BEFORE_H1;
      }
    }
    else
    {
      if ((p - dubinspath_.w2).dot(dubinspath_.q1) >= 0) // entering H2
      {
        if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // start in H3
        {
          dub_state_ = dubin_state::BEFORE_H3_WRONG_SIDE;
        }
        else
        {
          dub_state_ = dubin_state::BEFORE_H3;
        }
      }
    }
    break;
  case dubin_state::BEFORE_H3:
    output.flag = 0; //modified by kobe
    output.c[0] = dubinspath_.ce(0);
    output.c[1] = dubinspath_.ce(1);
    output.c[2] = dubinspath_.ce(2);
    output.rho = dubinspath_.R;
    output.lambda = dubinspath_.lame;
    if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // entering H3
    {
      if (!waypoint_received_)
      {
        if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
        {
          //dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
          dub_state_ = dubin_state::BEFORE_H3_WRONG_SIDE;
        }
        else
        {
          //dub_state_ = dubin_state::BEFORE_H1;
          dub_state_ = dubin_state::BEFORE_H3;
        }
      }
      else
      {
        //start new path
        // plan new Dubin's path to next waypoint configuration
        dubinsParameters(waypoint_start_, waypoint_end_, params.R_min);
        waypoint_received_ = false;

        if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
        {
          dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
        }
        else
        {
          dub_state_ = dubin_state::BEFORE_H1;
        }
      }
    }
    break;
  case dubin_state::BEFORE_H3_WRONG_SIDE:
    output.flag = 0; //modified by kobe
    output.c[0] = dubinspath_.ce(0);
    output.c[1] = dubinspath_.ce(1);
    output.c[2] = dubinspath_.ce(2);
    output.rho = dubinspath_.R;
    output.lambda = dubinspath_.lame;
    /*    
    if ((p - dubinspath_.w3).dot(dubinspath_.q3) < 0) // exit H3
    {
      if(!waypoint_received_)
        //dub_state_ = dubin_state::BEFORE_H1;
        dub_state_ = dubin_state::BEFORE_H3;
      else
        dub_state_ = dubin_state::BEFORE_H1;
    }
*/
    if ((p - dubinspath_.w3).dot(dubinspath_.q3) < 0) // exit H3
      dub_state_ = dubin_state::BEFORE_H3;
    else
      dub_state_ = dubin_state::BEFORE_H3_WRONG_SIDE;

    break;
  }
}

Eigen::Matrix3f path_manager_example::rotz(float theta)
{
  Eigen::Matrix3f R;
  R << cosf(theta), -sinf(theta), 0,
      sinf(theta), cosf(theta), 0,
      0, 0, 1;

  return R;
}
// done
float path_manager_example::mo(float in)
{
  float val;
  if (in > 0)
    val = fmod(in, 2.0 * M_PI_F);
  else
  {
    float n = floorf(in / 2.0 / M_PI_F);
    val = in - n * 2.0 * M_PI_F;
  }
  return val;
}

void path_manager_example::dubinsParameters(const waypoint_s start_node, const waypoint_s end_node, float R)
{
  float ell = sqrtf((start_node.w[0] - end_node.w[0]) * (start_node.w[0] - end_node.w[0]) +
                    (start_node.w[1] - end_node.w[1]) * (start_node.w[1] - end_node.w[1]));
  if (ell < 2.0 * R)
  {
    ROS_ERROR("SHOULD NOT HAPPEN: The distance between nodes must be larger than 2R.");
  }
  else
  {
    dubinspath_.ps(0) = start_node.w[0];
    dubinspath_.ps(1) = start_node.w[1];
    dubinspath_.ps(2) = start_node.w[2];
    dubinspath_.chis = start_node.chi_d;
    dubinspath_.pe(0) = end_node.w[0];
    dubinspath_.pe(1) = end_node.w[1];
    dubinspath_.pe(2) = end_node.w[2];
    dubinspath_.chie = end_node.chi_d;

    Eigen::Vector3f crs = dubinspath_.ps;
    crs(0) += R * (cosf(M_PI_2_F) * cosf(dubinspath_.chis) - sinf(M_PI_2_F) * sinf(dubinspath_.chis));
    crs(1) += R * (sinf(M_PI_2_F) * cosf(dubinspath_.chis) + cosf(M_PI_2_F) * sinf(dubinspath_.chis));
    Eigen::Vector3f cls = dubinspath_.ps;
    cls(0) += R * (cosf(-M_PI_2_F) * cosf(dubinspath_.chis) - sinf(-M_PI_2_F) * sinf(dubinspath_.chis));
    cls(1) += R * (sinf(-M_PI_2_F) * cosf(dubinspath_.chis) + cosf(-M_PI_2_F) * sinf(dubinspath_.chis));
    Eigen::Vector3f cre = dubinspath_.pe;
    cre(0) += R * (cosf(M_PI_2_F) * cosf(dubinspath_.chie) - sinf(M_PI_2_F) * sinf(dubinspath_.chie));
    cre(1) += R * (sinf(M_PI_2_F) * cosf(dubinspath_.chie) + cosf(M_PI_2_F) * sinf(dubinspath_.chie));
    Eigen::Vector3f cle = dubinspath_.pe;
    cle(0) += R * (cosf(-M_PI_2_F) * cosf(dubinspath_.chie) - sinf(-M_PI_2_F) * sinf(dubinspath_.chie));
    cle(1) += R * (sinf(-M_PI_2_F) * cosf(dubinspath_.chie) + cosf(-M_PI_2_F) * sinf(dubinspath_.chie));

    float theta, theta2;
    // compute L1
    theta = atan2f(cre(1) - crs(1), cre(0) - crs(0));
    float L1 = (crs - cre).norm() + R * mo(2.0 * M_PI_F + mo(theta - M_PI_2_F) - mo(dubinspath_.chis - M_PI_2_F)) + R * mo(2.0 * M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta - M_PI_2_F));

    // compute L2
    ell = (cle - crs).norm();
    theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
    float L2;
    if (2.0 * R > ell)
      L2 = 9999.0f;
    else
    {
      theta2 = theta - M_PI_2_F + asinf(2.0 * R / ell);
      L2 = sqrtf(ell * ell - 4.0 * R * R) + R * mo(2.0 * M_PI_F + mo(theta2) - mo(dubinspath_.chis - M_PI_2_F)) + R * mo(2.0 * M_PI_F + mo(theta2 + M_PI_F) - mo(dubinspath_.chie + M_PI_2_F));
    }

    // compute L3
    ell = (cre - cls).norm();
    theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
    float L3;
    if (2.0 * R > ell)
      L3 = 9999.0f;
    else
    {
      theta2 = acosf(2.0 * R / ell);
      L3 = sqrtf(ell * ell - 4 * R * R) + R * mo(2.0 * M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + theta2)) + R * mo(2.0 * M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta + theta2 - M_PI_F));
    }

    // compute L4
    theta = atan2f(cle(1) - cls(1), cle(0) - cls(0));
    float L4 = (cls - cle).norm() + R * mo(2.0 * M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + M_PI_2_F)) + R * mo(2.0 * M_PI_F + mo(theta + M_PI_2_F) - mo(dubinspath_.chie + M_PI_2_F));

    // L is the minimum distance
    int idx = 1;
    dubinspath_.L = L1;
    if (L2 < dubinspath_.L)
    {
      dubinspath_.L = L2;
      idx = 2;
    }
    if (L3 < dubinspath_.L)
    {
      dubinspath_.L = L3;
      idx = 3;
    }
    if (L4 < dubinspath_.L)
    {
      dubinspath_.L = L4;
      idx = 4;
    }

    Eigen::Vector3f e1;
    //        e1.zero();
    e1(0) = 1;
    e1(1) = 0;
    e1(2) = 0;
    switch (idx)
    {
    case 1:
      dubinspath_.cs = crs;
      dubinspath_.lams = 1;
      dubinspath_.ce = cre;
      dubinspath_.lame = 1;
      dubinspath_.q1 = (cre - crs).normalized();
      dubinspath_.w1 = dubinspath_.cs + (rotz(-M_PI_2_F) * dubinspath_.q1) * R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(-M_PI_2_F) * dubinspath_.q1) * R;
      break;
    case 2:
      dubinspath_.cs = crs;
      dubinspath_.lams = 1;
      dubinspath_.ce = cle;
      dubinspath_.lame = -1;
      ell = (cle - crs).norm();
      theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
      theta2 = theta - M_PI_2_F + asinf(2.0 * R / ell);
      dubinspath_.q1 = rotz(theta2 + M_PI_2_F) * e1;
      dubinspath_.w1 = dubinspath_.cs + (rotz(theta2) * e1) * R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(theta2 + M_PI_F) * e1) * R;
      break;
    case 3:
      dubinspath_.cs = cls;
      dubinspath_.lams = -1;
      dubinspath_.ce = cre;
      dubinspath_.lame = 1;
      ell = (cre - cls).norm();
      theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
      theta2 = acosf(2.0 * R / ell);
      dubinspath_.q1 = rotz(theta + theta2 - M_PI_2_F) * e1;
      dubinspath_.w1 = dubinspath_.cs + (rotz(theta + theta2) * e1) * R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(theta + theta2 - M_PI_F) * e1) * R;
      break;
    case 4:
      dubinspath_.cs = cls;
      dubinspath_.lams = -1;
      dubinspath_.ce = cle;
      dubinspath_.lame = -1;
      dubinspath_.q1 = (cle - cls).normalized();
      dubinspath_.w1 = dubinspath_.cs + (rotz(M_PI_2_F) * dubinspath_.q1) * R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(M_PI_2_F) * dubinspath_.q1) * R;
      break;
    }
    dubinspath_.w3 = dubinspath_.pe;
    dubinspath_.q3 = rotz(dubinspath_.chie) * e1;
    dubinspath_.R = R;
  }
}

} // namespace hector
