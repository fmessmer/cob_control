/**
 * @brief Velocity smoother implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <cob_base_velocity_smoother/velocity_smoother.h>

#include <boost/thread.hpp>

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define MIN_EXPECTED_RATE     0.02 // 50Hz
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)    ((a.linear.x == 0.0) && (a.linear.y == 0.0) && (a.angular.z == 0.0))

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cob_base_velocity_smoother {

/*********************
** Implementation
**********************/

VelocitySmoother::VelocitySmoother(const std::string &name)
: name(name)
, input_active(false)
, pr_next(0)
, dynamic_reconfigure_server(NULL)
{}

void VelocitySmoother::reconfigCB(cob_base_velocity_smoother::paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : \n\tSpeedLimit: %f %f %f\n\tAccelLimit: %f %f %f\n\tDecelFactors: %f %f",
            config.speed_lim_vx, config.speed_lim_vy, config.speed_lim_w, config.accel_lim_vx, config.accel_lim_vy, config.accel_lim_w, config.decel_factor, config.safe_factor);

  speed_lim_vx  = config.speed_lim_vx;
  speed_lim_vy  = config.speed_lim_vy;
  speed_lim_w  = config.speed_lim_w;
  accel_lim_vx  = config.accel_lim_vx;
  accel_lim_vy  = config.accel_lim_vy;
  accel_lim_w  = config.accel_lim_w;
  decel_factor = config.decel_factor;
  safe_factor = config.safe_factor;
}

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(_mutex);

  ros::Time now = ros::Time::now();
  double period = (now - last_cb_time).toSec();
  last_cb_time = now;

  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    period_record.push_back(period);
  }
  else
  {
    period_record[pr_next] = period;
  }

  pr_next++;
  pr_next %= period_record.size();

  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    // wait until we have some values; make a reasonable assumption meanwhile
    cb_avg_time = 1.0/frequency;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  // Bound speed with the maximum values
  target_vel.linear.x  =
      msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_vx) : std::max(msg->linear.x,  -speed_lim_vx);
  target_vel.linear.y  =
      msg->linear.y  > 0.0 ? std::min(msg->linear.y,  speed_lim_vy) : std::max(msg->linear.y,  -speed_lim_vy);
  target_vel.angular.z =
      msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w) : std::max(msg->angular.z, -speed_lim_w);

  input_active = true;
}

double VelocitySmoother::calculate_incs_per_sec(double target, double last_cmd, double acc_lim)
{
  double inc, inc_req, acc, max_acc;
  double period, dec_lim;

  if (input_active)
  {
    period = cb_avg_time;
    dec_lim = decel_factor*acc_lim;
  }
  else
  {
    period = 1.0/frequency;
    acc_lim = safe_factor*acc_lim;
    dec_lim = decel_factor*acc_lim;
  }

  inc = target - last_cmd;
  acc = inc/period;
  max_acc = (inc > 0.0) ? acc_lim : dec_lim;
  inc_req = (std::abs(acc)>max_acc) ? sign(acc)*max_acc : acc;  // inc per second
  return inc_req;
}

void VelocitySmoother::spin(const ros::TimerEvent& event)
{
  boost::mutex::scoped_lock lock(_mutex);

  //double period = (event.current_real - event.last_real).toSec();
  double period = 1.0/frequency;
  double last_command = (event.current_real - last_cb_time).toSec();

  if ((input_active == true) && (cb_avg_time > 0.0) &&
      (last_command > PERIOD_RECORD_SIZE*std::min(cb_avg_time, 1.0/frequency)))
  {
    // Velocity input not active anymore; normally last command is a zero-velocity one, but reassure
    // this, just in case something went wrong with our input, or he just forgot good manners...
    // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
    // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
    // several messages arrive with the same time and so lead to a zero median
    ROS_WARN_STREAM("Velocity Smoother : input got inactive!");
    input_active = false;
    if (IS_ZERO_VEOCITY(target_vel) == false)
    {
      ROS_WARN_STREAM("Velocity Smoother : leaving us a non-zero target velocity ("
            << target_vel.linear.x << ", " << target_vel.linear.y << ", " << target_vel.angular.z << ")...zeroing...[" << name << "]");
      target_vel = ZERO_VEL_COMMAND;
    }
  }

  //not yet reached target_vel
  if (input_active || IS_ZERO_VEOCITY(last_cmd_vel) == false)
  {
    //determine required inc
    double vx_inc_req, vy_inc_req, w_inc_req;
    vx_inc_req = calculate_incs_per_sec(target_vel.linear.x, last_cmd_vel.linear.x, accel_lim_vx);
    vy_inc_req = calculate_incs_per_sec(target_vel.linear.y, last_cmd_vel.linear.y, accel_lim_vy);
    w_inc_req = calculate_incs_per_sec(target_vel.angular.z, last_cmd_vel.angular.z, accel_lim_w);

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x  = last_cmd_vel.linear.x  + vx_inc_req*period;
    cmd_vel.linear.y  = last_cmd_vel.linear.y  + vy_inc_req*period;
    cmd_vel.angular.z  = last_cmd_vel.angular.z  + w_inc_req*period;

    smooth_vel_pub.publish(cmd_vel);
    last_cmd_vel = cmd_vel;
  }
}

/**
 * Initialise from private NodeHandle.
 * @param nh : private NodeHandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh)
{
  // Dynamic Reconfigure
  dynamic_reconfigure_callback = boost::bind(&VelocitySmoother::reconfigCB, this, _1, _2);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<cob_base_velocity_smoother::paramsConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

  nh.param("frequency",      frequency,     50.0);

  // Publishers and subscribers
  raw_in_vel_sub  = nh.subscribe("raw_cmd_vel",   1, &VelocitySmoother::velocityCB, this);
  smooth_vel_pub  = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

  timer = nh.createTimer(ros::Duration(1.0/frequency), &VelocitySmoother::spin, this);

  return true;
}
}
