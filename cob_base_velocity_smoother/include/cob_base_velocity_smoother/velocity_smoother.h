/**
 * @brief Velocity smoother interface
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

 // Modified by Benjamin Maidel: benjamin.maidel@ipa.fraunhofer.de
 //         and Felix Messmer:   felix.messmer@ipa.fraunhofer.de            

#ifndef VELOCITY_SMOOTHER_HPP_
#define VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cob_base_velocity_smoother/paramsConfig.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread/mutex.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cob_base_velocity_smoother {

/*****************************************************************************
** VelocitySmoother
*****************************************************************************/

class VelocitySmoother
{
public:
  VelocitySmoother(const std::string &name);

  ~VelocitySmoother()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

  bool init(ros::NodeHandle& nh);
  void spin(const ros::TimerEvent&);

private:
  std::string name;
  double speed_lim_vx, accel_lim_vx, decel_lim_vx;
  double speed_lim_vy, accel_lim_vy, decel_lim_vy;
  double speed_lim_w, accel_lim_w, decel_lim_w;
  double decel_factor, safe_factor;

  double frequency;

  geometry_msgs::Twist last_cmd_vel;
  geometry_msgs::Twist target_vel;

  bool                 input_active;
  double               cb_avg_time;
  ros::Time            last_cb_time;
  std::vector<double>  period_record; /**< Historic of latest periods between velocity commands */
  unsigned int         pr_next;       /**< Next position to fill in the periods record buffer */

  ros::Timer timer;

  ros::Subscriber raw_in_vel_sub;  /**< Incoming raw velocity commands */
  ros::Publisher  smooth_vel_pub;  /**< Outgoing smoothed velocity commands */
  
  boost::mutex _mutex;

  void velocityCB(const geometry_msgs::Twist::ConstPtr& msg);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double> values) {
    // Return the median element of an doubles vector
    nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  };

  double calculate_incs_per_sec(double target, double last_cmd, double acc_lim);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<cob_base_velocity_smoother::paramsConfig> *             dynamic_reconfigure_server;
  dynamic_reconfigure::Server<cob_base_velocity_smoother::paramsConfig>::CallbackType dynamic_reconfigure_callback;
  void reconfigCB(cob_base_velocity_smoother::paramsConfig &config, uint32_t unused_level);
};

} // cob_base_velocity_smoother

#endif /* VELOCITY_SMOOTHER_HPP_ */
