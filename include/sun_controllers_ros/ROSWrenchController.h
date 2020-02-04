/*
    ROS Wrench Controller
    Copyright 2020 Università della Campania Luigi Vanvitelli
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SUN_ROS_WRENCH_CONTROLLER
#define SUN_ROS_WRENCH_CONTROLLER

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#define NUM_ZERO_VEL_COMMANDS_TO_SEND 20
#define MEAN_NUM_SAMPLES 50
#define DEFAULT_GAINS 0.0

namespace sun {
class ROSWrenchController {

protected:
  // Protected members

  // true if a new wrench measure arrived
  bool wrench_measure_arrived_;

  // true if the controller is enabled_
  bool enabled_;

  // Wrench Controller Gains
  // [fx,fy,fz,taux,tauy,tauz]
  double gains_[6];

  // Wrench measure and command as rosmsg
  geometry_msgs::WrenchStamped wrench_measure_, wrench_command_;

  // Publisher for the twist commands
  ros::Publisher pub_twist_control_;

  // Command and measure wrench subscribers
  ros::Subscriber sub_wrench_command_, sub_wrench_measure_;

  // ServiceServer set enable
  ros::ServiceServer srv_server_set_enable_;

  // String for topic and service names
  std::string wrench_command_topic_str_, wrench_measure_topic_str_,
      twist_command_topic_str_, service_set_enable_str_;

public:
  // Public Constructor

  /*
      Constructor
      Initialize the controller
  */
  ROSWrenchController(ros::NodeHandle &nh_public, ros::NodeHandle &nh_private);

public:
  // Public Methods

  /*
      Stop the node
      use this in a SigintHandler to handle CTRL-C
  */
  void stop();

  /*
      Compute the mean of the measures
      using num_samples samples
      output has not the header set
  */
  geometry_msgs::WrenchStamped compute_measure_mean(int num_samples);

  /*
      Compute the control twist output
      Return a TwistStamped Message (header not set)
  */
  geometry_msgs::TwistStamped compute_control();

  /*
      Spin until new measure
  */
  void wait_wrench_measure();

  /*
      Get params from the parameter server
  */
  void get_ros_params(ros::NodeHandle &nh_private);

  /*
      Get Controller gains from the parameter server
  */
  void get_ros_params_gains(ros::NodeHandle &nh_private);

  /*
      Get topics and services name from the parameter server
  */
  void get_ros_params_topics_services_name(ros::NodeHandle &nh_private);

  /*
      Print the controller params
  */
  void print_params();

protected:
  // Protected Methods

  /*
  Service Callbk - Set Enable
  Set enable state of the Controller
  */
  bool srv_server_set_enabled_cb_(std_srvs::SetBool::Request &request,
                                  std_srvs::SetBool::Response &response);

  /*
      Command cb
      Update the command
  */
  void wrench_command_cb_(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  /*
      Measure CB
      Update the measure and call the controller
  */
  void wrench_measure_cb_(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  /*
      Append the stamp to the control message
      and publish it
  */
  void publish_control_(geometry_msgs::TwistStamped msg);

  /*
      Called before stop
       - call the user prestop callbk (if present)
       - Publish ZERO vel
  */
  void pre_stop_();

  /*
      Called before start
       - call the user prestart callbk (if present)
       - set the initial command equal to the mean of input (to have an initial
     Zero error)
  */
  void pre_start_();

}; // class
} // namespace sun

#endif