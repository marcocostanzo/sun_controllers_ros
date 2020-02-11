/*
    ROS Wrench Controller Client
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

#ifndef SUN_ROS_WRENCH_CONTROLLER_CLIENT_H
#define SUN_ROS_WRENCH_CONTROLLER_CLIENT_H

#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

namespace sun
{
class ROSWrenchControllerClient
{
protected:
  // Protected members

  ros::NodeHandle nh_public_;

  // true if a new wrench measure arrived
  bool wrench_measure_arrived_;

  // Wrench measure as rosmsg
  geometry_msgs::WrenchStamped wrench_measure_;

  // Publisher for the wrench commands
  ros::Publisher pub_wrench_control_;

  // Measure wrench subscribers
  ros::Subscriber sub_wrench_measure_;

  // ServiceClient set enable
  ros::ServiceClient srv_client_set_enable_;

  // String for topic and service names
  std::string wrench_command_topic_str_, wrench_measure_topic_str_, service_set_enable_str_;

public:
  // Public Constructor

  /*
      Constructor
      Initialize the controller
  */
  ROSWrenchControllerClient(const ros::NodeHandle& nh_public, const std::string& wrench_command_topic,
                            const std::string& wrench_measure_topic, const std::string& service_set_enable);

public:
  // Public Methods

  /*
      Start/Stop the controller
  */
  void set_enable(bool b);

  /*
  Publish the wrench command on the controller
  */
  void publish_wrench_command(geometry_msgs::WrenchStamped wrench_command);

  /*
  Wait for the steady state
  */
  void wait_steady_state(const geometry_msgs::WrenchStamped& desired_wrench, const ros::Duration& max_wait,
                         double epsilon_force = 0.2, double epsilon_torque = 0.003,
                         const ros::Time& t0 = ros::Time::now());

  void wait_steady_state(geometry_msgs::WrenchStamped desired_wrench, bool b_fx, bool b_fy, bool b_fz, bool b_mx,
                         bool b_my, bool b_mz, const ros::Duration& max_wait, double epsilon_force = 0.2,
                         double epsilon_torque = 0.003, const ros::Time& t0 = ros::Time::now());

  /*
      Check steady state
  */
  bool is_steady_state(const geometry_msgs::WrenchStamped& w1, const geometry_msgs::WrenchStamped& w2,
                       double epsilon_force = 0.2, double epsilon_torque = 0.003);

  /*
  Get a sample of the measure
  */
  const geometry_msgs::WrenchStamped& get_measure_sample(const ros::Duration& max_wait,
                                                         const ros::Time& t0 = ros::Time::now());

protected:
  // Protected Methods

  /*
      Measure CB
      Update the measure and call the controller
  */
  void wrench_measure_cb_(const geometry_msgs::WrenchStamped::ConstPtr& msg);

};  // class
}  // namespace sun

#endif