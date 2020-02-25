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

#define SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_FORCE 0.2

#define SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_TORQUE 0.01

namespace sun
{
class ROSWrenchControllerClient
{
public:
  class timeout_exception : public std::runtime_error
  {
  public:
    timeout_exception(std::string const& msg) : std::runtime_error(msg)
    {
    }
  };

protected:
  // Protected members

  ros::NodeHandle nh_public_;

  // Publisher for the wrench commands
  ros::Publisher pub_wrench_control_;

  // ServiceClient set enable
  ros::ServiceClient srv_client_set_enable_;

  // String for topic and service names
  std::string wrench_measure_topic_str_;

public:
  // Public Constructor

  /*
      Constructor
      Initialize the controller
  */
  ROSWrenchControllerClient(const ros::NodeHandle& nh_public, const std::string& wrench_command_topic,
                            const std::string& wrench_measure_topic, const std::string& service_set_enable);

public:
  enum Stop_Conditions
  {
    EQUAL,
    GREATER,
    LESS
  };

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
  Get a measure subscriber, actual_wrench and msg_arrived are filled on spin()
  */
  ros::Subscriber get_measure_subscriber(geometry_msgs::WrenchStamped& actual_wrench, bool& msg_arrived);

  /*
  Wait for the steady state
  */
  void wait_steady_state(const geometry_msgs::Wrench& desired_wrench, const ros::Duration& timeout = ros::Duration(-1),
                         double epsilon_force = SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_FORCE,
                         double epsilon_torque = SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_TORQUE);
  void wait_steady_state(const geometry_msgs::Wrench& desired_wrench, const bool mask[6],
                         const ros::Duration& timeout = ros::Duration(-1),
                         double epsilon_force = SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_FORCE,
                         double epsilon_torque = SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_TORQUE);
  void wait_component_steady_state(int component_index, double desired_value, int stop_condition,
                                   const ros::Duration& timeout = ros::Duration(-1),
                                   double epsilon_force = SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_FORCE,
                                   double epsilon_torque = SUN_ROS_WRENCH_CONTROLLER_CLIENT_DEFAULT_EPS_TORQUE);

  geometry_msgs::WrenchStamped get_measure_sample(const ros::Duration& timeout = ros::Duration(-1));

};  // class

}  // namespace sun

#endif