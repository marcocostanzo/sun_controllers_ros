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

#include "sun_controllers_ros/ROSWrenchControllerClient.h"

using namespace sun;

/*
    Constructor
    Initialize the controller
*/
ROSWrenchControllerClient::ROSWrenchControllerClient(const ros::NodeHandle& nh_public,
                                                     const std::string& wrench_command_topic,
                                                     const std::string& wrench_measure_topic,
                                                     const std::string& service_set_enable)
  : nh_public_(nh_public)
{
  // Subscribers
  sub_wrench_measure_ =
      nh_public_.subscribe(wrench_command_topic_str_, 1, &ROSWrenchControllerClient::wrench_measure_cb_, this);

  // Publishers
  pub_wrench_control_ = nh_public_.advertise<geometry_msgs::WrenchStamped>(wrench_command_topic_str_, 1);

  // Service Clients
  srv_client_set_enable_ = nh_public_.serviceClient<std_srvs::SetBool>(service_set_enable_str_);
}

/*
    Start/Stop the controller
*/
void ROSWrenchControllerClient::set_enable(bool b)
{
  std_srvs::SetBool srv_msg;
  srv_msg.request.data = b;

  if (!srv_client_set_enable_.call(srv_msg))
  {
    throw std::runtime_error("ROSWrenchControllerClient::set_enable Error in the server");
  }
  if (!srv_msg.response.success)
  {
    throw std::runtime_error("ROSWrenchControllerClient::set_enable response false - " + srv_msg.response.message);
  }
}

/*
Publish the wrench command on the controller
*/
void ROSWrenchControllerClient::publish_wrench_command(geometry_msgs::WrenchStamped wrench_command)
{
  wrench_command.header.stamp = ros::Time::now();
  pub_wrench_control_.publish(wrench_command);
}

/*
Wait for the steady state
*/
void ROSWrenchControllerClient::wait_steady_state(const geometry_msgs::WrenchStamped& desired_wrench,
                                                  const ros::Duration& max_wait, double epsilon_force,
                                                  double epsilon_torque, const ros::Time& t0)
{
  ros::Duration wait_remain = max_wait;
  // wait a measure sample
  get_measure_sample(wait_remain, t0);
  wait_remain = max_wait - (ros::Time::now() - t0);
  bool steady_state = is_steady_state(wrench_measure_, desired_wrench, epsilon_force, epsilon_torque);
  while (ros::ok() && !steady_state && (wait_remain.toSec() > 0.0))
  {
    get_measure_sample(wait_remain, t0);
    steady_state = is_steady_state(wrench_measure_, desired_wrench, epsilon_force, epsilon_torque);
    wait_remain = max_wait - (ros::Time::now() - t0);
  }
  if (!steady_state)
  {
    throw std::runtime_error("ROSWrenchControllerClient::wait_steady_state Timeout");
  }
}

/*
Wait for the steady state on specifc components
*/
void ROSWrenchControllerClient::wait_steady_state(geometry_msgs::WrenchStamped desired_wrench, bool b_fx, bool b_fy,
                                                  bool b_fz, bool b_mx, bool b_my, bool b_mz,
                                                  const ros::Duration& max_wait, double epsilon_force,
                                                  double epsilon_torque, const ros::Time& t0)
{
  ros::Duration wait_remain = max_wait;
  // set the desired wrench
  if (!b_fx)
    desired_wrench.wrench.force.x = 0.0;
  if (!b_fy)
    desired_wrench.wrench.force.y = 0.0;
  if (!b_fz)
    desired_wrench.wrench.force.z = 0.0;
  if (!b_mx)
    desired_wrench.wrench.torque.x = 0.0;
  if (!b_my)
    desired_wrench.wrench.torque.y = 0.0;
  if (!b_mz)
    desired_wrench.wrench.torque.z = 0.0;
  // wait a measure sample
  geometry_msgs::WrenchStamped wrench_measure = get_measure_sample(wait_remain, t0);
  if (!b_fx)
    wrench_measure.wrench.force.x = 0.0;
  if (!b_fy)
    wrench_measure.wrench.force.y = 0.0;
  if (!b_fz)
    wrench_measure.wrench.force.z = 0.0;
  if (!b_mx)
    wrench_measure.wrench.torque.x = 0.0;
  if (!b_my)
    wrench_measure.wrench.torque.y = 0.0;
  if (!b_mz)
    wrench_measure.wrench.torque.z = 0.0;
  wait_remain = max_wait - (ros::Time::now() - t0);
  bool steady_state = is_steady_state(wrench_measure, desired_wrench, epsilon_force, epsilon_torque);
  while (ros::ok() && !steady_state && (wait_remain.toSec() > 0.0))
  {
    wrench_measure = get_measure_sample(wait_remain, t0);
    if (!b_fx)
      wrench_measure.wrench.force.x = 0.0;
    if (!b_fy)
      wrench_measure.wrench.force.y = 0.0;
    if (!b_fz)
      wrench_measure.wrench.force.z = 0.0;
    if (!b_mx)
      wrench_measure.wrench.torque.x = 0.0;
    if (!b_my)
      wrench_measure.wrench.torque.y = 0.0;
    if (!b_mz)
      wrench_measure.wrench.torque.z = 0.0;
    steady_state = is_steady_state(wrench_measure, desired_wrench, epsilon_force, epsilon_torque);
    wait_remain = max_wait - (ros::Time::now() - t0);
  }
  if (!steady_state)
  {
    throw std::runtime_error("ROSWrenchControllerClient::wait_steady_state Timeout");
  }
}

/*
    Check steady state
*/
bool ROSWrenchControllerClient::is_steady_state(const geometry_msgs::WrenchStamped& w1,
                                                const geometry_msgs::WrenchStamped& w2, double epsilon_force,
                                                double epsilon_torque)
{
  return ((sqrt(pow(w1.wrench.force.x - w2.wrench.force.x, 2) + pow(w1.wrench.force.y - w2.wrench.force.y, 2) +
                pow(w1.wrench.force.z - w2.wrench.force.z, 2)) < epsilon_force) &&
          (sqrt(pow(w1.wrench.torque.x - w2.wrench.torque.x, 2) + pow(w1.wrench.torque.y - w2.wrench.torque.y, 2) +
                pow(w1.wrench.torque.z - w2.wrench.torque.z, 2)) < epsilon_torque));
}

/*
Get a sample of the measure
*/
const geometry_msgs::WrenchStamped& ROSWrenchControllerClient::get_measure_sample(const ros::Duration& max_wait,
                                                                                  const ros::Time& t0)
{
  ros::Duration wait_remain = max_wait;
  wrench_measure_arrived_ = false;
  while (ros::ok() && !wrench_measure_arrived_ && (wait_remain.toSec() > 0.0))
  {
    ros::spinOnce();
    wait_remain = max_wait - (ros::Time::now() - t0);
  }
  if (!wrench_measure_arrived_)
  {
    throw std::runtime_error("ROSWrenchControllerClient::get_measure_sample timeout");
  }
  return wrench_measure_;
}

/*
    Measure CB
    Update the measure and call the controller
*/
void ROSWrenchControllerClient::wrench_measure_cb_(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  // Update measure
  wrench_measure_ = *msg;
  wrench_measure_arrived_ = true;
}
