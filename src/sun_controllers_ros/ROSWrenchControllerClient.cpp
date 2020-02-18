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

void wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg, geometry_msgs::WrenchStamped& out_msg,
               bool& msg_arrived);

void wrench_error(geometry_msgs::Wrench w1, geometry_msgs::Wrench w2, const bool mask[6], double& force_error,
                  double& torque_error);

void wrench_error(const geometry_msgs::Wrench& w1, const geometry_msgs::Wrench& w2, double& force_error,
                  double& torque_error);

namespace sun
{
/*
    Constructor
    Initialize the controller
*/
ROSWrenchControllerClient::ROSWrenchControllerClient(const ros::NodeHandle& nh_public,
                                                     const std::string& wrench_command_topic,
                                                     const std::string& wrench_measure_topic,
                                                     const std::string& service_set_enable)
  : nh_public_(nh_public), wrench_measure_topic_str_(wrench_measure_topic)
{
  // Publishers
  pub_wrench_control_ = nh_public_.advertise<geometry_msgs::WrenchStamped>(wrench_command_topic, 1);

  // Service Clients
  srv_client_set_enable_ = nh_public_.serviceClient<std_srvs::SetBool>(service_set_enable);
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
   Wait for robot stop to desired pose
*/
void ROSWrenchControllerClient::wait_steady_state(const geometry_msgs::Wrench& desired_wrench,
                                                  const ros::Duration& timeout, double epsilon_force,
                                                  double epsilon_torque)
{
  ros::Time start_time = ros::Time::now();

  geometry_msgs::WrenchStamped actual_wrench;
  bool msg_arrived;
  boost::function<void(const geometry_msgs::WrenchStamped::ConstPtr& msg)> sub_cb =
      boost::bind(wrench_cb, _1, boost::ref(actual_wrench), boost::ref(msg_arrived));
  ros::Subscriber sub_pose = nh_public_.subscribe(wrench_measure_topic_str_, 1, sub_cb);

  msg_arrived = false;
  while (ros::ok())
  {
    if (msg_arrived)
    {
      double force_error, torque_error;
      wrench_error(desired_wrench, actual_wrench.wrench, force_error, torque_error);
      if (force_error < epsilon_force && torque_error < epsilon_torque)
      {
        return;
      }
      msg_arrived = false;
    }
    if (timeout >= ros::Duration(0))
    {
      ros::Time current_time = ros::Time::now();
      if ((current_time - start_time) >= timeout)
      {
        throw timeout_exception("ROSWrenchControllerClient::wait_steady_state timeout");
      }
    }
    ros::spinOnce();
  }
}

void ROSWrenchControllerClient::wait_steady_state(const geometry_msgs::Wrench& desired_wrench, const bool mask[6],
                                                  const ros::Duration& timeout, double epsilon_force,
                                                  double epsilon_torque)
{
  ros::Time start_time = ros::Time::now();

  geometry_msgs::WrenchStamped actual_wrench;
  bool msg_arrived;
  boost::function<void(const geometry_msgs::WrenchStamped::ConstPtr& msg)> sub_cb =
      boost::bind(wrench_cb, _1, boost::ref(actual_wrench), boost::ref(msg_arrived));
  ros::Subscriber sub_pose = nh_public_.subscribe(wrench_measure_topic_str_, 1, sub_cb);

  msg_arrived = false;
  while (ros::ok())
  {
    if (msg_arrived)
    {
      double force_error, torque_error;
      wrench_error(desired_wrench, actual_wrench.wrench, mask, force_error, torque_error);
      if (force_error < epsilon_force && torque_error < epsilon_torque)
      {
        return;
      }
      msg_arrived = false;
    }
    if (timeout >= ros::Duration(0))
    {
      ros::Time current_time = ros::Time::now();
      if ((current_time - start_time) >= timeout)
      {
        throw timeout_exception("ROSWrenchControllerClient::wait_steady_state timeout");
      }
    }
    ros::spinOnce();
  }
}

geometry_msgs::WrenchStamped ROSWrenchControllerClient::get_measure_sample(const ros::Duration& timeout)
{
  ros::Time start_time = ros::Time::now();

  geometry_msgs::WrenchStamped actual_wrench;
  bool msg_arrived;
  boost::function<void(const geometry_msgs::WrenchStamped::ConstPtr& msg)> sub_cb =
      boost::bind(wrench_cb, _1, boost::ref(actual_wrench), boost::ref(msg_arrived));
  ros::Subscriber sub_pose = nh_public_.subscribe(wrench_measure_topic_str_, 1, sub_cb);

  msg_arrived = false;
  while (ros::ok())
  {
    if (msg_arrived)
    {
      return actual_wrench;
    }
    if (timeout >= ros::Duration(0))
    {
      ros::Time current_time = ros::Time::now();
      if ((current_time - start_time) >= timeout)
      {
        throw timeout_exception("ROSWrenchControllerClient::get_measure_sample timeout");
      }
    }
    ros::spinOnce();
  }
}

};  // namespace sun

void wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg, geometry_msgs::WrenchStamped& out_msg,
               bool& msg_arrived)
{
  // Update measure
  out_msg = *msg;
  msg_arrived = true;
}

void wrench_error(geometry_msgs::Wrench w1, geometry_msgs::Wrench w2, const bool mask[6], double& force_error,
                  double& torque_error)
{
  if (mask[0])
  {
    w1.force.x = 0.0;
    w2.force.x = 0.0;
  }
  if (mask[1])
  {
    w1.force.y = 0.0;
    w2.force.y = 0.0;
  }
  if (mask[2])
  {
    w1.force.z = 0.0;
    w2.force.z = 0.0;
  }
  if (mask[3])
  {
    w1.torque.x = 0.0;
    w2.torque.x = 0.0;
  }
  if (mask[4])
  {
    w1.torque.y = 0.0;
    w2.torque.y = 0.0;
  }
  if (mask[5])
  {
    w1.torque.z = 0.0;
    w2.torque.z = 0.0;
  }
  wrench_error(w1, w2, force_error, torque_error);
}

void wrench_error(const geometry_msgs::Wrench& w1, const geometry_msgs::Wrench& w2, double& force_error,
                  double& torque_error)
{
  force_error =
      sqrt(pow(w1.force.x - w2.force.x, 2) + pow(w1.force.y - w2.force.y, 2) + pow(w1.force.z - w2.force.z, 2));

  torque_error =
      sqrt(pow(w1.torque.x - w2.torque.x, 2) + pow(w1.torque.y - w2.torque.y, 2) + pow(w1.torque.z - w2.torque.z, 2));
}