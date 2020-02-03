/*
    ROS Force Controller
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

#ifndef SUN_ROS_FORCE_CONTROLLER
#define SUN_ROS_FORCE_CONTROLLER

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_srvs/SetBool.h"

#define NUM_ZERO_VEL_COMMANDS_TO_SEND 20
#define MEAN_NUM_SAMPLES 50
#define DEFAULT_GAINS 0.0

class ROSForceController
{
    public:

    bool wrench_measure_arrived;
    bool enabled;
    geometry_msgs::WrenchStamped wrench_measure, wrench_command;
    ros::Publisher pub_twiast_control;
    double gains[6];
    ros::Subscriber sub_wrench_command, sub_wrench_measure;
    ros::ServiceServer srv_server_set_enable;
    std::string wrench_command_topic_str, 
                wrench_measure_topic_str, 
                twist_command_topic_str,
                service_set_enable_str;

    /*
        Constructor
        Initialize the controller
    */
    ROSForceController(
        ros::NodeHandle& nh_public,
        ros::NodeHandle& nh_private
        )
    {
        enabled = false;

        get_ros_params(nh_private);

        //Subscribers
        sub_wrench_command = 
            nh_public.subscribe(wrench_command_topic_str, 1, &ROSForceController::wrench_command_cb, this);
        sub_wrench_measure = 
            nh_public.subscribe(wrench_measure_topic_str, 1, &ROSForceController::wrench_measure_cb, this);

        //Publishers
        pub_twiast_control = 
            nh_public.advertise<geometry_msgs::TwistStamped>(twist_command_topic_str, 1);

        //Service Servers
        srv_server_set_enable = 
            nh_public.advertiseService(service_set_enable_str , &ROSForceController::srv_server_set_enabled_cb ,this);
    }

    /*
        Stop the node
        use this in a SigintHandler to handle CTRL-C
    */
    void stop()
    {
        if(enabled)
            pre_stop();
        enabled = false;
    }

    /*
    Service Callbk - Set Enable
    Set enable state of the Controller
    */
    bool srv_server_set_enabled_cb(
        std_srvs::SetBool::Request& request, 
        std_srvs::SetBool::Response& response
    )
    {
        response.message = "Force Controller ";
        if (request.data != enabled)
        { //If status changed
            if(request.data) 
            { //if enabled requested
                pre_start();
            } 
            else 
            { //if disable requested
                pre_stop();
            }
            ROS_INFO("Force Controller %s", (request.data ? "ENABLED" : "DISABLED"));
        } else {
            response.message += "Already ";
        }
        enabled = request.data;
        response.success = true;
        response.message += request.data  ? "Enabled" : "Disabled";
        return true;
    }

    /*
        Command cb
        Update the command
    */
    void wrench_command_cb( const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {
        if(enabled) //Parse command only if enabled
        {
            wrench_command = *msg;
        }
    }

    /*
        Measure CB
        Update the measure and call the controller
    */
    void wrench_measure_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        //Update measure
        wrench_measure = *msg;
        wrench_measure_arrived = true;

        if(enabled)
        {
            publish_control( compute_control() );
        }

    }

    /*
        Append the stamp to the control message
        and publish it 
    */
    void publish_control(geometry_msgs::TwistStamped msg)
    {
        //append time now
        msg.header.stamp = ros::Time::now();
        
        //call the the publish callbk
        pub_twiast_control.publish(msg);
    }

    /*
        Called before stop
         - call the user prestop callbk (if present)
         - Publish ZERO vel
    */
    void pre_stop()
    {
        //TODO call a user pre stop cb if present
        
        //Build zero vel
        geometry_msgs::TwistStamped msg;
        msg.twist.linear.x = 0.0;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;

        for(int i=0; i<NUM_ZERO_VEL_COMMANDS_TO_SEND; i++)
            publish_control(msg);
    }

    /*
        Called before start
         - call the user prestart callbk (if present)
         - set the initial command equal to the mean of input (to have an initial Zero error)
    */
    void pre_start()
    {
        //TODO call a user pre start cb if present

        //Compute mean of measure and set a zero initial error
        wrench_command = compute_measure_mean(MEAN_NUM_SAMPLES);
    }

    /*
        Compute the mean of the measures
        using num_samples samples
        output has not the header set
    */
    geometry_msgs::WrenchStamped compute_measure_mean(int num_samples)
    {
        geometry_msgs::WrenchStamped mean;
        mean.wrench.force.x = 0.0;
        mean.wrench.force.y = 0.0;
        mean.wrench.force.z = 0.0;
        mean.wrench.torque.x = 0.0;
        mean.wrench.torque.y = 0.0;
        mean.wrench.torque.z = 0.0;
        
        for(int i=0; i<num_samples; i++)
        {
            wait_wrench_measure();
            mean.wrench.force.x += wrench_measure.wrench.force.x;
            mean.wrench.force.y += wrench_measure.wrench.force.y;
            mean.wrench.force.z += wrench_measure.wrench.force.z;
            mean.wrench.torque.x += wrench_measure.wrench.torque.x;
            mean.wrench.torque.y += wrench_measure.wrench.torque.y;
            mean.wrench.torque.z += wrench_measure.wrench.torque.z;
        }
        mean.wrench.force.x /= num_samples;
        mean.wrench.force.y /= num_samples;
        mean.wrench.force.z /= num_samples;
        mean.wrench.torque.x /= num_samples;
        mean.wrench.torque.y /= num_samples;
        mean.wrench.torque.z /= num_samples;
        
        return mean;
    }

    /*
        Compute the control twist output
        Return a TwistStamped Message (header not set)
    */
    geometry_msgs::TwistStamped compute_control()
    {
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x = gains[0]*( wrench_command.wrench.force.x - wrench_measure.wrench.force.x );
        twist.twist.linear.y = gains[1]*( wrench_command.wrench.force.y - wrench_measure.wrench.force.y );
        twist.twist.linear.z = gains[2]*( wrench_command.wrench.force.z - wrench_measure.wrench.force.z );
        twist.twist.angular.x = gains[3]*( wrench_command.wrench.torque.x - wrench_measure.wrench.torque.x );
        twist.twist.angular.y = gains[4]*( wrench_command.wrench.torque.y - wrench_measure.wrench.torque.y );
        twist.twist.angular.z = gains[5]*( wrench_command.wrench.torque.z - wrench_measure.wrench.torque.z );
        return twist;
    }

    /*
        Spin until new measure
    */
    void wait_wrench_measure()
    {
        wrench_measure_arrived = false;
        while(ros::ok() && !wrench_measure_arrived)
            ros::spinOnce();
    }

    /*
        Get params from the parameter server
    */
    void get_ros_params(ros::NodeHandle& nh_private)
    {
        get_ros_params_gains(nh_private);
        get_ros_params_topics_services_name(nh_private);
    }

    /*
        Get Controller gains from the parameter server
    */
    void get_ros_params_gains(ros::NodeHandle& nh_private)
    {
        if(nh_private.hasParam("gains"))
        {
            std::vector<double> gains_std;
            if(!nh_private.getParam("gains", gains_std))
            { //Param is not std::vector<double>
                ROS_ERROR("[Force Controller] INVALID PARAM TYPE: gains has to be a vector [...]");
                ros::shutdown();
                exit(-1);
            }
            if(gains_std.size() != 6)
            {
                ROS_ERROR("[Force Controller] INVALID PARAM: gains length has to be 6");
                ros::shutdown();
                exit(-1);
            }
            for(int i=0;i<6;i++)
                gains[i]=gains_std[i];
        }
        else
        {
            for(int i=0;i<6;i++)
                gains[i]=DEFAULT_GAINS;
        }
    }

    /*
        Get topics and services name from the parameter server
    */
    void get_ros_params_topics_services_name(ros::NodeHandle& nh_private)
    {
        nh_private.param("wrench_command_topic", wrench_command_topic_str, std::string("wrench_command"));
        nh_private.param("wrench_measure_topic", wrench_measure_topic_str, std::string("wrench_measure"));
        nh_private.param("twist_command_topic", twist_command_topic_str, std::string("twist_command"));
        nh_private.param("service_set_enable", service_set_enable_str, std::string("set_enable"));
    }

    /*
        Print the controller params
    */
    void print_params()
    {
        std::cout << "CIAO\n";
        std::string output = "[Force Controller] PARAMS:\n";
        
        output += "\tGains: ";
        for(int i=0;i<6;i++)
            output += std::to_string(gains[i]) + " ";
        
        output += "\n";
        output += "\twrench_command_topic: " +  wrench_command_topic_str + "\n";
        output += "\twrench_measure_topic: " +  wrench_measure_topic_str + "\n";
        output += "\ttwist_command_topic: " +  twist_command_topic_str + "\n";
        output += "\tservice_set_enable: " +  service_set_enable_str + "\n";

        ROS_INFO_STREAM(output);
    }

};

#endif