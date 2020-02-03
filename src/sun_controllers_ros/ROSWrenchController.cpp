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
#include "sun_controllers_ros/ROSWrenchController.h"

    /*
        Constructor
        Initialize the controller
    */
    sun::ROSWrenchController::ROSWrenchController(
        ros::NodeHandle& nh_public,
        ros::NodeHandle& nh_private
        )
    {
        enabled_ = false;

        get_ros_params(nh_private);

        //Subscribers
        sub_wrench_command_ = 
            nh_public.subscribe(wrench_command_topic_str_, 1, &ROSWrenchController::wrench_command_cb_, this);
        sub_wrench_measure_ = 
            nh_public.subscribe(wrench_measure_topic_str_, 1, &ROSWrenchController::wrench_measure_cb_, this);

        //Publishers
        pub_twist_control_ = 
            nh_public.advertise<geometry_msgs::TwistStamped>(twist_command_topic_str_, 1);

        //Service Servers
        srv_server_set_enable_ = 
            nh_public.advertiseService(service_set_enable_str_ , &ROSWrenchController::srv_server_set_enabled_cb_ ,this);
    }

    /*
        Stop the node
        use this in a SigintHandler to handle CTRL-C
    */
    void sun::ROSWrenchController::stop()
    {
        if(enabled_)
            pre_stop_();
        enabled_ = false;
    }

    /*
        Compute the mean of the measures
        using num_samples samples
        output has not the header set
    */
    geometry_msgs::WrenchStamped sun::ROSWrenchController::compute_measure_mean(int num_samples)
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
            mean.wrench.force.x += wrench_measure_.wrench.force.x;
            mean.wrench.force.y += wrench_measure_.wrench.force.y;
            mean.wrench.force.z += wrench_measure_.wrench.force.z;
            mean.wrench.torque.x += wrench_measure_.wrench.torque.x;
            mean.wrench.torque.y += wrench_measure_.wrench.torque.y;
            mean.wrench.torque.z += wrench_measure_.wrench.torque.z;
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
    geometry_msgs::TwistStamped sun::ROSWrenchController::compute_control()
    {
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x = gains_[0]*( wrench_command_.wrench.force.x - wrench_measure_.wrench.force.x );
        twist.twist.linear.y = gains_[1]*( wrench_command_.wrench.force.y - wrench_measure_.wrench.force.y );
        twist.twist.linear.z = gains_[2]*( wrench_command_.wrench.force.z - wrench_measure_.wrench.force.z );
        twist.twist.angular.x = gains_[3]*( wrench_command_.wrench.torque.x - wrench_measure_.wrench.torque.x );
        twist.twist.angular.y = gains_[4]*( wrench_command_.wrench.torque.y - wrench_measure_.wrench.torque.y );
        twist.twist.angular.z = gains_[5]*( wrench_command_.wrench.torque.z - wrench_measure_.wrench.torque.z );
        return twist;
    }

    /*
        Spin until new measure
    */
    void sun::ROSWrenchController::wait_wrench_measure()
    {
        wrench_measure_arrived_ = false;
        while(ros::ok() && !wrench_measure_arrived_)
            ros::spinOnce();
    }

    /*
        Get params from the parameter server
    */
    void sun::ROSWrenchController::get_ros_params(ros::NodeHandle& nh_private)
    {
        get_ros_params_gains(nh_private);
        get_ros_params_topics_services_name(nh_private);
    }

    /*
        Get Controller gains from the parameter server
    */
    void sun::ROSWrenchController::get_ros_params_gains(ros::NodeHandle& nh_private)
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
                gains_[i]=gains_std[i];
        }
        else
        {
            for(int i=0;i<6;i++)
                gains_[i]=DEFAULT_GAINS;
        }
    }

    /*
        Get topics and services name from the parameter server
    */
    void sun::ROSWrenchController::get_ros_params_topics_services_name(ros::NodeHandle& nh_private)
    {
        nh_private.param("wrench_command_topic", wrench_command_topic_str_, std::string("wrench_command"));
        nh_private.param("wrench_measure_topic", wrench_measure_topic_str_, std::string("wrench_measure"));
        nh_private.param("twist_command_topic", twist_command_topic_str_, std::string("twist_command"));
        nh_private.param("service_set_enable", service_set_enable_str_, std::string("set_enable"));
    }

    /*
        Print the controller params
    */
    void sun::ROSWrenchController::print_params()
    {
        std::string output = "[Force Controller] PARAMS:\n";
        
        output += "\tGains: ";
        for(int i=0;i<6;i++)
            output += std::to_string(gains_[i]) + " ";
        
        output += "\n";
        output += "\twrench_command_topic: " +  wrench_command_topic_str_ + "\n";
        output += "\twrench_measure_topic: " +  wrench_measure_topic_str_ + "\n";
        output += "\ttwist_command_topic: " +  twist_command_topic_str_ + "\n";
        output += "\tservice_set_enable: " +  service_set_enable_str_ + "\n";

        ROS_INFO_STREAM(output);
    }

    /*
    Service Callbk - Set Enable
    Set enable state of the Controller
    */
    bool sun::ROSWrenchController::srv_server_set_enabled_cb_(
        std_srvs::SetBool::Request& request, 
        std_srvs::SetBool::Response& response
    )
    {
        response.message = "Force Controller ";
        if (request.data != enabled_)
        { //If status changed
            if(request.data) 
            { //if enabled_ requested
                pre_start_();
            } 
            else 
            { //if disable requested
                pre_stop_();
            }
            ROS_INFO("Force Controller %s", (request.data ? "enabled_" : "DISABLED"));
        } else {
            response.message += "Already ";
        }
        enabled_ = request.data;
        response.success = true;
        response.message += request.data  ? "enabled_" : "Disabled";
        return true;
    }

    /*
        Command cb
        Update the command
    */
    void sun::ROSWrenchController::wrench_command_cb_( const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {
        if(enabled_) //Parse command only if enabled_
        {
            wrench_command_ = *msg;
        }
    }

    /*
        Measure CB
        Update the measure and call the controller
    */
    void sun::ROSWrenchController::wrench_measure_cb_(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        //Update measure
        wrench_measure_ = *msg;
        wrench_measure_arrived_ = true;

        if(enabled_)
        {
            publish_control_( compute_control() );
        }

    }

    /*
        Append the stamp to the control message
        and publish it 
    */
    void sun::ROSWrenchController::publish_control_(geometry_msgs::TwistStamped msg)
    {
        //append time now
        msg.header.stamp = ros::Time::now();
        
        //call the the publish callbk
        pub_twist_control_.publish(msg);
    }

    /*
        Called before stop
         - call the user prestop callbk (if present)
         - Publish ZERO vel
    */
    void sun::ROSWrenchController::pre_stop_()
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
            publish_control_(msg);
    }

    /*
        Called before start
         - call the user prestart callbk (if present)
         - set the initial command equal to the mean of input (to have an initial Zero error)
    */
    void sun::ROSWrenchController::pre_start_()
    {
        //TODO call a user pre start cb if present

        //Compute mean of measure and set a zero initial error
        wrench_command_ = compute_measure_mean(MEAN_NUM_SAMPLES);
    }
