/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_lookat_controller
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This class provides a virtual driver for the lookat kinematic chain
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_lookat_controller/cob_lookat_driver.h>


bool CobLookatDriver::initialize()
{
	// JointNames
	if(!nh_.getParam("joint_names", joints_))
	{
		ROS_ERROR("Parameter 'joint_names' not set");
		return false;
	}
	dof_ = joints_.size();
	
	current_pos_.assign(dof_, 0.0);
	current_pos_[0] = 1.0;	//this is in order to see the frame
	current_vel_.assign(dof_, 0.0);
	current_eff_.assign(dof_, 0.0);
	
	if (!nh_.getParam("update_rate", update_rate_))
	{	update_rate_ = 50.0;	}
	
	if (!nh_.getParam("max_command_silence", max_command_silence_))
	{	max_command_silence_ = 0.3;	}
	
	command_vel_sub_ = nh_.subscribe("joint_group_velocity_controller/command", 1, &CobLookatDriver::command_vel_cb, this);
	jointstate_pub_ = nh_.advertise<sensor_msgs::JointState> ("joint_states", 1);
	
	ROS_INFO("...initialized!");
	return true;
}

void CobLookatDriver::run()
{
	ros::Rate r(update_rate_);
	while(ros::ok())
	{
		update_state();
		publish_state();
		
		ros::spinOnce();
		r.sleep();
	}
}

void CobLookatDriver::update_state()
{
	double dt = (ros::Time::now() - last_update_).toSec();
	
	for(unsigned int i=0; i<dof_; i++)
		current_pos_[i] += current_vel_[i]*dt;
	
	// max_vel_silence_time - stopping
	if(dt>max_command_silence_)
		current_vel_.assign(dof_, 0.0);
}



void CobLookatDriver::publish_state()
{
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
	msg.name = joints_;
	msg.position = current_pos_;
	msg.velocity = current_vel_;
	msg.effort = current_eff_;
	
	jointstate_pub_.publish(msg);
}

void CobLookatDriver::command_vel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	///ToDo: Do some checks!
	if(msg->data.size() != dof_)
	{
		ROS_ERROR("DoF do not match! Stopping!");
		current_vel_.assign(dof_, 0.0);
		return;
	}
	
	for(unsigned int i=0; i<dof_; i++)
	{
		current_vel_[i]=msg->data[i];
	}
	
	last_update_ = ros::Time::now();
}

