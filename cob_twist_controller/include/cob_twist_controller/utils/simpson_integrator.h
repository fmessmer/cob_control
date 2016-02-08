/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Felix Messmer, email: felix.messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: November, 2015
 *
 * \brief
 *   This Class provides a helper for integrating velocities using Simpson method
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H
#define COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H

#include <vector>

#include <ros/ros.h>
#include <kdl/jntarray.hpp>

#include "cob_twist_controller/utils/moving_average.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>


#include <iostream>
#include <fstream>

class SimpsonIntegrator
{
    public:
        explicit SimpsonIntegrator(const uint8_t dof)
        {
            dof_ = dof;
            for (uint8_t i = 0; i < dof_; i++)
            {
                ma_.push_back(new MovingAvgSimple_double_t(1));
                //ma_.push_back(new MovingAvgExponential_double_t(0.5));
                ma_output_.push_back(new MovingAvgSimple_double_t(2));
            }
            last_update_time_ = ros::Time(0.0);
            last_period_ = ros::Duration(0.0);
            start_shiftig_pos = false;
            calculate_velocity = false;
            q_dot_ik_pub_ = nh_.advertise<std_msgs::Float64> ("debug/q_dot_ik", 1);
            derived_q_dot_ik_pub_ = nh_.advertise<std_msgs::Float64> ("debug/derived_q_dot_ik", 1);
            pos_pub_ = nh_.advertise<std_msgs::Float64> ("debug/pos", 1);
            ratio_pub_ = nh_.advertise<std_msgs::Float64> ("debug/ratio", 1);
            pos_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("debug/pos_array", 1);
            summarize_ratio = 0;
            counter = 0;
        }
        ~SimpsonIntegrator()
        {}


        void resetIntegration()
        {
            // resetting outdated values
            vel_last_.clear();
            vel_before_last_.clear();

            // resetting moving average
            for (unsigned int i = 0; i < dof_; ++i)
            {
                ma_[i]->reset();
                ma_output_[i]->reset();
            }
        }

        bool updateIntegration(const KDL::JntArray& q_dot_ik,
                               const KDL::JntArray& current_q,
                               std::vector<double>& pos,
                               std::vector<double>& vel,
                               std::vector<double>& accl)
        {
//            if (!time.is_open()){
//                time.open ("/home/fxm-cm/Desktop/time.m");
//            }
//            if (!signal1.is_open()){
//                signal1.open ("/home/fxm-cm/Desktop/signal1.m");
//            }
//            if (!signal2.is_open()){
//                signal2.open ("/home/fxm-cm/Desktop/signal2.m");
//            }
//            if (!signal3.is_open()){
//                signal3.open ("/home/fxm-cm/Desktop/signal3.m");
//            }
//            if (!signal4.is_open()){
//                signal4.open ("/home/fxm-cm/Desktop/signal4.m");
//            }
//            if (!signal5.is_open()){
//                signal5.open ("/home/fxm-cm/Desktop/signal5.m");
//            }
//            if (!signal6.is_open()){
//                signal6.open ("/home/fxm-cm/Desktop/signal6.m");
//            }

            double norm_real_vel=0;
            double norm_int_vel=0;
            double norm_pos = 0;

            std_msgs::Float64 q_dot_ik_msg;
            std_msgs::Float64 derived_q_dot_ik_msg;
            std_msgs::Float64 pos_msg;
            std_msgs::Float64 ratio_msg;
            std_msgs::Float64MultiArray pos_array_msg;

            ros::Time now = ros::Time::now();
            ros::Duration period = now - last_update_time_;

            bool value_valid = false;
            pos.clear();
            vel.clear();
            accl.clear();
            pos_array_msg.data.clear();

            // ToDo: Test these conditions and find good thresholds
            // if (period.toSec() > 2*last_period_.toSec())  // missed about a cycle
            if (period.toSec() > ros::Duration(0.5).toSec())  // missed about 'max_command_silence'
            {
                ROS_WARN_STREAM("reset Integration: " << period.toSec());
                resetIntegration();
                counter = 0;
                summarize_ratio = 0;
            }

            if (!vel_before_last_.empty())
            {
                for (unsigned int i = 0; i < dof_; ++i)
                {
                    // Simpson
                    double integration_value = static_cast<double>(period.toSec() / 6.0 * (vel_before_last_[i] + 4.0 * (vel_before_last_[i] + vel_last_[i]) + vel_before_last_[i] + vel_last_[i] + q_dot_ik(i)) + current_q(i));
                    // double integration_value = static_cast<double>(period.toSec() * q_dot_ik(i) + current_q(i));
                    ma_[i]->addElement(integration_value);
                    double avg = 0.0;

                    // if (ma_[i]->calcMovingAverage(avg))
                    // {
                        // pos.push_back(avg);
                        pos.push_back(integration_value);
                        vel.push_back(q_dot_ik(i));
                    // }
                }

                value_valid = true;
            }

            // Continuously shift the vectors for simpson integration
            vel_before_last_.clear();
            for (unsigned int i=0; i < vel_last_.size(); ++i)
            {
                vel_before_last_.push_back(vel_last_[i]);
            }

            vel_last_.clear();
            for (unsigned int i=0; i < q_dot_ik.rows(); ++i)
            {
                vel_last_.push_back(q_dot_ik(i));
            }

            // Get acceleration
            for(unsigned int i=0; i < vel.size(); ++i)
            {
                accl.push_back((vel.at(i) - vel_last_.at(i)) / period.toSec());
                ROS_INFO("vel(%d): %.8f",i, vel.at(i));
                ROS_INFO("vel_last(%d): %.8f",i, vel_last_.at(i));
                ROS_INFO("accl(%d): %.8f",i, accl.at(i));
            }

            if (calculate_velocity)
            {
                // Norm of real velocity
                for (unsigned int i=0; i < q_dot_ik.rows(); ++i)
                {
                    norm_real_vel+=pow(q_dot_ik(i),2);
                }
                norm_real_vel = sqrt(norm_real_vel);

                // Norm of integrated position to velocity.
                for (unsigned int i=0; i < pos.size(); ++i)
                {
                    norm_int_vel += pow( (pos.at(i) - pos_before_.at(i)) / period.toSec(), 2);
                }
                norm_int_vel = sqrt(norm_int_vel);

                // Norm of the position.
                for (unsigned int i=0; i < pos.size(); ++i)
                {
                    norm_pos += pow(pos.at(i), 2);
                }
                norm_pos = sqrt(norm_pos);

//                if(pos.size()!=0)
//                {
//                    if (time.is_open()){
//                        time <<  ros::Time::now() << "\n";
//                    }
//                    if (signal1.is_open()){
//                          signal1 <<  pos.at(0) << "\n";
//                    }
//                    if (signal2.is_open()){
//                          signal2 <<  pos.at(1) << "\n";
//                    }
//                    if (signal3.is_open()){
//                          signal3 <<  pos.at(2) << "\n";
//                    }
//                    if (signal4.is_open()){
//                          signal4 <<  pos.at(3) << "\n";
//                    }
//                    if (signal5.is_open()){
//                          signal5 <<  pos.at(4) << "\n";
//                    }
//                    if (signal6.is_open()){
//                          signal6 <<  pos.at(5) << "\n";
//                    }
//                }
                double avg_int_vel;
                ma_output_[0]->addElement(norm_int_vel);
                ma_output_[0]->calcMovingAverage(avg_int_vel);

                // Prepare messages
                q_dot_ik_msg.data = norm_real_vel;
                derived_q_dot_ik_msg.data = avg_int_vel;
                pos_msg.data = norm_pos;
                ratio_msg.data = avg_int_vel / norm_real_vel;
                pos_array_msg.data = pos;

                // Publish
                q_dot_ik_pub_.publish(q_dot_ik_msg);
                derived_q_dot_ik_pub_.publish(derived_q_dot_ik_msg);
                pos_pub_.publish(pos_msg);
                ratio_pub_.publish(ratio_msg);
                pos_array_pub_.publish(pos_array_msg);

                if(avg_int_vel / norm_real_vel < 3 && isnan(avg_int_vel / norm_real_vel) == 0)
                {
                    summarize_ratio += avg_int_vel / norm_real_vel;
                    ROS_WARN_STREAM("ratio: " << summarize_ratio/counter);
                    counter++;
                }
            }

            if(value_valid)
            {
                pos_before_.clear();

                for (unsigned int i=0; i < pos.size(); ++i)
                {
                    pos_before_.push_back(pos.at(i));
                }
                calculate_velocity = true;
            }

            last_update_time_ = now;
            last_period_ = period;


            return value_valid;
        }

    private:
        std::vector<MovingAvgBase_double_t*> ma_;
        std::vector<MovingAvgBase_double_t*> ma_output_;
        uint8_t dof_;
        std::vector<double> vel_last_, vel_before_last_;
        ros::Time last_update_time_;
        ros::Duration last_period_;
        bool start_shiftig_pos;
        bool calculate_velocity;
        std::vector<double> pos_before_;
        ros::Publisher q_dot_ik_pub_, derived_q_dot_ik_pub_, pos_pub_, ratio_pub_;
        ros::NodeHandle nh_;
        double summarize_ratio;
        unsigned int counter;
        ros::Publisher pos_array_pub_;

//        std::ofstream time;
//        std::ofstream signal1,signal2,signal3,signal4,signal5,signal6;
};

#endif  // COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H
