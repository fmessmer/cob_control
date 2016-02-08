#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <kdl/jntarray.hpp>
#include <cob_twist_controller/utils/simpson_integrator.h>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <signal.h>
#include <trajectory_msgs/JointTrajectory.h>

class SimpsonIntegratorTester
{
public:
    SimpsonIntegratorTester()
    {
        dof_ = 7;
        idx_ = 4; // 0, 2, 4, 6

        q_.resize(dof_);
        KDL::SetToZero(q_);
        q_dot_.resize(dof_);
        KDL::SetToZero(q_dot_);
        simpson_integrated_q_dot_.resize(dof_);
        KDL::SetToZero(simpson_integrated_q_dot_);
        euler_integrated_q_dot_.resize(dof_);
        KDL::SetToZero(euler_integrated_q_dot_);
        simpson_derived_q_dot_.resize(dof_);
        KDL::SetToZero(simpson_derived_q_dot_);



        integrator_.reset(new SimpsonIntegrator(dof_));
        output_sin_pub_ = nh_.advertise<std_msgs::Float64>("sin", 1);
        output_derived_simpson_pub_ = nh_.advertise<std_msgs::Float64>("derived_simpson", 1);
        output_simpson_sin_pub_ = nh_.advertise<std_msgs::Float64>("simpson", 1);
        output_cos_pub_ = nh_.advertise<std_msgs::Float64>("cos", 1);
        output_euler_sin_pub_ = nh_.advertise<std_msgs::Float64>("euler", 1);
        trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm/joint_trajectory_controller/command", 1);

        stop_=false;
        time_ = 0;
        
        ros::Duration(1.0).sleep();
    }


    ~SimpsonIntegratorTester()
    {}

    void run()
    {
        ros::Rate r(100.0);

//        boost::thread start_thread;
//        start_thread = boost::thread(boost::bind(&SimpsonIntegratorTester::stopIntegration, this));
//        ros::AsyncSpinner spinner(0);
//        spinner.start();
        ROS_INFO("Start integration \n Enter any key to stop it.");

        ros::Time time = ros::Time::now();
        ros::Time last_update_time = time;
        ros::Duration period = time - last_update_time;
        double old_pos = -99;
        double a = 0.4, b = 0.4, c = 0, d = 0;
        euler_integrated_q_dot_(idx_) = a*sin(b*time_+c) + d;
        
        trajectory_msgs::JointTrajectoryPoint traj_point;
        traj_point.positions.assign(dof_,0.0);
    
        std::vector<std::string> joint_names;
        joint_names.push_back("arm_1_joint");
        joint_names.push_back("arm_2_joint");
        joint_names.push_back("arm_3_joint");
        joint_names.push_back("arm_4_joint");
        joint_names.push_back("arm_5_joint");
        joint_names.push_back("arm_6_joint");
        joint_names.push_back("arm_7_joint");
        
        while(ros::ok())
        {
            time = ros::Time::now();
            period = time - last_update_time;

            std::vector<double> next_q;
            std::vector<double> next_q_dot;

            q_dot_(idx_) = a*b*cos(c+b*time_);
            q_(idx_)     = a*sin(b*time_+c) + d;

            if (integrator_->updateIntegration(q_dot_, q_, next_q, next_q_dot, q_dotdot_))
            {
                simpson_integrated_q_dot_(idx_) = next_q[idx_];
                q_dot_(idx_) = next_q_dot[idx_];
            }

            euler_integrated_q_dot_(idx_) += q_dot_(idx_) * period.toSec();

            if(old_pos != -99)
            {
                simpson_derived_q_dot_(idx_) = (euler_integrated_q_dot_(idx_) - old_pos)/period.toSec();
            }
            

            traj_point.positions[idx_] = q_(idx_);
            //~ traj_point.velocities = vel;
            //~ traj_point.time_from_start = ros::Duration(0.5);  // Forced time in which the current position has to move to the next position
            traj_point.time_from_start = ros::Duration(period.toSec());  // Forced time in which the current position has to move to the next position

            trajectory_msgs::JointTrajectory traj_msg;
            traj_msg.points.push_back(traj_point); 
            traj_msg.joint_names = joint_names;
            //~ traj_msg.header.stamp = ros::Time::now();
            


            std_msgs::Float64 simpson_q_;
            simpson_q_.data = simpson_integrated_q_dot_(idx_);
            std_msgs::Float64 euler_q_;
            euler_q_.data = euler_integrated_q_dot_(idx_);
            std_msgs::Float64 real_q;
            real_q.data = q_(idx_);
            std_msgs::Float64 derived_q_;
            derived_q_.data = simpson_derived_q_dot_(idx_);
            std_msgs::Float64 real_sin;
            real_sin.data = q_dot_(idx_);


            output_simpson_sin_pub_.publish(simpson_q_);
            output_cos_pub_.publish(real_q);
            output_euler_sin_pub_.publish(euler_q_);
            output_euler_sin_pub_.publish(euler_q_);
            output_derived_simpson_pub_.publish(derived_q_);
            output_sin_pub_.publish(real_sin);
            trajectory_pub_.publish(traj_msg);
            
            time_+=period.toSec();
            last_update_time = time;
            old_pos = euler_integrated_q_dot_(idx_);
            ros::spinOnce();
            r.sleep();
        }
    }

//        tcsetattr(kfd, TCSANOW, &cooked);

    void stopIntegration()
    {
        c = 0x0;
        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        while(ros::ok())
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }
            if(c == 0x61)
            {
                stop_ = true;
                break;
            }
        }
    }


    ros::NodeHandle nh_;
    ros::Publisher output_sin_pub_;
    ros::Publisher output_derived_simpson_pub_;

    ros::Publisher output_simpson_sin_pub_;
    ros::Publisher output_cos_pub_;
    ros::Publisher output_euler_sin_pub_;
    ros::Publisher trajectory_pub_;

    KDL::JntArray q_;
    KDL::JntArray q_dot_;
    std::vector<double> q_dotdot_;
    KDL::JntArray simpson_integrated_q_dot_;
    KDL::JntArray euler_integrated_q_dot_;
    KDL::JntArray simpson_derived_q_dot_;


    boost::shared_ptr<SimpsonIntegrator> integrator_;
    bool stop_;
    double time_;
    unsigned int dof_;
    unsigned int idx_;

    /// For Keyboard commands
    char c;
    int kfd;
    struct termios cooked, raw;

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_simpson_integrator_node");

    SimpsonIntegratorTester sit;
    sit.run();
    ros::spin();
    return 0;
}
