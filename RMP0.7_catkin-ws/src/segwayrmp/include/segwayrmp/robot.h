#ifndef _ROBOT_H
#define _ROBOT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "comm_ctrl_navigation.h"
#include "segway_msgs/ticks.h"
#include "segway_msgs/speed.h"
#include "segway_msgs/info.h"
#include "segway_msgs/motor_enable.h"
#include "segway_msgs/chassis_poweroff.h"
#include "segway_msgs/load_param.h"
#include "segway_msgs/event_info.h"
#include "segway_msgs/vel_max.h"
#include "segway_msgs/remove_push.h"
#include "segway_msgs/vel_max_feedback.h"
#include "segway_msgs/set_hang_cmd.h"
#include "segway_msgs/set_hang_feedback.h"
#include "Ge_encoder_odometry.h"


namespace robot
{

    class Chassis
    {
        public:
            Chassis(const ros::NodeHandle& nh);
        private:
            void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel);
            void cmd_enable_callback(const segway_msgs::motor_enable::ConstPtr& msg);
            void cmd_load_callback(const segway_msgs::load_param::ConstPtr& msg);
            void cmd_poweroff_callback(const segway_msgs::chassis_poweroff::ConstPtr& msg);
            void cmd_vel_max_callback(const segway_msgs::vel_max::ConstPtr& msg);
            void cmd_remove_push_callback(const segway_msgs::remove_push::ConstPtr& msg);
            void cmd_set_hang_callback(const segway_msgs::set_hang_cmd::ConstPtr& msg);
            // static void PubData(StampedBasicFrame_ *frame);
            void TimeUpdate1000Hz(const ros::TimerEvent& event);
            void TimeUpdate1Hz(const ros::TimerEvent& event);
            void PubOdomToRosOdom(Odometry odom_data);
            void PubImuToRosImu(void);
        
            ros::NodeHandle nh_;

            ros::Subscriber velocity_sub_;
            ros::Subscriber enable_sub_;
            ros::Subscriber load_sub_;
            ros::Subscriber poweroff_sub_;
            ros::Subscriber vel_max_sub_;
            ros::Subscriber remove_push_sub_;
            ros::Subscriber set_hang_cmd_sub_;

            ros::Publisher Speed_pub;
            ros::Publisher Ticks_pub;
            ros::Publisher Info_pub;
            ros::Publisher Event_info_pub;
            ros::Publisher Odom_pub;
            ros::Publisher Imu_pub;
            ros::Publisher Set_Limit_Fb_pub;
            ros::Publisher hang_mode_fb_pub;

            ros::Timer update_timer_;
            ros::Timer update_timer2_;

            segway_msgs::speed speed;
            segway_msgs::ticks Ticks;
            segway_msgs::info Info;
            segway_msgs::event_info Event_info;
            nav_msgs::Odometry ROS_odom;
            sensor_msgs::Imu ros_imu;
            segway_msgs::vel_max_feedback Set_Limit_Fb;
            segway_msgs::set_hang_feedback set_hang_fb;

            double linear_vel_;
            double angular_vel_;

            s_aprctrl_datastamped_t timestamp_data;
            s_aprctrl_event_t       event_data;

            //std::shared_ptr<Ge_encoder_odometry> m_ge_encoder ;
            Ge_encoder_odometry m_ge_encoder;

    };

}

#endif
