/*
   test chassis response
*/
#if 1
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "segwayrmp/robot.h"


using namespace std;

namespace
{
    void event_info_callback(const segway_msgs::event_info::ConstPtr& msg)
    {
        ROS_INFO("receive_chassis event_info.msg, event_id:%u\r\n", 
         msg->event_id);
    }

    void info_callback(const segway_msgs::info::ConstPtr& msg)
    {
        ROS_INFO("receive_chassis info.msg, motor_version:%#x, motor_error:%#x, \
        connect_version:%#x, connect_error:%#x, route_version:%#x, route_version:%#x, \
        bat_soc:%d, motor_work_mode:%u, vehicle_meters:%u\r\n", 
         msg->motor_version,  msg->motor_error, 
         msg->connect_version,  msg->connect_error,  
         msg->route_version,  msg->route_version,  
         msg->bat_soc,  msg->motor_work_mode, msg->vehicle_meters);
    }

    void speed_callback(const segway_msgs::speed::ConstPtr& msg)
    {
        ROS_INFO("receive_chassis speed.msg, l_speed:%f, r_speed:%f, \
        speed_timestamp:%lu\r\n", 
         msg->l_speed,  msg->r_speed, 
         msg->speed_timestamp);
    }

    void ticks_callback(const segway_msgs::ticks::ConstPtr& msg)
    {
        ROS_INFO("receive_chassis ticks.msg, l_ticks:%d, r_ticks:%d, \
        ticks_timestamp:%lu\r\n", 
         msg->l_ticks,  msg->r_ticks, 
         msg->ticks_timestamp);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        ROS_INFO("receive_chassis Imu.msg, angular_vel:%f, linear_acc:%f, \
        ticks_timestamp:%d\r\n", 
        msg->angular_velocity.z, msg->linear_acceleration.x, 
        msg->header.seq);
    }
}

int main(int argc, char **argv) {
    // Initiate ROS
    ros::init(argc, argv, "receive_chassis_test");
    ros::NodeHandle node_;

    //chassis
    ros::Subscriber event_info_ = node_.subscribe("event_info", 10, &event_info_callback);
    ros::Subscriber info_sub_ = node_.subscribe("info", 10, info_callback);
    ros::Subscriber speed_sub_ = node_.subscribe("speed", 10, speed_callback);
    ros::Subscriber ticks_sub_ = node_.subscribe("ticks", 10, ticks_callback);
    ros::Subscriber imu_sub_ = node_.subscribe("imu", 10, imu_callback);

    ros::spin();
    return 0;
}
#endif
