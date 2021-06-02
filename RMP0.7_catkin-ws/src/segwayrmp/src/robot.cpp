#include "segwayrmp/robot.h"
#include <sensor_msgs/Imu.h>
#include "Ge_encoder_odometry.h"
#if 1
#define ODOM_BY_CHASSIS

#define IMU_ANGULAR_VEL_CONVERT_UINIT 0.0009288//(2000 /32767 / 0.0163835 / 70 / 57.3)//FS:2000dps; 0.0163835/70：coefficient；57.3：rad->degree
#define IMU_LINEAR_VEL_CONVERT_UINIT 0.0023943//(9.80665 * 8 /32767 )//FS:8G; 
car_speed_data_t SpeedData;
uint64_t    Speed_TimeStamp;
uint8_t     Speed_update;

motor_ticks_t TicksData;
uint64_t    Ticks_TimeStamp;
uint8_t     Ticks_update;

imu_gyr_original_data_t ImuGyrData;
uint64_t    ImuGyr_TimeStamp;
uint8_t     ImuGyr_update;

imu_acc_original_data_t ImuAccData;
uint64_t    ImuAcc_TimeStamp;
uint8_t     ImuAcc_update;

odom_pos_xy_t       OdomPoseXy;
odom_euler_xy_t     OdomEulerXy;
odom_euler_z_t      OdomEulerZ;
odom_vel_line_xy_t  OdomVelLineXy;
uint64_t    Odom_TimeStamp;
uint8_t     Odom_update;

set_max_limit_value_fb_t    LimitVelSetFb;
uint64_t    LimitFb_TimeStamp;
uint8_t     LimitFb_update;

int chassis_event_id = 0;

static void PubData(StampedBasicFrame_ *frame)
{
    if(frame->type_id == Chassis_Data_Speed)
    {
        memcpy(&SpeedData,frame->data, sizeof(SpeedData));//Speed data from chassis
        Speed_TimeStamp = frame->timestamp;
        Speed_update = 1;
    }else if(frame->type_id == Chassis_Data_Ticks)
    {
        memcpy(&TicksData,frame->data, sizeof(TicksData));//Ticks data from chassis
        Ticks_TimeStamp = frame->timestamp;
        Ticks_update = 1;
        // ROS_INFO("LTICKS:%d, rticks:%d", TicksData.l_ticks, TicksData.r_ticks);        
    }
    else if (frame->type_id == Chassis_Data_Imu_Gyr)
    {
        memcpy(&ImuGyrData,frame->data, sizeof(ImuGyrData));//Ticks data from chassis
        ImuGyr_TimeStamp = frame->timestamp;
        ImuGyr_update = 1;        
        // ROS_INFO("GYR0:%d, gyr1:%d, gyr2:%d", ImuGyrData.gyr[0], ImuGyrData.gyr[1], ImuGyrData.gyr[2]);
    }
    else if (frame->type_id == Chassis_Data_Imu_Acc)
    {
        memcpy(&ImuAccData,frame->data, sizeof(ImuAccData));//Ticks data from chassis
        ImuAcc_TimeStamp = frame->timestamp;
        ImuAcc_update = 1;
        // ROS_INFO("ACC0:%d, acc1:%d, acc2:%d", ImuAccData.acc[0], ImuAccData.acc[1], ImuAccData.acc[2]);
    }
    else if (frame->type_id == Chassis_Data_Odom_Pose_xy)
    {
        memcpy(&OdomPoseXy,frame->data, sizeof(OdomPoseXy));//Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 1;
        // ROS_INFO("odomPosX:%f, odomPosY:%f", OdomPoseXy.pos_x, OdomPoseXy.pos_y);
    }
    else if (frame->type_id == Chassis_Data_Odom_Euler_xy)
    {
        memcpy(&OdomEulerXy,frame->data, sizeof(OdomEulerXy));//Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 2;
        // ROS_INFO("OdomEulerX:%f, OdomEulerY:%f", OdomEulerXy.euler_x, OdomEulerXy.euler_y);
    }
    else if (frame->type_id == Chassis_Data_Odom_Euler_z)
    {
        memcpy(&OdomEulerZ,frame->data, sizeof(OdomEulerZ));//Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 4;
        // ROS_INFO("OdomEulerZ:%f", OdomEulerZ.euler_z);
    }
    else if (frame->type_id == Chassis_Data_Odom_Linevel_xy)
    {
        memcpy(&OdomVelLineXy,frame->data, sizeof(OdomVelLineXy));//Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 8;
        // ROS_INFO("OdomVelLineX:%f, OdomVelLineY:%f", OdomVelLineXy.vel_line_x, OdomVelLineXy.vel_line_y);
    }
    else if (frame->type_id == Chassis_Data_Limit_vel_set_fb)
    {
        memcpy(&LimitVelSetFb,frame->data, sizeof(LimitVelSetFb));//Ticks data from chassis
        LimitFb_TimeStamp = frame->timestamp;
        LimitFb_update = 1;
     }
}

static void EvnetPubData(int event_no)
{
    chassis_event_id = event_no;
}

//The timestamp from the upper computer is the count of microseconds，
ros::Time timestamp2rostime(int64_t timestamp){
//    std::string suanz = std::to_string(timestamp);
//    std::string sec_string = suanz.substr(0,10);
//    std::string nsec_string = suanz.substr(10,9);
//    while(nsec_string.length() < 9){
//        nsec_string += "0";
//    }
//    return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
    uint32_t sec_ = timestamp / 1000000;
    uint32_t nsec_ = (timestamp % 1000000) * 1000;
    return ros::Time(sec_, nsec_);
}

double getOrientationX()
{
    float x = OdomEulerXy.euler_x / 2;
    float y = OdomEulerXy.euler_y / 2;
    float z = OdomEulerZ.euler_z / 2;

    return (cos(x) * sin(y) * cos(z) + sin(x) * cos(y) * sin(z));
}

double getOrientationY()
{
    float x = OdomEulerXy.euler_x / 2;
    float y = OdomEulerXy.euler_y / 2;
    float z = OdomEulerZ.euler_z / 2;
    return (sin(x) * cos(y) * cos(z) - cos(x) * sin(y) * sin(z));
}

double getOrientationZ()
{
    float x = OdomEulerXy.euler_x / 2;
    float y = OdomEulerXy.euler_y / 2;
    float z = OdomEulerZ.euler_z / 2;
    return (0 - sin(x) * sin(y) * cos(z) + cos(x) * cos(y) * sin(z));
}

double getOrientationW()
{
    float x = OdomEulerXy.euler_x;
    float y = OdomEulerXy.euler_y;
    float z = OdomEulerZ.euler_z;
    return (cos(x) * cos(y) * cos(z) - sin(x) * sin(y) * sin(z));
}
namespace robot
{

    Chassis::Chassis(const ros::NodeHandle& nh): nh_(nh)
    {
        timestamp_data.on_new_data = PubData;
        aprctrl_datastamped_jni_register(&timestamp_data);

        event_data.event_callback = EvnetPubData;
        aprctrl_eventcallback_jni_register(&event_data);

        velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Chassis::cmd_vel_callback, this);
        enable_sub_ = nh_.subscribe("cmd_enable", 1, &Chassis::cmd_enable_callback, this);
        load_sub_ = nh_.subscribe("cmd_load", 1, &Chassis::cmd_load_callback, this);
        poweroff_sub_ = nh_.subscribe("cmd_poweroff", 1, &Chassis::cmd_poweroff_callback, this);
        vel_max_sub_ = nh_.subscribe("cmd_vel_max", 1, &Chassis::cmd_vel_max_callback, this);
        remove_push_sub_ = nh_.subscribe("cmd_remove_push", 1, &Chassis::cmd_remove_push_callback, this);
        set_hang_cmd_sub_ = nh_.subscribe("set_hang_cmd", 1, &Chassis::cmd_set_hang_callback, this);

        Speed_pub = nh_.advertise<segway_msgs::speed>("speed", 10);
        Ticks_pub = nh_.advertise<segway_msgs::ticks>("ticks", 10);
        Info_pub = nh_.advertise<segway_msgs::info>("info", 1);
        Event_info_pub = nh_.advertise<segway_msgs::event_info>("event_info", 1);         
        Odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);         
        Imu_pub  = nh_.advertise<sensor_msgs::Imu>("imu", 1);  
        Set_Limit_Fb_pub = nh_.advertise<segway_msgs::vel_max_feedback>("vel_max_feedback", 1);   
        hang_mode_fb_pub = nh_.advertise<segway_msgs::set_hang_feedback>("set_hang_feedback", 1);
        
        update_timer_ = nh_.createTimer(ros::Duration(0.001), &Chassis::TimeUpdate1000Hz,this);
        update_timer2_ = nh_.createTimer(ros::Duration(1), &Chassis::TimeUpdate1Hz,this);
    }

    /* code */
    void Chassis::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_input)//订阅/cmd_vel主题回调函数
    {
        angular_vel_ = cmd_input->angular.z ;//get angular velocity of /cmd_vel,rad/s
        linear_vel_ = cmd_input->linear.x ;//get linear velocity of /cmd_vel.m/s

        set_cmd_vel(linear_vel_,angular_vel_);//Configure coefficients according to chassis parameters

        //ROS_INFO("angular_temp:%f  rad/s   linear_temp:%f  m/s", angular_vel_,linear_vel_);
    }

    void Chassis::cmd_enable_callback(const segway_msgs::motor_enable::ConstPtr& msg)
    {
        if (true == msg->motor_enable)
        {
            set_enable_ctrl(1);//need to set motor_enable 1 for chassis movement            
        }
        else
        {
            set_enable_ctrl(0);//need to set motor_enable 1 for chassis movement
        }
        ROS_INFO("cmd_enable:%d", msg->motor_enable);
    }

    void Chassis::cmd_load_callback(const segway_msgs::load_param::ConstPtr& msg)
    {
       static int16_t pre_set_chassis_load_param = -1;
        if (pre_set_chassis_load_param != msg->set_chassis_load_param)
        {
            pre_set_chassis_load_param = msg->set_chassis_load_param;
            //Sets whether the chassis parameters are empty or full load, 0: no_load, 1: full_load
            set_chassis_load_state(msg->set_chassis_load_param);
            ROS_INFO("set_chassis_load_param:%d", msg->set_chassis_load_param);
        }
    }

    void Chassis::cmd_poweroff_callback(const segway_msgs::chassis_poweroff::ConstPtr& msg)
    {
        if (true == msg->chassis_poweroff)
        {
            inform_route_poweroff();//Set two - wheel differential chassis shutdown
        }
    }

    void Chassis::cmd_vel_max_callback(const segway_msgs::vel_max::ConstPtr& msg)
    {
        set_line_forward_max_vel(msg->forward_max_vel);//Set the maximum linear velocity in the direction of advance
        set_line_backward_max_vel(msg->backward_max_vel);//Set the maximum linear velocity in the backward direction
        set_angular_max_vel(msg->angular_max_vel);//Set the maximum angular velocity
        
        ROS_INFO("forward_max_vel:%d, backward_max_vel:%d, angular_max_vel:%d",
                    msg->forward_max_vel, msg->backward_max_vel, msg->angular_max_vel);
    }

    void Chassis::cmd_remove_push_callback(const segway_msgs::remove_push::ConstPtr& msg)
    {
        if (msg->remove_push_cmd == true)
        {
            set_remove_push_cmd();// When the vehicle is in push mode, issue the cancel push mode instruction
            ROS_INFO("remove_push_cmd:%d", msg->remove_push_cmd);
        }
    }

    void Chassis::cmd_set_hang_callback(const segway_msgs::set_hang_cmd::ConstPtr& msg)
    {
         ROS_INFO("set_hang_cmd:%d", msg->setHangCmd);
        if (msg->setHangCmd == 1)
        {
            set_chassis_hang_mode(1);// set the hang mode for chassis 
        }
        else
        {
            set_chassis_hang_mode(0);// cancel the hang mode for chassis 
        }        
    }

    void Chassis::PubOdomToRosOdom(Odometry odom_data){
        // ROS_INFO_STREAM("Odom_data to be published: " << 
        //     odom_data.pose_.orientation  << ","<< odom_data.pose_.x << "," << odom_data.pose_.y);
        ROS_odom.header.stamp = timestamp2rostime(odom_data.TimeStamp);
        ROS_odom.header.frame_id = "Odometry";
        ROS_odom.pose.pose.position.x = odom_data.pose_.x;
        ROS_odom.pose.pose.position.y = odom_data.pose_.y;
        ROS_odom.pose.pose.position.z = 0;
        ROS_odom.pose.pose.orientation.x = 0;
        ROS_odom.pose.pose.orientation.y = 0;
        ROS_odom.pose.pose.orientation.z = sin(odom_data.pose_.orientation/2);
        ROS_odom.pose.pose.orientation.w = cos(odom_data.pose_.orientation/2);
        ROS_odom.twist.twist.linear.x = odom_data.twist_.v_x;
        ROS_odom.twist.twist.linear.y = odom_data.twist_.v_y;
        ROS_odom.twist.twist.linear.z = 0;
        ROS_odom.twist.twist.angular.x = 0;
        ROS_odom.twist.twist.angular.y = 0;
        ROS_odom.twist.twist.angular.z = odom_data.twist_.w_z;
        Odom_pub.publish(ROS_odom);
    }

    void Chassis::PubImuToRosImu(void)
    {
        uint64_t imu_stamp = ImuGyr_TimeStamp > ImuAcc_TimeStamp ? ImuGyr_TimeStamp : ImuAcc_TimeStamp;
        ros_imu.header.seq++;
        ros_imu.header.stamp = timestamp2rostime(imu_stamp);
        ros_imu.header.frame_id = "robot_imu";
        ros_imu.angular_velocity.x = (double)ImuGyrData.gyr[0] / 900.0;//* IMU_ANGULAR_VEL_CONVERT_UINIT;
        ros_imu.angular_velocity.y = (double)ImuGyrData.gyr[1] / 900.0;//* IMU_ANGULAR_VEL_CONVERT_UINIT;
        ros_imu.angular_velocity.z = (double)ImuGyrData.gyr[2] / 900.0;//* IMU_ANGULAR_VEL_CONVERT_UINIT;
        ros_imu.linear_acceleration.x = (double)ImuAccData.acc[0] / 4000.0 * 9.8; // * IMU_LINEAR_VEL_CONVERT_UINIT;
        ros_imu.linear_acceleration.y = (double)ImuAccData.acc[1] / 4000.0 * 9.8; // * IMU_LINEAR_VEL_CONVERT_UINIT;
        ros_imu.linear_acceleration.z = (double)ImuAccData.acc[2] / 4000.0 * 9.8; // * IMU_LINEAR_VEL_CONVERT_UINIT;
        Imu_pub.publish(ros_imu);
        // ROS_INFO("ros_imu:angular_vel:%f  rad/s   linear_acc:%f  m/s2", 
        //             ros_imu.angular_velocity.z, ros_imu.linear_acceleration.x);
    }

    void Chassis::TimeUpdate1000Hz(const ros::TimerEvent& event)
    {      
        if(Speed_update == 1)
        {
            speed.l_speed = SpeedData.l_speed;
            speed.l_speed /= 3600; //change the units from m/h to m/s
            speed.r_speed = SpeedData.r_speed;
            speed.r_speed /= 3600; //change the units from m/h to m/s
            speed.speed_timestamp = Speed_TimeStamp;
            Speed_update = 0;
            Speed_pub.publish(speed);
            // ROS_INFO("###chassis speed, l_speed:%f m/s, r_speed:%f m/s", speed.l_speed, speed.r_speed);
        }

        if(Ticks_update == 1)
        {
            Ticks.l_ticks = TicksData.l_ticks;
            Ticks.r_ticks = TicksData.r_ticks;
            Ticks.ticks_timestamp = Ticks_TimeStamp;
            Ticks_update = 0;
            Ticks_pub.publish(Ticks);
#ifndef ODOM_BY_CHASSIS
            SensorData::Ticks tick_msg(Ticks_TimeStamp, TicksData.l_ticks, TicksData.r_ticks);
            if(robot::Chassis::m_ge_encoder.add_ticks(tick_msg)){
                Odometry odome = robot::Chassis::m_ge_encoder.GetOdometry();
                PubOdomToRosOdom(odome);
            }            
#endif
        }

        if (chassis_event_id != 0)
        {
            Event_info.event_id = chassis_event_id;
            Event_info_pub.publish(Event_info);
        }

        if (ImuGyr_update == 1 && ImuAcc_update == 1)
        {
#ifndef ODOM_BY_CHASSIS
            double yaw_radps = 0;
            yaw_radps = (double)ImuGyrData.gyr[2] / 900;//* IMU_ANGULAR_VEL_CONVERT_UINIT;
            SensorData::BaseImu imu_msg(ImuAcc_TimeStamp,yaw_radps);
            if(robot::Chassis::m_ge_encoder.add_imubase(imu_msg)){
                Odometry odome = robot::Chassis::m_ge_encoder.GetOdometry();
                PubOdomToRosOdom(odome);
            }
#endif
            PubImuToRosImu();

            ImuGyr_update = 0;
            ImuAcc_update = 0;
        }

#ifdef ODOM_BY_CHASSIS
        static uint64_t time_pre = 0;
        if (Odom_update == 15)
        {
            Odom_update = 0;

            ROS_odom.header.stamp = timestamp2rostime(Odom_TimeStamp);
            ROS_odom.header.frame_id = "Odometry";
            ROS_odom.pose.pose.position.x = OdomPoseXy.pos_x;
            ROS_odom.pose.pose.position.y = OdomPoseXy.pos_y;
            ROS_odom.pose.pose.position.z = 0;
            ROS_odom.pose.pose.orientation.x = getOrientationX();
            ROS_odom.pose.pose.orientation.y = getOrientationY();
            ROS_odom.pose.pose.orientation.z = getOrientationZ();
            ROS_odom.pose.pose.orientation.w = getOrientationW();
            ROS_odom.twist.twist.linear.x = OdomVelLineXy.vel_line_x;
            ROS_odom.twist.twist.linear.y = OdomVelLineXy.vel_line_y;
            ROS_odom.twist.twist.linear.z = 0;
            ROS_odom.twist.twist.angular.x = (double)ImuGyrData.gyr[0] / 900.0;
            ROS_odom.twist.twist.angular.y = (double)ImuGyrData.gyr[1] / 900.0;
            ROS_odom.twist.twist.angular.z = (double)ImuGyrData.gyr[2] / 900.0;//* IMU_ANGULAR_VEL_CONVERT_UINIT;
            if ((Odom_TimeStamp - time_pre) > 100000)
            {
                static uint8_t first = 1;
                if (first)
                {
                    first = 0;
                }
                else
                {
                    //uint64_t timegap = (Odom_TimeStamp - time_pre);
                    //printf("!!!!!!!!!! timeout !!!timegap: %lu, curtime:%lu, pretime:%lu\n", timegap, Odom_TimeStamp, time_pre);
                }
            }
            time_pre = Odom_TimeStamp;
            Odom_pub.publish(ROS_odom);
        }
#endif
        ros::spinOnce();
    }

    void Chassis::TimeUpdate1Hz(const ros::TimerEvent& event)
    {
        Info.host_version = get_version(Host);
        Info.host_error = get_err_state(Host);
        Info.motor_version = get_version(Motor);
        Info.motor_error = get_err_state(Motor);
        Info.connect_version = get_version(connect);
        Info.connect_error = get_err_state(connect);
        Info.route_version = get_version(route);
        Info.route_error = get_err_state(route);
        Info.bat_soc = get_bat_soc();
        Info.bat_charging = get_bat_charging();
        Info.bat_vol = get_bat_mvol();
        Info.bat_current = get_bat_mcurrent();
        Info.bat_temp = get_bat_temp();
        Info.motor_work_mode = get_chassis_work_model();
        Info.vehicle_meters = get_vehicle_meter();
        Info.chassis_ctrl_cmd_src = get_ctrl_cmd_src();
        Info.chassis_load_param = get_chassis_load_state(); //0: no_load, 1: full_load
        Info.chassis_mode = get_chassis_mode();//0: lock_mode, 1:ctrl_mode, 2:push_mode, 3:emergency mode, 4:error mode

        Info_pub.publish(Info);

        set_hang_fb.hangModeFb = get_chassis_hang_mode();// 1: in the hang_mode; others: not in the hang_mode.
        hang_mode_fb_pub.publish(set_hang_fb);
        
        if (LimitFb_update == 1)
        {
            LimitFb_update = 0;
            Set_Limit_Fb.forward_max_vel_fb  = LimitVelSetFb.set_forward_limit_vel_fb;
            Set_Limit_Fb.backward_max_vel_fb = LimitVelSetFb.set_backward_limit_vel_fb;
            Set_Limit_Fb.angular_max_vel_fb  = LimitVelSetFb.set_angular_limit_vel_fb;
            Set_Limit_Fb.fb_timestamp = LimitFb_TimeStamp;

            Set_Limit_Fb_pub.publish(Set_Limit_Fb);
        }

        ros::spinOnce();
    }
}
#endif
