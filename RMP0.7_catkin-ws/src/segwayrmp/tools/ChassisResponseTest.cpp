/*
   test chassis response
*/
#if 1
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "segwayrmp/robot.h"


using namespace std;

namespace {

    enum TestCase {
        swing = 1,
        circleleft = 2,
        circleright = 3,
    };

    struct SwingParam {
        float max_run_time = 60.0;
        float max_vel = 1.0;
    } swing_param;

    struct CircleleftParam {
        float max_run_time = 60.0;
        float max_angular = 0.3;
        float max_vel = 0;
    } circleleft_param;

    struct CirclerightParam {
        float max_run_time = 60.0;
        float max_angular = -0.3;
        float max_vel = 0;
    } circleright_param;


    int rate = 100;
    int select_case = -1;
    static float cur_time = 0;
    geometry_msgs::Twist cmd_vel;
    segway_msgs::motor_enable cmd_enable;
    static float pre_time = 0;
    static float temp_target_speed = swing_param.max_vel;
    static float temp_middle_value = temp_target_speed;

    void showHelp(int &select_case) {
        cout << "==================================" << endl;
        cout << "     Chassis Response Test Tool   " << endl;
        cout << "==================================" << endl;
        cout << "0. Exit." << endl;
        cout << "1. Test chassis swing back and forth for 1 minutes." << endl;
        cout << "2. Test chassis circle to the left for 1 minutes." << endl;
        cout << "3. Test chassis Circle to the right for 1 minutes." << endl;
        cout << "Please select test case num 0~3 : ";

        cin >> select_case;
        cout << "select_case  " << select_case << endl;
        cur_time = 0;
        pre_time = 0;
        temp_target_speed = swing_param.max_vel;
        temp_middle_value = temp_target_speed;
        if (select_case == 0) {
            cout << "shutdown!" << endl;
            ros::shutdown();
        }
    }

    void responseTest() {
        switch (select_case) {
            case TestCase::swing :
                if (cur_time < swing_param.max_run_time) {
                    if ((cur_time - pre_time) > 2.5) {
                        //swing_param.max_vel = -swing_param.max_vel;                        
                        temp_middle_value = -temp_middle_value;//swing_param.max_vel;
                        temp_target_speed = temp_middle_value;
                        pre_time = cur_time;
                    }
                    else if ((cur_time - pre_time) > 2.0) {                        
                        temp_target_speed = 0;
                    }                    
                    
                    cmd_vel.linear.x = temp_target_speed;
                    cmd_vel.angular.z = 0;
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("swing cur_time %.2f, linear_vel %.2f angular_vel %.2f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::circleleft :
                if (cur_time < circleleft_param.max_run_time) {
                    cmd_vel.linear.x = circleleft_param.max_vel;
                    cmd_vel.angular.z = circleleft_param.max_angular;// / circleleft_param.max_run_time;
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("circleleft cur_time %.2f, linear_vel %.2f angular_vel %.2f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::circleright :
                if (cur_time < circleright_param.max_run_time) {
                    cmd_vel.linear.x = circleright_param.max_vel;
                    cmd_vel.angular.z = circleright_param.max_angular;
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("circleright cur_time %.2f, linear_vel %.2f angular_vel %.2f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;      
            default:
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                showHelp(select_case);
                cur_time = 0;
                break;
        }
        cur_time += 1.0f / rate;
    }

}

int main(int argc, char **argv) {
    // Initiate ROS
    ros::init(argc, argv, "chassis_response_test");
    ros::NodeHandle n_;
    
    std::vector<std::string> names;
    if (n_.getParamNames(names))    {
        for (std::string name : names){
            printf("%s\n", name.c_str());
        }
    }

    n_.setParam("segwaySmartCarSerial", "ttyUSB0");

    //chassis
    ros::Publisher cmd_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher cmd_enable_pub = n_.advertise<segway_msgs::motor_enable>("/cmd_enable", 1);
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        responseTest();
        cmd_enable.motor_enable = true;
        cmd_enable_pub.publish(cmd_enable);
        cmd_pub.publish(cmd_vel);
        loop_rate.sleep();
    }
    return 0;
}
#endif
