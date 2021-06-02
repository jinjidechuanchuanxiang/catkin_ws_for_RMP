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
        snake = 2,
        circleleft = 3,
        circleright = 4,
        stop = 5,
        overspeed = 6,
        overturnspeed = 7,
        velTracking1    = 11,        
        velTracking2    = 12,
        velTracking3    = 13,
        velTracking4    = 14,
        velTracking5    = 15,
        velTracking6    = 16,
        brakingDis1     = 17,
        brakingDis2     = 18,
        brakingDis3     = 19,
        moveTest        = 21,
        moveTest1        = 22,
        moveTest2        = 23,
        moveTest3        = 24,
        moveTest4        = 25,
        moveTest5        = 26,
        moveTest6        = 27,
        moveTurnTest        = 31,
        moveTurnTest1        = 32,
        moveTurnTest2        = 33,
        moveTurnTest3        = 34,
        moveTurnTest4        = 35,
        moveTurnTest5        = 36,
        moveTurnTest6        = 37,
        disable_cmd          = 41,
        setChaasisHangMode = 42,
        cancelChaasisHangMode = 43,
    };

    struct SwingParam {
        float max_run_time = 4200.0;
        float max_vel = 1.0;
    } swing_param;

    struct SnakeParam {
        float max_run_time = 12.0;
        float max_angular = 0.25;
        float max_vel = 0.5;
    } snake_param;

    struct CircleleftParam {
        float max_run_time = 15.0;
        float max_angular = 0.3;
        float max_vel = 0;
    } circleleft_param;

    struct CirclerightParam {
        float max_run_time = 4200.0;
        float max_angular = -0.3;
        float max_vel = 0;
    } circleright_param;

    struct StopParam {
        float max_run_time = 5.0;
    } stop_param;

    struct OverspeedParam {
        float max_run_time = 4.0;
        float max_vel = 4;
    } overspeed_param;

    struct OverturnspeedParam {
        float max_run_time = 4.0;
        float max_angular = 4;
    } overturnspeed_param;

    typedef struct {
        float target;
        float acc;
        float dec;
    } trackingParam;
    trackingParam tracking_param1 = {
        .target = 1,
        .acc = 0.5,
        .dec = 0.5,
    };
    trackingParam tracking_param2 = {
        .target = 2,
        .acc = 0.5,
        .dec = 0.5,
    };
    trackingParam tracking_param3 = {
        .target = 2,
        .acc = 1,
        .dec = 1,
    };
    trackingParam tracking_param4 = {
        .target = 3,
        .acc = 1.5,
        .dec = 1.5,
    };
    trackingParam tracking_param5 = {
        .target = 3,
        .acc = 1.5,
        .dec = 2,
    };
    trackingParam tracking_param6 = {
        .target = 3,
        .acc = 1.5,
        .dec = 3,
    };

    int rate = 100;
    int select_case = -1;
    static float cur_time = 0;

    ros::Publisher cmd_pub;
    ros::Publisher cmd_enable_pub;
    ros::Publisher cmd_set_hang_pub;

    geometry_msgs::Twist cmd_vel;
    segway_msgs::motor_enable cmd_enable;
    segway_msgs::set_hang_cmd setHangCmd;
    static float pre_time = 0;

    void showHelp(int &select_case) {
        cout << "==================================" << endl;
        cout << "     Chassis Response Test Tool   " << endl;
        cout << "==================================" << endl;
        cout << "0. Exit." << endl;
        cout << "1. Test chassis swing back and forth for 70 minutes." << endl;
        cout << "2. Test chassis for a snake forward for 10 seconds." << endl;
        cout << "3. Test chassis circle to the left for 70 minutes." << endl;
        cout << "4. Test chassis Circle to the right for 70 minutes." << endl;
        cout << "5. Test mode: stop." << endl;
        cout << "6. Test overspeed for chassis max speed, running for 3 seconds." << endl;
        cout << "7. Test overturnspeed for chassis max turn speed, running for 3 seconds." << endl;
        cout << "Please select test case num 0~7 : "<< endl;        
        cout << "------------------------------------I'm the cutting line------------------------------------ "<< endl;
        cout << "11. Test speed tracking, accelerate from 0 to 1 at 0.5m/s2, then decelarate to 0 at -0.5m/s2." << endl;
        cout << "12. Test speed tracking, accelerate from 0 to 2 at 0.5m/s2, then decelarate to 0 at -0.5m/s2." << endl;
        cout << "13. Test speed tracking, accelerate from 0 to 2 at 1m/s2, then decelarate to 0 at -1m/s2." << endl;
        cout << "14. Test speed tracking, accelerate from 0 to 3 at 1.5m/s2, then decelarate to 0 at -1.5m/s2." << endl;
        cout << "15. Test speed tracking, accelerate from 0 to 3 at 1.5m/s2, then decelarate to 0 at -2m/s2." << endl;
        cout << "16. Test speed tracking, accelerate from 0 to 3 at 1.5m/s2, then decelarate to 0 at -3m/s2." << endl;
        cout << "17. Test ultimate braking distance, accelerate to 1m/s, then brake down to 0." << endl;
        cout << "18. Test ultimate braking distance, accelerate to 2m/s, then brake down to 0." << endl;
        cout << "19. Test ultimate braking distance, accelerate to 3m/s, then brake down to 0." << endl;
        cout << "Please select test case num 11~19 : "<< endl;
        cout << "21. Test small velocity movement, The ten velocities change in turn, each lasting 10 seconds. such as 0.02/0.04/0.05/0.06/0.08/0.10 m/s." << endl;
        cout << "22. Test small velocity movement, 0.02m/s 10s." << endl;
        cout << "23. Test small velocity movement, 0.025m/s 10s." << endl;
        cout << "24. Test small velocity movement, 0.03m/s 10s." << endl;
        cout << "25. Test small velocity movement, 0.035m/s 10s." << endl;
        cout << "26. Test small velocity movement, 0.04m/s 10s." << endl;
        cout << "27. Test small velocity movement, 0.05m/s 10s." << endl;
        cout << "Please select test case num 21~27 : "<< endl;
        cout << "31. Test small angular velocity movement, The ten velocities change in turn, each lasting 10 seconds. such as 0.01/0.02/0.03/0.04/0.05/0.06/0.07/0.08/0.09/0.10 rad/s." << endl;
        cout << "32. Test small angular velocity movement, 0.01rad/s 10s." << endl;
        cout << "33. Test small angular velocity movement, 0.02rad/s 10s." << endl;
        cout << "34. Test small angular velocity movement, 0.03rad/s 10s." << endl;
        cout << "35. Test small angular velocity movement, 0.04rad/s 10s." << endl;
        cout << "36. Test small angular velocity movement, 0.05rad/s 10s." << endl;
        cout << "37. Test small angular velocity movement, 0.06rad/s 10s." << endl;
        cout << "Please select test case num 31~37 : " << endl;
        cout << "41. Test host eanble, hold 10s, in 0m/s，then host diable." << endl;
        cout << "Please select test case num 41 : ";

        cin >> select_case;
        cout << "select_case  " << select_case << endl;
        cur_time = 0;
        pre_time = 0;
        if (select_case == 0) {
            cout << "shutdown!" << endl;
            ros::shutdown();
        }
    }

    void responseTest() {
        switch (select_case) {
            case TestCase::swing :
                static float temp_target_speed = swing_param.max_vel;
                if (cur_time < swing_param.max_run_time) {
                    if ((cur_time - pre_time) > 4.5) {
                        swing_param.max_vel = -swing_param.max_vel;                        
                        temp_target_speed = swing_param.max_vel;
                        pre_time = cur_time;
                    }
                    else if ((cur_time - pre_time) > 4.0) {                        
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
            case TestCase::snake :     
                static float temp_target_angular = 0;     
                static float pre_time_snake = 0;      
                if (cur_time < snake_param.max_run_time) {
                    if (cur_time < 1.0) {
                        temp_target_angular = snake_param.max_angular;                        
                    } 
                    else if (cur_time >0.99 && cur_time < 1.01){
                        pre_time_snake = cur_time;
                        temp_target_angular = -snake_param.max_angular;
                    }
                    else{                        
                        if ((cur_time - pre_time_snake) > 2.0){
                            temp_target_angular = -temp_target_angular;
                            pre_time_snake = cur_time;
                        }                        
                    } 
                    cmd_vel.angular.z = temp_target_angular;
                    cmd_vel.linear.x = snake_param.max_vel;
                    
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                    pre_time_snake = 0;
                }
                ROS_INFO("snake cur_time %.2f, pre_time_snake:%.2f, linear_vel %.2f angular_vel %.2f",
                         cur_time, pre_time_snake, cmd_vel.linear.x, cmd_vel.angular.z);

                break;
            case TestCase::circleleft :
                if (cur_time < circleleft_param.max_run_time) {
                    cmd_vel.linear.x = circleleft_param.max_vel;
                    cmd_vel.angular.z = circleleft_param.max_angular;// / circleleft_param.max_run_time;
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    //select_case = -1;
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
            case TestCase::stop :
                if (cur_time < stop_param.max_run_time) {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;

                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("stop cur_time %.2f, linear_vel %.2f angular_vel %.2f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::overspeed:
                if (cur_time < overspeed_param.max_run_time) {
                    cmd_vel.linear.x = overspeed_param.max_vel;
                    cmd_vel.angular.z = 0;
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("overspeed cur_time %.2f, linear_vel %.2f angular_vel %.2f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::overturnspeed:
                if (cur_time < overturnspeed_param.max_run_time) {
                    cmd_vel.angular.z = overturnspeed_param.max_angular;
                    cmd_vel.linear.x = 0;
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("overturnspeed cur_time %.2f, linear_vel %.2f angular_vel %.2f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::velTracking1:
                if (cur_time < tracking_param1.target / tracking_param1.acc) {
                    cmd_vel.linear.x += tracking_param1.acc / rate;
                    cmd_vel.angular.z = 0;
                } else if (cur_time < (tracking_param1.target / tracking_param1.acc + 0.5f)){
                    ;//do_nothing, keep vel for 0.5 seconds
                }else if (cmd_vel.linear.x > 0){
                    cmd_vel.linear.x -= tracking_param1.dec / rate;
                    cmd_vel.angular.z = 0;                    
                }
                else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }

                if (cmd_vel.linear.x < 0)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("velTracking1 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::velTracking2:
                if (cur_time < tracking_param2.target / tracking_param2.acc) {
                    cmd_vel.linear.x += tracking_param2.acc / rate;
                    cmd_vel.angular.z = 0;
                } else if (cur_time < (tracking_param2.target / tracking_param2.acc + 0.5f)){
                    ;//do_nothing, keep vel for 0.5 seconds
                }else if (cmd_vel.linear.x > 0){
                    cmd_vel.linear.x -= tracking_param2.dec / rate;
                    cmd_vel.angular.z = 0;                    
                }
                else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }

                if (cmd_vel.linear.x < 0)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("velTracking2 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::velTracking3:
                if (cur_time < tracking_param3.target / tracking_param3.acc) {
                    cmd_vel.linear.x += tracking_param3.acc / rate;
                    cmd_vel.angular.z = 0;
                } else if (cur_time < (tracking_param3.target / tracking_param3.acc + 0.5f)){
                    ;//do_nothing, keep vel for 0.5 seconds
                }else if (cmd_vel.linear.x > 0){
                    cmd_vel.linear.x -= tracking_param3.dec / rate;
                    cmd_vel.angular.z = 0;                    
                }
                else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }

                if (cmd_vel.linear.x < 0)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("velTracking3 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::velTracking4:
                if (cur_time < tracking_param4.target / tracking_param4.acc) {
                    cmd_vel.linear.x += tracking_param4.acc / rate;
                    cmd_vel.angular.z = 0;
                } else if (cur_time < (tracking_param4.target / tracking_param4.acc + 0.5f)){
                    ;//do_nothing, keep vel for 0.5 seconds
                }else if (cmd_vel.linear.x > 0){
                    cmd_vel.linear.x -= tracking_param4.dec / rate;
                    cmd_vel.angular.z = 0;                    
                }
                else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }

                if (cmd_vel.linear.x < 0)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("velTracking4 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::velTracking5:
                if (cur_time < tracking_param5.target / tracking_param5.acc) {
                    cmd_vel.linear.x += tracking_param5.acc / rate;
                    cmd_vel.angular.z = 0;
                } else if (cur_time < (tracking_param5.target / tracking_param5.acc + 0.5f)){
                    ;//do_nothing, keep vel for 0.5 seconds
                }else if (cmd_vel.linear.x > 0){
                    cmd_vel.linear.x -= tracking_param5.dec / rate;
                    cmd_vel.angular.z = 0;                    
                }
                else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }

                if (cmd_vel.linear.x < 0)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("velTracking5 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::velTracking6:
                if (cur_time < tracking_param6.target / tracking_param6.acc) {
                    cmd_vel.linear.x += tracking_param6.acc / rate;
                    cmd_vel.angular.z = 0;
                } else if (cur_time < (tracking_param6.target / tracking_param6.acc + 0.5f)){
                    ;//do_nothing, keep vel for 0.5 seconds
                }else if (cmd_vel.linear.x > 0){
                    cmd_vel.linear.x -= tracking_param6.dec / rate;
                    cmd_vel.angular.z = 0;                    
                }
                else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }

                if (cmd_vel.linear.x < 0)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("velTracking6 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);
                break;
            case TestCase::brakingDis1:
                if (cur_time < 2)
                {
                    cmd_vel.linear.x = 1;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("brakingDis1 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);                
                break;
            case TestCase::brakingDis2:
                if (cur_time < 2)
                {
                    cmd_vel.linear.x = 2;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("brakingDis2 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);   
                break;
            case TestCase::brakingDis3:
                if (cur_time < 2)
                {
                    cmd_vel.linear.x = 3;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("brakingDis3 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTest:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.02;
                    cmd_vel.angular.z = 0;
                }
                else if (cur_time < 20)
                {
                    cmd_vel.linear.x = 0.04;
                    cmd_vel.angular.z = 0;
                }
                else if (cur_time < 30)
                {
                    cmd_vel.linear.x = 0.05;
                    cmd_vel.angular.z = 0;
                }
                else if (cur_time < 40)
                {
                    cmd_vel.linear.x = 0.06;
                    cmd_vel.angular.z = 0;
                }
                else if (cur_time < 50)
                {
                    cmd_vel.linear.x = 0.08;
                    cmd_vel.angular.z = 0;
                }
                else if (cur_time < 60)
                {
                    cmd_vel.linear.x = 0.1;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTest1:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.02;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest1 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTest2:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.025;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest2 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTest3:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.03;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest3 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTest4:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.035;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest4 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTest5:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.04;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest5 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTest6:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.05;
                    cmd_vel.angular.z = 0;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest6 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTurnTest:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.01;
                }
                else if (cur_time < 20)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.02;
                }
                else if (cur_time < 30)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.03;
                }
                else if (cur_time < 40)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.04;
                }
                else if (cur_time < 50)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.05;
                }
                else if (cur_time < 60)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.06;
                }
                else if (cur_time < 70)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.07;
                }
                else if (cur_time < 80)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.08;
                }
                else if (cur_time < 90)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.09;
                }
                else if (cur_time < 100)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.1;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTurnTest1:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.01;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest1 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTurnTest2:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.02;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest2 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTurnTest3:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.03;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest3 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTurnTest4:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.04;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest4 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTurnTest5:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.05;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest5 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::moveTurnTest6:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.06;
                }
                else
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest6 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::disable_cmd:
                if (cur_time < 10)
                {
                    cmd_vel.linear.x = 0.01;
                    cmd_vel.angular.z = 0;
                }
                else if (cur_time < 11)
                {
                    cmd_enable.motor_enable = 0;
                    cmd_enable_pub.publish(cmd_enable);
                }
                else
                {                    
                    cmd_vel.linear.x = 0.01;
                    cmd_vel.angular.z = 0;
                    select_case = -1;
                }
                ROS_INFO("moveTest6 cur_time %.4f, linear_vel %.4f angular_vel %.4f",
                         cur_time, cmd_vel.linear.x, cmd_vel.angular.z);  
                break; 
            case TestCase::setChaasisHangMode: 
                if (cur_time < 4)
                {
                    setHangCmd.setHangCmd = 1;
                    cmd_set_hang_pub.publish(setHangCmd);
                }
                else
                {
                    select_case = -1;
                }                
                break;
            case TestCase::cancelChaasisHangMode: 
               if (cur_time < 4)
                {
                    setHangCmd.setHangCmd = 0;
                    cmd_set_hang_pub.publish(setHangCmd);
                }
                else
                {
                    select_case = -1;
                }                
                break;
            default:
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                //showHelp(select_case);
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
    static uint8_t enable = 0;
    //chassis
    cmd_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cmd_enable_pub = n_.advertise<segway_msgs::motor_enable>("/cmd_enable", 1);
    cmd_set_hang_pub = n_.advertise<segway_msgs::set_hang_cmd>("/set_hang_cmd", 1);

    ros::Rate loop_rate(rate);

    showHelp(select_case);
    while (ros::ok()) {
        responseTest();
        if (cmd_vel.linear.x == 0 && cmd_vel.angular.z == 0)
        {
            enable = 0;
        }
        else if ((cmd_vel.linear.x != 0 || cmd_vel.angular.z != 0) && enable == 0)
        {
            enable = 1;
        }
        else
        {
            enable = 2;
        }
        
        if (enable == 1)
        {
            cmd_enable.motor_enable = 1;
            cmd_enable_pub.publish(cmd_enable);
        }
        
        cmd_pub.publish(cmd_vel);
        loop_rate.sleep();
    }
    return 0;
}
#endif
