#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMM_CTRL_NAVIGATION_H
#define COMM_CTRL_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

//--------------------------DATA CALL BACK INDEX--------------------------
#define Chassis_Data_Speed              4
#define Chassis_Data_Ticks              6
#define Chassis_Data_Limit_vel_set_fb   15
#define Chassis_Data_Odom_Pose_xy		16
#define Chassis_Data_Odom_Euler_xy		17
#define Chassis_Data_Odom_Euler_z		18
#define Chassis_Data_Odom_Linevel_xy	19
#define Chassis_Data_Imu_Gyr            20	//IMU Gyroscope data 陀螺仪数据上报
#define Chassis_Data_Imu_Acc            21	//IMU Accelerometer data 加速度计数据上报

//-----------------------Event---------------------------------------------

#define OnEmergeStopEvent           2	//Enter the scram
#define OutEmergeStopEvent          3	//Exit the scram
#define Locked_Rotor_Protect_Event  4   //Locked-rotor protection event
#define CalibPasheCurrentSuccess    20	//Calibration current successful
#define CalibPasheCurrentFaild      21	//Calibration current failed
#define CalibGyroscopeSuccess      	22	//Gyroscope calibration successful
#define CalibGyroscopeFaild        	23	//Gyroscope calibration failed


//---------The proportional coefficient of the callback gyro data------------------
#define CHASSIS_IMU_GYR_VALUE_TRANS_SCALE		900			//900~~32768(MAX value for int16: map table ) * 57.3(degrees for 1 rad) /2000(Gyroscope range: 2000dps)
#define CHASSIS_IMU_ACC_VALUE_TRANS_SCALE		4000		//4000~~32768(MAX value for int16: map table )/8(Accelerometer range: 8g)

#define CHASSIS_LINE_SPEED_SCALE	3600	//The chassis reports the gain of linear velocity
#define CHASSIS_ANGULAR_SPEED_SCALE	1000	//The chassis reports the gain of angular velocity

#define NO_LOAD  	0		// Set the chassis parameters as no-load parameters, The chassis defaults to no load
#define	FULL_LOAD 	1		// Set the chassis parameters as full load parameters

//-----------------------Version V0.6 and above---------------------------
#define LOCK_MODE	0		// Lock the car
#define CTRL_MODE	1		// Control the car
#define PUSH_MODE	2		// push the car
#define EMERG_MODE	3		// emergency
#define ERROR_MODE	4		// Internal error


//-------------------Timestamp------------------------
#define MAX_BASIC_FRM_SZ 0x1F

#pragma pack(1)
typedef struct StampedBasicFrame_{
    uint32_t type_id;                 //Data type number
    uint64_t timestamp;               //Linux timestamp
    char  data[MAX_BASIC_FRM_SZ];     //Chassis specific data
} StampedBasicFrame;
#pragma pack()
typedef void (*h_aprctrl_datastamped_t)(StampedBasicFrame* frm);
typedef void (*h_aprctrl_event_t)(int32_t event_num);

typedef struct {
    h_aprctrl_datastamped_t on_new_data;
}s_aprctrl_datastamped_t;

typedef struct {
    h_aprctrl_event_t event_callback;
}s_aprctrl_event_t;

typedef enum {
    Host = 1,
    Motor = 2,
    route = 3,
    connect = 4 //v0.6无connect板卡
}board_name_e;

typedef enum {
	comu_serial = 0,
	comu_can = 1
}comu_choice_e;

typedef struct{
	int16_t car_speed;//Vehicle linear speed
	int16_t turn_speed;//Turning speed
	int16_t l_speed;//Left wheel speed
	int16_t r_speed;//Right wheel speed
}car_speed_data_t;

typedef struct{
	int32_t l_ticks;//Left wheel ticks
	int32_t r_ticks;//Right wheel ticks
}motor_ticks_t;

typedef struct{
	int16_t gyr[3];
}imu_gyr_original_data_t;

typedef struct{
	int16_t acc[3];
}imu_acc_original_data_t;

typedef struct{
	float pos_x;
	float pos_y;
}odom_pos_xy_t;

typedef struct{
	float pos_z;
}odom_pos_z_t;//RESERVED


typedef struct{
	float euler_x;
	float euler_y;
}odom_euler_xy_t;

typedef struct{
	float euler_z;
}odom_euler_z_t;

typedef struct{
	float vel_line_x;
	float vel_line_y;
}odom_vel_line_xy_t;

typedef struct{
	float vel_line_z;
}odom_vel_line_z_t;//RESERVED

typedef struct{
	int16_t set_forward_limit_vel_fb;
	int16_t set_backward_limit_vel_fb;
	int16_t set_angular_limit_vel_fb;
}set_max_limit_value_fb_t;


void aprctrl_datastamped_jni_register(s_aprctrl_datastamped_t* f);  //The callback registration function
void aprctrl_eventcallback_jni_register(s_aprctrl_event_t* f);     //Event callback registration function

uint16_t get_version(board_name_e board_name);//Gets the software version number
uint32_t get_err_state(board_name_e board_name);//Gets the software error status
int16_t  get_bat_soc(void);//Gets the percentage of battery left
int16_t  get_bat_charging(void);//Gets the charging status of the battery
int32_t  get_bat_mvol(void);//Gets battery voltage, mV
int32_t  get_bat_mcurrent(void);//Gets battery current, mA
int16_t  get_bat_temp(void);// Gets battery temperature
int16_t  get_chassis_work_model(void);//Get the chassis working mode, 1: The motor torque; 0: The motor does not add force
uint32_t get_vehicle_meter(void);//get the meters of the car since chassis power on, unit:meter
uint16_t get_ctrl_cmd_src(void);//Gets the source of the car control command, 0:remote; 1:host
int16_t  get_chassis_load_state(void);//Gets whether the chassis parameters are empty or full load, 0: no_load, 1: full_load
uint16_t get_chassis_mode(void);//Gets chassis mode, 0:lock; 1:ctrl; 2:push; 3:emergency; 4:error.
int16_t  get_chassis_hang_mode(void);//Get chassis hang_mode; 1: in Hang_mode; 0: not in Hand_mode.


void set_cmd_vel(double linear_x,double angular_z);//Set the linear and angular speeds of the vehicle: m/s ; rad/s.
void set_line_forward_max_vel(double linear_forward_max_x);//Set the maximum linear velocity in the direction of advance
void set_line_backward_max_vel(double linear_backward_max_x);//Set the maximum linear velocity in the backward direction
void set_angular_max_vel(double angular_max_z);//Set the maximum angular velocity
void set_enable_ctrl(uint16_t enable_flag);//Set chassis movement enable
void set_calib_chassis_gyro(void);//Calibrate chassis gyro
int  init_control_ctrl(void);//Chassis initialization
void exit_control_ctrl(void);//Chassis software finished running
void set_smart_car_serial(const char * serial_no);//Set the serial port name
void set_comu_interface(comu_choice_e comu_choice); //Set communication interface: 'comu_serial': serial port; 'comu_can': CAN port
void set_chassis_load_state(int16_t newLoadSet);//Sets whether the chassis parameters are empty or full load, 0: no_load, 1: full_load
void inform_route_poweroff(void);//Set two - wheel differential chassis shutdown
void set_remove_push_cmd(void);//An order to remove the push status
void set_chassis_hang_mode(int16_t enterHand);//Set chassis hang_mode; 1: enter the Hang_mode; 0: exit the Hand_mode.

int32_t IapChassisBoard(char * filePath, char * version);
int32_t IapRouteBoard(char * filePath, char * version);
int32_t getIapTotalProgress(void);

#endif

#ifdef __cplusplus
}
#endif
