// ========================================================================
// Copyright (c) 2022, SEA Ltd.
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 
// ========================================================================

#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include <iostream>
#include <memory>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <can_interface/WheelRPM.h>
#include <sbg_driver/SbgEkfEuler.h>
#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgGpsPos.h>
#include <sbg_driver/SbgImuData.h>
#include <external_interface/TargetVehicle.h>
#include <tf/tf.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <fstream>

// overseer mode
#define MANUAL 1
#define AUTO 2
#define E_STOPPED 3
#define STOPPED 5
#define DESCENDING 6
#define IDLE 7

typedef struct
{
    int64_t utc_time_millisec;
    uint8_t gnss_no_satellites;
    uint16_t diff_age;
    uint8_t RTK_status;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    float vel_forward_ms;
    float vel_lateral_ms;
    float vel_east_ms;
    float vel_north_ms;
    float vel_z_ms;
    float heading_deg;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    double acc_x;
    double acc_y;
    double acc_z;
    float yaw_rate_rads;
    float roll_rate_deg;
    float pitch_rate_deg;
    float cross_track_error_m;
    float desired_omega_rads;
    float desired_velocity_ms;
    float motor_velocity_RL_rpm;
    float motor_velocity_RR_rpm;
    float motor_velocity_FL_rpm;
    float motor_velocity_FR_rpm;
    float motor_current_RL_raw;
    float motor_current_RR_raw;
    float motor_current_FL_raw;
    float motor_current_FR_raw;
    float desired_motor_velocity_RL_rpm;
    float desired_motor_velocity_RR_rpm;
    float desired_motor_velocity_FL_rpm;
    float desired_motor_velocity_FR_rpm;
    float desired_adj_L_rpm;
    float desired_adj_R_rpm;
    int motor_winding_temp_RL;
    int motor_winding_temp_RR;
    int motor_winding_temp_FL;
    int motor_winding_temp_FR;
    int motor_error_code_RL;
    int motor_error_code_RR;
    int motor_error_code_FL;
    int motor_error_code_FR;
    float batt_voltage;
    uint16_t batt_amp;
    uint8_t batt_soc;
    int batt_temp;
    uint8_t robot_temp;
    float vehicle_speed;
    float vehicle_latitude;
    float vehicle_longitude;
    float vehicle_heading;
    bool brake_command;
    int brake_status;
    int fully_seated_L;
    int fully_seated_R;
    bool disable_motors;
   
} DataFrame;

class MotorInfoSub {
private:
    std::string motor_name_;
    ros::NodeHandle nh_;
    ros::Subscriber motor_current_sub_;
    ros::Subscriber motor_rpm_sub_;
    ros::Subscriber motor_temp_sub_;
    ros::Subscriber motor_error_sub_;
    float current_;
    float rpm_;
    int temperature_;
    uint16_t error_;
    uint8_t heart_beat_; 

public:
    MotorInfoSub(ros::NodeHandle* nh, std::string name);
    ~MotorInfoSub();
    void MotorCurrentCallback(const std_msgs::Float32::ConstPtr& msg);
    void MotorRpmCallback(const std_msgs::Float32::ConstPtr& msg);
    void MotorWindingTempCallback(const std_msgs::Int32::ConstPtr& msg);
    void MotorErrorCallback(const std_msgs::Int32::ConstPtr& msg);
    float GetCurrent();
    float GetRpm();
    int GetTemperature();
    uint16_t GetError();
};

class DataRecorderSub {
private:
// Subscribers
    ros::NodeHandle nh_;
    ros::Publisher csv_converted_;

    ros::Subscriber gps_odom_sub_;
    ros::Subscriber gps_heading_sub_;
    ros::Subscriber gps_imu_sub_;
    ros::Subscriber overseer_states_sub_;
    ros::Subscriber record_cmd_sub_;
    ros::Subscriber motors_rpm_cmd_sub_;
    ros::Subscriber robot_temperature_sub_;
    ros::Subscriber battery_voltage_sub_;
    ros::Subscriber battery_temperature_sub_;
    ros::Subscriber target_vehicle_sub_;

    // Sbg
    ros::Subscriber sbg_gps_nav_sub_;
    ros::Subscriber sbg_gps_euler_sub_;
    ros::Subscriber sbg_gps_gnss_sub_;
    ros::Subscriber sbg_gps_imu_sub_;

    ros::Subscriber dual_antenna_info_sub_;
    ros::Subscriber desired_velocity_sub_;
    ros::Subscriber cross_track_error_sub_;
    std::shared_ptr<MotorInfoSub> motor_RL;
    std::shared_ptr<MotorInfoSub> motor_RR;
    std::shared_ptr<MotorInfoSub> motor_FL;
    std::shared_ptr<MotorInfoSub> motor_FR;
    DataFrame df_;
    std::ofstream wf;

    //Mechanical Brake
    ros::Subscriber brake_command_sub_;
    ros::Subscriber brake_status_sub_;
    ros::Subscriber fully_seated_L_sub_;
    ros::Subscriber fully_seated_R_sub_;
    ros::Subscriber disable_motors_sub_;

    bool recording = false;
    bool record_command_on = false;
    bool convert_to_csv = true;
    double time_since_stop = ros::Time::now().toSec();


    std::string csv_header[60] = {"utc_time(ms)", 
                                "gnss_satellites", "diff_age(ms)", "RTK_status",
                                "latitude(deg)", "longitude(deg)", "altitude(m)", 
                                "vel_forward(m/s)", "vel_lateral(m/s)",
                                "vel_east(m/s)", "vel_north(m/s)", "vel_z(m/s)", "heading(deg)", "roll(deg)", "pitch(deg)",
                                "Ax(g)", "Ay(g)","Az(g)", "yaw_rate(rad/s)", "roll_rate(deg/s)", "pitch_rate(deg/s)", 
                                "cte(m)", 
                                "desired_omega(rad/s)", "desired_velocity(m/s)", 
                                "actual_rpm_RL", "desired_rpm_RL", 
                                "actual_rpm_RR", "desired_rpm_RR",
                                "actual_rpm_FL", "desired_rpm_FL",
                                "actual_rpm_FR", "desired_rpm_FR",
                                "adj_rpm_L", "adj_rpm_R",
                                "actual_current_RL(A)", "actual_current_RR(A)", "actual_current_FL(A)", "actual_current_FR(A)",
                                "winding_temp_RL(F)", "winding_temp_RR(F)", "winding_temp_FL(F)", "winding_temp_FR(F)",
                                "error_code_RL", "error_code_RR", "error_code_FL", "error_code_FR", 
                                "battery_voltage(V)", "battery_temp(F)", "robot_temp(F)",
                                "vehicle_speed(m/s)", "vehicle_latitude(deg)", "vehicle_longitude(deg)", "vehicle_heading(deg)",
                                "brake_command", "brake_status", "Left_Brake_fullyseated", "Right_Brake_fullyseated", "disable_motors" };

public:    
    std::string export_path = "";

    //Constructor
    DataRecorderSub(ros::NodeHandle* nh);

    // Destructor
    ~DataRecorderSub();

    // Initialize subscribers
    void InitializeSubscribers();

    // Subscribing functions
    void OverseerCallback(const std_msgs::Int32::ConstPtr& msg);
    void RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg);
    void MotorsRpmCmdCallback(const can_interface::WheelRPM::ConstPtr& msg);
    void DesiredVelocityCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void CrossTrackErrorCallback(const std_msgs::Float32::ConstPtr& msg);
    void RobotTemperatureCallback(const std_msgs::Int32::ConstPtr& msg);
    void BatteryVoltageCallback(const std_msgs::Float32::ConstPtr& msg);
    void BatteryTemperatureCallback(const std_msgs::Int32::ConstPtr& msg);
    void TargetVehicleCallback(const external_interface::TargetVehicle::ConstPtr& msg);
    void BrakeCommandCallback(const std_msgs::Bool::ConstPtr& msg);
    void BrakeStatusCallback(const std_msgs::Int32::ConstPtr& msg);
    void LeftBrakeCallback(const std_msgs::Int32::ConstPtr& msg);
    void RightBrakeCallback(const std_msgs::Int32::ConstPtr& msg);
    void DisableMotorsCallback(const std_msgs::Bool::ConstPtr& msg);

    // Sbg
    void SbgGpsNavCallback(const sbg_driver::SbgEkfNav::ConstPtr& msg);
    void SbgGpsEulerCallback(const sbg_driver::SbgEkfEuler::ConstPtr& msg);
    void SbgGpsGnnsCallback(const sbg_driver::SbgGpsPos::ConstPtr& msg);
    void SbgGpsImuCallback(const sbg_driver::SbgImuData::ConstPtr& msg);

    // Write csv data
    void WriteBinary();
    void ConvertBin2Csv();
    bool GetRecordingStatus();
    bool GetConvertToCsvStatus();
    void UpdateConvertToCsvStatus(bool status);
    int SetupRecording();
};

#endif