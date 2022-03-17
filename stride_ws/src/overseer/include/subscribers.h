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
#include <nav_msgs/Odometry.h>
#include <microstrain_inertial_msgs/FilterHeading.h>
#include <microstrain_inertial_msgs/GNSSFixInfo.h>
#include <microstrain_inertial_msgs/GNSSDualAntennaStatus.h>
#include <can_interface/WheelRPM.h>
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
    uint32_t status;
    uint32_t drive_status;
    uint8_t gnss1_info;
    uint8_t gnss2_info;
    uint8_t dual_antenna_info;
    float heading_uncertainty;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    float east_m;
    float north_m;
    float vel_long_ms;
    float vel_lat_ms;
    float vel_east_ms;
    float vel_north_ms;
    float heading_deg;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    double acc_x_mss;
    double acc_y_mss;
    double acc_z_mss;
    float yaw_rate_rads;
    float goal_east_m;
    float goal_north_m;
    uint8_t lookahead_m;
    float desired_steering_deg;
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
    uint16_t batt_voltage;
    uint16_t batt_amp;
    uint8_t batt_soc;
    uint8_t batt_temp;
    uint8_t robot_temp;
} DataFrame;

class MotorInfoSub {
private:
    std::string motor_name_;
    ros::NodeHandle nh_;
    ros::Subscriber motor_current_sub_;
    ros::Subscriber motor_rpm_sub_;
    float current_;
    float rpm_;

public:
    MotorInfoSub(ros::NodeHandle* nh, std::string name);
    ~MotorInfoSub();
    void MotorCurrentCallback(const std_msgs::Float32::ConstPtr& msg);
    void MotorRpmCallback(const std_msgs::Float32::ConstPtr& msg);
    float GetCurrent();
    float GetRpm();
};


class DataRecorderSub {
private:
// Subscribers
    ros::NodeHandle nh_;
    ros::Subscriber gps_odom_sub_;
    ros::Subscriber gps_heading_sub_;
    ros::Subscriber gps_imu_sub_;
    ros::Subscriber overseer_states_sub_;
    ros::Subscriber record_cmd_sub_;
    ros::Subscriber motors_rpm_cmd_sub_;
    ros::Subscriber gnss1_info_sub_;
    ros::Subscriber gnss2_info_sub_;
    ros::Subscriber dual_antenna_info_sub_;
    std::shared_ptr<MotorInfoSub> motor_RL;
    std::shared_ptr<MotorInfoSub> motor_RR;
    std::shared_ptr<MotorInfoSub> motor_FL;
    std::shared_ptr<MotorInfoSub> motor_FR;
    DataFrame df_;
    std::ofstream wf;

    bool recording = false;
    bool record_command_on = false;
    bool convert_to_csv = true;
    double time_since_stop = ros::Time::now().toSec();


    std::string csv_header[47] = {"utc_time(millisec)", "general_status", "drive_status", "gnss1", "gnss2", "dual_gnss", "heading_unc",
                                "latitude(deg)", "longitude(deg)", "altitude(m)", "east(m)", "north(m)",
                                "vel_longitudinal(m/s)", "vel_lateral(m/s)",
                                "vel_east(m/s)", "vel_north(m/s)", "heading(deg)", "roll(deg)", "pitch(deg)", "yaw(deg",
                                "Ax(m/s^2)", "Ay(m/s^2)","Az(m/s^2)", "yaw_rate(rad/s)",
                                "goal_east(m)", "goal_north(m)", "lookahead(m)", 
                                "desired_steering(deg)",
                                "desired_velocity(ms)", 
                                "actual_rpm_RL", "desired_rpm_RL", 
                                "actual_rpm_RR", "desired_rpm_RR",
                                "actual_rpm_FL", "desired_rpm_FL",
                                "actual_rpm_FR", "desired_rpm_FR",
                                "actual_current_RL(A)", "actual_current_RR(A)",
                                "actual_current_FL(A)", "actual_current_FR(A)",
                                "battery_voltage(V)", "battery_amp(A)", "battery_soc(%)", "battery_temp(C)", "robot_temp(C)"};

public:    
    std::string export_path = "";

    //Constructor
    DataRecorderSub(ros::NodeHandle* nh);

    // Destructor
    ~DataRecorderSub();

    // Initialize subscribers
    void InitializeSubscribers();

    // Subscribing functions
    void GpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void GpsImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void GpsHeadingCallback(const microstrain_inertial_msgs::FilterHeading::ConstPtr& msg);
    void OverseerCallback(const std_msgs::Int32::ConstPtr& msg);
    void RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg);
    void MotorsRpmCmdCallback(const can_interface::WheelRPM::ConstPtr& msg);
    void Gnss1InfoCallback(const microstrain_inertial_msgs::GNSSFixInfo::ConstPtr& msg);
    void Gnss2InfoCallback(const microstrain_inertial_msgs::GNSSFixInfo::ConstPtr& msg);
    void DualAntennaInfoCallback(const microstrain_inertial_msgs::GNSSDualAntennaStatus::ConstPtr& msg);

    // Write csv data
    void WriteBinary();
    void ConvertBin2Csv();
    bool GetRecordingStatus();
    bool GetConvertToCsvStatus();
    void UpdateConvertToCsvStatus(bool status);
    int SetupRecording();
};

#endif