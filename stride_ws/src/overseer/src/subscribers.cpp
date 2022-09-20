#include <iostream>
#include <subscribers.h>
#include <math.h>
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

DataRecorderSub::DataRecorderSub(ros::NodeHandle* nh): nh_(*nh) {
    // nh_.getParam("General/Reference_Latitude", refLat);
    ros::param::get("~export_file", export_path);
    memset(&df_, 0, sizeof(df_));
    InitializeSubscribers();
}

DataRecorderSub::~DataRecorderSub() {}

void DataRecorderSub::InitializeSubscribers() {
    csv_converted_ = nh_.advertise<std_msgs::Empty>("/csv_converted", 1);

    // Sbg gps
    sbg_gps_nav_sub_   = nh_.subscribe("/sbg/ekf_nav", 1, &DataRecorderSub::SbgGpsNavCallback, this);
    sbg_gps_euler_sub_ = nh_.subscribe("/sbg/ekf_euler", 1, &DataRecorderSub::SbgGpsEulerCallback, this);
    sbg_gps_gnss_sub_  = nh_.subscribe("/sbg/gps_pos", 1, &DataRecorderSub::SbgGpsGnnsCallback, this);
    sbg_gps_imu_sub_   = nh_.subscribe("/sbg/imu_data", 1, &DataRecorderSub::SbgGpsImuCallback, this);
    
    // Robot Info
    overseer_states_sub_ = nh_.subscribe("/overseer/state", 1, &DataRecorderSub::OverseerCallback, this);
    record_cmd_sub_      = nh_.subscribe("/cmd/record", 1, &DataRecorderSub::RecordCommandCallback, this);
    motors_rpm_cmd_sub_  = nh_.subscribe("/wheel_rpm_command", 1, &DataRecorderSub::MotorsRpmCmdCallback, this);
    robot_temperature_sub_  = nh_.subscribe("/robot_temperature", 1, &DataRecorderSub::RobotTemperatureCallback, this);
    desired_velocity_sub_ = nh_.subscribe("/robot_velocity_command", 1, &DataRecorderSub::DesiredVelocityCallback, this);
    battery_voltage_sub_ = nh_.subscribe("/battery_voltage", 1, &DataRecorderSub::BatteryVoltageCallback, this);
    battery_temperature_sub_ = nh_.subscribe("/battery_temperature", 1, &DataRecorderSub::BatteryTemperatureCallback, this);
    cross_track_error_sub_ = nh_.subscribe("/path_follower/cross_track_error", 1, &DataRecorderSub::CrossTrackErrorCallback, this);
    target_vehicle_sub_ = nh_.subscribe("/target", 1, &DataRecorderSub::TargetVehicleCallback, this);
    
    // motor_RL = new MotorInfoSub(&nh_, "left_back");
    motor_RL = std::make_shared<MotorInfoSub>(&nh_, "left_back");
    motor_RR = std::make_shared<MotorInfoSub>(&nh_, "right_back");
    motor_FL = std::make_shared<MotorInfoSub>(&nh_, "left_front");
    motor_FR = std::make_shared<MotorInfoSub>(&nh_, "right_front");
}

void DataRecorderSub::SbgGpsNavCallback(const sbg_driver::SbgEkfNav::ConstPtr& msg) {
    df_.latitude_deg = msg->latitude;
    df_.longitude_deg = msg->longitude;
    df_.altitude_m = msg->altitude;
    df_.vel_east_ms  = msg->velocity.y;
    df_.vel_north_ms = msg->velocity.x;
    df_.vel_z_ms     = msg->velocity.z;
}

void DataRecorderSub::SbgGpsEulerCallback(const sbg_driver::SbgEkfEuler::ConstPtr& msg) {
    df_.yaw_deg = radToDeg(msg->angle.z);
    df_.pitch_deg = radToDeg(msg->angle.y);
    df_.roll_deg = radToDeg(msg->angle.x);
    df_.vel_forward_ms = df_.vel_north_ms * cos(df_.yaw_deg* (M_PI/180)) + df_.vel_east_ms * sin(df_.yaw_deg* (M_PI/180));
    df_.vel_lateral_ms = -df_.vel_north_ms * sin(df_.yaw_deg* (M_PI/180)) + df_.vel_east_ms * cos(df_.yaw_deg* (M_PI/180));
}

void DataRecorderSub::SbgGpsGnnsCallback(const sbg_driver::SbgGpsPos::ConstPtr& msg) {
    df_.gnss_no_satellites = msg->num_sv_used;
    df_.diff_age = msg->diff_age;
    df_.RTK_status = msg->status.type;
}

void DataRecorderSub::SbgGpsImuCallback(const sbg_driver::SbgImuData::ConstPtr& msg) {
    df_.acc_x_mss = msg->accel.x;
    df_.acc_y_mss = msg->accel.y;
    df_.acc_z_mss = msg->accel.z;
    df_.yaw_rate_rads = msg->gyro.z;

}

void DataRecorderSub::RobotTemperatureCallback(const std_msgs::Int32::ConstPtr& msg) {
    df_.robot_temp = msg->data;
}

void DataRecorderSub::OverseerCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (!record_command_on) {
        if (msg->data == AUTO && !recording)
        {
            SetupRecording();
            ROS_INFO("AUTO mode is ON. Start recording.");
        } else if (msg->data != AUTO && recording)
        {
            recording = false;
            time_since_stop = ros::Time::now().toSec();
            ROS_INFO("AUTO mode is OFF. Stop recording.");
        }
    }
}

void DataRecorderSub::DesiredVelocityCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    df_.desired_velocity_ms = msg->x;
    df_.desired_omega_rads = msg->theta;
}

void DataRecorderSub::CrossTrackErrorCallback(const std_msgs::Float32::ConstPtr& msg) {
    df_.cross_track_error_m = msg->data;
}

void DataRecorderSub::RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    record_command_on = msg->data;
    int ret = 0;
    if (record_command_on && !recording) {
        ret = SetupRecording();
        ROS_INFO("Record command is ON. Start recording.");
        return;
    } 

    if (!record_command_on && recording) {
        recording = false;
        ROS_INFO("Record command is OFF. Stop recording.");
    }
}

void DataRecorderSub::MotorsRpmCmdCallback(const can_interface::WheelRPM::ConstPtr& msg) {
    df_.desired_motor_velocity_RL_rpm = msg->left_back;
    df_.desired_motor_velocity_RR_rpm = msg->right_back;
    df_.desired_motor_velocity_FL_rpm = msg->left_front;
    df_.desired_motor_velocity_FR_rpm = msg->right_front;
    df_.desired_adj_L_rpm = msg->adj_left;
    df_.desired_adj_R_rpm = msg->adj_right;
}

void DataRecorderSub::BatteryVoltageCallback(const std_msgs::Float32::ConstPtr& msg) {
    df_.batt_voltage = msg->data;
}

void DataRecorderSub::BatteryTemperatureCallback(const std_msgs::Int32::ConstPtr& msg) {
    df_.batt_temp = msg->data;
}

void DataRecorderSub::TargetVehicleCallback(const external_interface::TargetVehicle::ConstPtr& msg) {
    df_.vehicle_speed = msg -> velocity;
    df_.vehicle_latitude = msg -> latitude;
    df_.vehicle_longitude = msg -> longitude;
    df_.vehicle_heading = msg -> heading;
}

int DataRecorderSub::SetupRecording() {
    wf.open(export_path + "/data.bin", std::ios::out | std::ios::binary);
    if (!wf.is_open()) {
        ROS_WARN("Cannot open file to write. Test may run but data is not being recorded.");
        return -1;
    }
    recording = true;
    convert_to_csv = false;
    ROS_INFO("Started data recording!");
    return 0;
}

bool DataRecorderSub::GetRecordingStatus() {
    return recording;
}

bool DataRecorderSub::GetConvertToCsvStatus() {
    return convert_to_csv;
}

void DataRecorderSub::UpdateConvertToCsvStatus(bool status) {
    convert_to_csv = status;
}

void DataRecorderSub::WriteBinary() {
    if (wf.is_open())
        df_.utc_time_millisec = ros::Time::now().toNSec() * 0.000001;
        df_.motor_velocity_RL_rpm = motor_RL->GetRpm();
        df_.motor_velocity_RR_rpm = motor_RR->GetRpm();
        df_.motor_velocity_FL_rpm = motor_FL->GetRpm();
        df_.motor_velocity_FR_rpm = motor_FR->GetRpm();
        df_.motor_current_RL_raw = motor_RL->GetCurrent();
        df_.motor_current_RR_raw = motor_RR->GetCurrent();
        df_.motor_current_FL_raw = motor_FL->GetCurrent();
        df_.motor_current_FR_raw = motor_FR->GetCurrent();
        df_.motor_winding_temp_RL = motor_RL->GetTemperature();
        df_.motor_winding_temp_RR = motor_RR->GetTemperature();
        df_.motor_winding_temp_FL = motor_FL->GetTemperature();
        df_.motor_winding_temp_FR = motor_FR->GetTemperature();
        wf.write( (char *) &df_, sizeof(DataFrame));
}

void DataRecorderSub::ConvertBin2Csv() {
    if (wf.is_open())
        wf.close();

    std::ifstream inFile (export_path + "/data.bin", std::ios::in | std::ios::binary);

    if (inFile.is_open())
    {
        std::ofstream outFile (export_path + "/data.csv");
        outFile << std::fixed << std::setprecision(10); // set precision for double
        DataFrame temp;
        std::string dem = ",";

        for (auto s: csv_header)
        {
            outFile << s << dem;
        }
        outFile << "\n";

        while (!inFile.eof())
        {
            inFile.read((char*)&temp, sizeof(temp));
            if (!inFile.eof())
            {
                outFile << temp.utc_time_millisec << dem;
                outFile << unsigned(temp.gnss_no_satellites) << dem;
                outFile << temp.diff_age << dem;
                outFile << unsigned(temp.RTK_status) << dem;
                outFile << temp.latitude_deg << dem;
                outFile << temp.longitude_deg << dem;
                outFile << temp.altitude_m << dem;
                outFile << temp.vel_forward_ms << dem;
                outFile << temp.vel_lateral_ms << dem;
                outFile << temp.vel_east_ms << dem;
                outFile << temp.vel_north_ms << dem;
                outFile << temp.vel_z_ms << dem;
                outFile << temp.yaw_deg << dem;
                outFile << temp.roll_deg << dem;
                outFile << temp.pitch_deg << dem;
                outFile << temp.acc_x_mss << dem;
                outFile << temp.acc_y_mss << dem;
                outFile << temp.acc_z_mss << dem;
                outFile << temp.yaw_rate_rads << dem;
                outFile << temp.cross_track_error_m << dem;
                outFile << temp.desired_omega_rads << dem;
                outFile << temp.desired_velocity_ms << dem;
                outFile << int(temp.motor_velocity_RL_rpm) << dem;
                outFile << int(temp.desired_motor_velocity_RL_rpm) << dem;
                outFile << int(temp.motor_velocity_RR_rpm) << dem;
                outFile << int(temp.desired_motor_velocity_RR_rpm) << dem;
                outFile << int(temp.motor_velocity_FL_rpm) << dem;
                outFile << int(temp.desired_motor_velocity_FL_rpm) << dem;
                outFile << int(temp.motor_velocity_FR_rpm) << dem;
                outFile << int(temp.desired_motor_velocity_FR_rpm) << dem;
                outFile << int(temp.desired_adj_L_rpm) << dem;
                outFile << int(temp.desired_adj_R_rpm) << dem;
                outFile << temp.motor_current_RL_raw << dem;
                outFile << temp.motor_current_RR_raw << dem;
                outFile << temp.motor_current_FL_raw << dem;
                outFile << temp.motor_current_FR_raw << dem;
                outFile << temp.motor_winding_temp_RL << dem;
                outFile << temp.motor_winding_temp_RR << dem;
                outFile << temp.motor_winding_temp_FL << dem;
                outFile << temp.motor_winding_temp_FR << dem;
                outFile << temp.batt_voltage << dem;
                outFile << temp.batt_temp << dem;
                outFile << unsigned(temp.robot_temp) << dem ;
                outFile << temp.vehicle_speed << dem;
                outFile << temp.vehicle_latitude << dem;
                outFile << temp.vehicle_longitude << dem;
                outFile << temp.vehicle_heading << dem << "\n";
            }
        }
        outFile.close();
        ROS_INFO("Finished export bin to csv");
        std_msgs::Empty msg;
        csv_converted_.publish(msg);
    }
}

MotorInfoSub::MotorInfoSub(ros::NodeHandle* nh, std::string name) : 
                            motor_name_(name), current_(0), rpm_(0) {
    char c_name[100];
    sprintf(c_name, "/motor_controller/%s/motor_current_draw", motor_name_.c_str());
    motor_current_sub_ = nh->subscribe(std::string(c_name), 1, &MotorInfoSub::MotorCurrentCallback, this);

    sprintf(c_name, "/motor_controller/%s/wheel_rpm_actual", motor_name_.c_str());
    motor_rpm_sub_ = nh->subscribe(std::string(c_name), 1, &MotorInfoSub::MotorRpmCallback, this);

    sprintf(c_name, "/motor_controller/%s/winding_temperature", motor_name_.c_str());
    motor_temp_sub_ = nh->subscribe(std::string(c_name), 1, &MotorInfoSub::MotorWindingTempCallback, this);

    sprintf(c_name, "/motor_controller/%s/error_word", motor_name_.c_str());
    motor_error_sub_ = nh->subscribe(std::string(c_name), 1, &MotorInfoSub::MotorErrorCallback, this);
}

MotorInfoSub::~MotorInfoSub() {
    // std::cout << "destroy " << motor_name_ << std::endl;
}

void MotorInfoSub::MotorCurrentCallback(const std_msgs::Float32::ConstPtr& msg) {
    current_ = msg->data;
}

void MotorInfoSub::MotorRpmCallback(const std_msgs::Float32::ConstPtr& msg) {
    rpm_ = msg->data;
}

void MotorInfoSub::MotorWindingTempCallback(const std_msgs::Int32::ConstPtr& msg) {
    temperature_ = msg->data;
}

void MotorInfoSub::MotorErrorCallback(const std_msgs::Int32::ConstPtr& msg) {
    error_ = msg->data;
}

float MotorInfoSub::GetCurrent() {
    return current_;
}

float MotorInfoSub::GetRpm() {
    return rpm_;
}

int MotorInfoSub::GetTemperature() {
    return temperature_;
}

uint16_t MotorInfoSub::GetError() {
    return error_;
}