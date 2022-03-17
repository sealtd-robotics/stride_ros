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
    gps_odom_sub_        = nh_.subscribe("/nav/odom", 1, &DataRecorderSub::GpsOdomCallback, this);
    gps_heading_sub_     = nh_.subscribe("/nav/heading", 1, &DataRecorderSub::GpsHeadingCallback, this);
    gps_imu_sub_         = nh_.subscribe("/nav/filtered_imu/data", 1, &DataRecorderSub::GpsImuCallback, this);
    overseer_states_sub_ = nh_.subscribe("/overseer/state", 1, &DataRecorderSub::OverseerCallback, this);
    record_cmd_sub_      = nh_.subscribe("/cmd/record", 1, &DataRecorderSub::RecordCommandCallback, this);
    motors_rpm_cmd_sub_  = nh_.subscribe("/wheel_rpm_command", 1, &DataRecorderSub::MotorsRpmCmdCallback, this);

    // motor_RL = new MotorInfoSub(&nh_, "left_back");
    motor_RL = std::make_shared<MotorInfoSub>(&nh_, "left_back");
    motor_RR = std::make_shared<MotorInfoSub>(&nh_, "right_back");
    motor_FL = std::make_shared<MotorInfoSub>(&nh_, "left_front");
    motor_FR = std::make_shared<MotorInfoSub>(&nh_, "right_front");
}

void DataRecorderSub::GpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // LLH and NED, switch x,y for ENU
    // df_.utc_time_millisec = msg->header.stamp.toNSec() * 1E-6;
    df_.latitude_deg = msg->pose.pose.position.x;
    df_.longitude_deg = msg->pose.pose.position.y;
    df_.altitude_m = msg->pose.pose.position.z;
    df_.east_m = 0;
    df_.north_m = 0;
    df_.vel_east_ms = msg->twist.twist.linear.y;
    df_.vel_north_ms = msg->twist.twist.linear.x;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    df_.roll_deg = radToDeg(roll);
    df_.pitch_deg = radToDeg(pitch);
    df_.yaw_deg = radToDeg(yaw);

    // // Let Odom drive data recording rate
    // if (recording)
    //     WriteBinary();
}

void DataRecorderSub::GpsImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    df_.acc_x_mss = msg->linear_acceleration.x;
    df_.acc_y_mss = msg->linear_acceleration.y;
    df_.acc_z_mss = msg->linear_acceleration.z;
    df_.yaw_rate_rads = msg->angular_velocity.z;
}

void DataRecorderSub::GpsHeadingCallback(const microstrain_inertial_msgs::FilterHeading::ConstPtr& msg) {
    df_.heading_deg = msg->heading_deg;
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

void DataRecorderSub::RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    record_command_on = msg->data;
    int ret = 0;
    if (record_command_on && !recording) {
        ret = SetupRecording();
        ROS_INFO("Record command is ON. Stop recording.");
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
}

// void DataRecorderSub::MotorCurrentCallback(const std_msgs::Float32::ConstPtr& msg) {

// }

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
        df_.utc_time_millisec = ros::Time::now().toNSec() * 1E-6;
        wf.write( (char *) &df_, sizeof(DataFrame));
}

void DataRecorderSub::ConvertBin2Csv() {
    if (wf.is_open())
        wf.close();

    std::ifstream inFile (export_path + "/data.bin", std::ios::in | std::ios::binary);

    if (inFile.is_open())
    {
        std::ofstream outFile (export_path + "/data.csv");
        outFile << std::fixed << std::setprecision(8); // set precision for double
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
                outFile << temp.status << dem;
                outFile << temp.drive_status << dem;
                outFile << temp.latitude_deg << dem;
                outFile << temp.longitude_deg << dem;
                outFile << temp.altitude_m << dem;
                outFile << temp.east_m << dem;
                outFile << temp.north_m << dem;
                outFile << temp.vel_long_ms << dem;
                outFile << temp.vel_lat_ms << dem;
                outFile << temp.vel_east_ms << dem;
                outFile << temp.vel_north_ms << dem;
                outFile << temp.heading_deg << dem;
                outFile << temp.roll_deg << dem;
                outFile << temp.pitch_deg << dem;
                outFile << temp.yaw_deg << dem;
                outFile << temp.acc_x_mss << dem;
                outFile << temp.acc_y_mss << dem;
                outFile << temp.acc_z_mss << dem;
                outFile << temp.yaw_rate_rads << dem;
                outFile << temp.goal_east_m << dem;
                outFile << temp.goal_north_m << dem;
                outFile << unsigned(temp.lookahead_m) << dem;
                outFile << temp.desired_steering_deg << dem;
                outFile << temp.desired_velocity_ms << dem;
                outFile << int(motor_RL->GetRpm()) << dem;
                outFile << int(temp.desired_motor_velocity_RL_rpm) << dem;
                outFile << int(motor_RR->GetRpm()) << dem;
                outFile << int(temp.desired_motor_velocity_RR_rpm) << dem;
                outFile << int(motor_FL->GetRpm()) << dem;
                outFile << int(temp.desired_motor_velocity_FL_rpm) << dem;
                outFile << int(motor_FR->GetRpm()) << dem;
                outFile << int(temp.desired_motor_velocity_FR_rpm) << dem;
                outFile << motor_RL->GetCurrent() << dem;
                outFile << motor_RR->GetCurrent() << dem;
                outFile << motor_FL->GetCurrent() << dem;
                outFile << motor_FR->GetCurrent() << dem;
                outFile << temp.batt_voltage << dem;
                outFile << temp.batt_amp << dem;
                outFile << unsigned(temp.batt_soc) << dem;
                outFile << unsigned(temp.batt_temp) << dem;
                outFile << unsigned(temp.robot_temp) << dem << "\n";
            }
        }
        outFile.close();
        ROS_INFO("Finished export bin to csv");
    }
}

MotorInfoSub::MotorInfoSub(ros::NodeHandle* nh, std::string name) : 
                            motor_name_(name), current_(0), rpm_(0) {
    char c_name[100];
    sprintf(c_name, "/motor_controller/%s/motor_current_draw", motor_name_.c_str());
    motor_current_sub_ = nh->subscribe(std::string(c_name), 1, &MotorInfoSub::MotorCurrentCallback, this);

    sprintf(c_name, "/motor_controller/%s/wheel_rpm_actual", motor_name_.c_str());
    motor_rpm_sub_ = nh->subscribe(std::string(c_name), 1, &MotorInfoSub::MotorRpmCallback, this);
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

float MotorInfoSub::GetCurrent() {
    return current_;
}

float MotorInfoSub::GetRpm() {
    return rpm_;
}