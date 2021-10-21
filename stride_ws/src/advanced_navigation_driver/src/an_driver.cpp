/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2017, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <fstream>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <ctime>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <thread>

#define RADIANS_TO_DEGREES (180.0/M_PI)
#define AN_PACKET_HEADER_SIZE 5

void RequestMagneticCalibrationStatus()
{
	// The GPS cannot be configured to continuously output calibration data. A request is needed.

	an_packet_t *status_request{an_packet_allocate(1, 1)};
	status_request->data[0] = 191;
	an_packet_encode(status_request);

	while(ros::ok)
	{
		SendBuf(reinterpret_cast<unsigned char *> (status_request->header),6);
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}

	an_packet_free(&status_request);
}

void MagneticCalibrationCallback(const std_msgs::UInt8& msg)
{
	an_packet_t *an_packet{an_packet_allocate(1, 190)};
	an_packet->data[0] = (int)msg.data;
	an_packet_encode(an_packet);
	SendBuf(reinterpret_cast<unsigned char *> (an_packet->header), AN_PACKET_HEADER_SIZE + 1);
	an_packet_free(&an_packet);
}

void RequestDebugMessages()
{
	an_packet_t *an_packet{an_packet_allocate(15, 1)};
	an_packet->data[0] = 3;
	an_packet->data[1] = 69;
	an_packet->data[2] = 180;
	an_packet->data[3] = 181;
	an_packet->data[4] = 182;
	an_packet->data[5] = 184;
	an_packet->data[6] = 185;
	an_packet->data[7] = 186;
	an_packet->data[8] = 188;
	an_packet->data[9] = 191;
	an_packet->data[10] = 192;
	an_packet->data[11] = 194;
	an_packet->data[12] = 195;
	an_packet->data[13] = 198;
	an_packet->data[14] = 199;
	an_packet_encode(an_packet);
	SendBuf(reinterpret_cast<unsigned char *> (an_packet->header), AN_PACKET_HEADER_SIZE + 15);
	an_packet_free(&an_packet);
}

int main(int argc, char *argv[]) {
	// Debug logging
	bool should_log_for_debug = false;

	// Set up ROS node //
	ros::init(argc, argv, "an_device_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	ros::Rate loop_rate(100);

	printf("\nYour Advanced Navigation ROS driver has started\nPress Ctrl-C to interrupt\n");

	// Set up the COM port
	std::string com_port;
	int baud_rate;
	std::string imu_frame_id;
	std::string nav_sat_frame_id;
	std::string topic_prefix;

	if (argc >= 3) {
		com_port = std::string(argv[1]);
		baud_rate = atoi(argv[2]);
	}
	else {
		pnh.param("port", com_port, std::string("/dev/ttyUSB0"));
		pnh.param("baud_rate", baud_rate, 115200);
	}

	pnh.param("imu_frame_id", imu_frame_id, std::string("imu"));
	pnh.param("nav_sat_frame_id", nav_sat_frame_id, std::string("gps"));
	pnh.param("topic_prefix", topic_prefix, std::string("an_device"));

	// Subscribers
	ros::Subscriber sub = nh.subscribe(topic_prefix + "/magnetic_calibration/calibrate", 1000, MagneticCalibrationCallback);

	// Initialise Publishers and Topics //
	ros::Publisher nav_sat_fix_pub=nh.advertise<sensor_msgs::NavSatFix>(topic_prefix + "/NavSatFix",10);
	ros::Publisher heading_pub = nh.advertise<std_msgs::Float32>(topic_prefix + "/heading", 10);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(topic_prefix + "/Twist",10);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(topic_prefix + "/Imu",10);
	ros::Publisher system_status_pub = nh.advertise<std_msgs::UInt16>(topic_prefix + "/system_status",10);
	ros::Publisher filter_status_pub = nh.advertise<std_msgs::UInt16>(topic_prefix + "/filter_status",10);
	ros::Publisher magnetometers_pub = nh.advertise<geometry_msgs::Vector3>(topic_prefix + "/magnetometers",10);
	ros::Publisher magnetic_calibration_status_pub = nh.advertise<std_msgs::UInt8>(topic_prefix + "/magnetic_calibration/status",10);
	ros::Publisher magnetic_calibration_progress_pub = nh.advertise<std_msgs::UInt8>(topic_prefix + "/magnetic_calibration/progress",10);
	ros::Publisher magnetic_calibration_error_pub = nh.advertise<std_msgs::UInt8>(topic_prefix + "/magnetic_calibration/error",10);

	// Initialise messages
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec=0;
	nav_sat_fix_msg.header.stamp.nsec=0;
	nav_sat_fix_msg.header.frame_id='0';
	nav_sat_fix_msg.status.status=0;
	nav_sat_fix_msg.status.service=1; // fixed to GPS
	nav_sat_fix_msg.latitude=0.0;
	nav_sat_fix_msg.longitude=0.0;
	nav_sat_fix_msg.altitude=0.0;
	nav_sat_fix_msg.position_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type=2; // fixed to variance on the diagonal

	std_msgs::Float32 heading;
	heading.data = 0;

	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x=0.0;
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp.sec=0;
	imu_msg.header.stamp.nsec=0;
	imu_msg.header.frame_id='0';
	imu_msg.orientation.x=0.0;
	imu_msg.orientation.y=0.0;
	imu_msg.orientation.z=0.0;
	imu_msg.orientation.w=0.0;
	imu_msg.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x=0.0;
	imu_msg.angular_velocity.y=0.0;
	imu_msg.angular_velocity.z=0.0;
	imu_msg.angular_velocity_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg.linear_acceleration.x=0.0;
	imu_msg.linear_acceleration.y=0.0;
	imu_msg.linear_acceleration.z=0.0;
	imu_msg.linear_acceleration_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed

	std_msgs::UInt16 system_status;
	system_status.data = 0;

	std_msgs::UInt16 filter_status;
	filter_status.data = 0;

	geometry_msgs::Vector3 magnetometers;
	magnetometers.x = 0;
	magnetometers.y = 0;
	magnetometers.z = 0;

	std_msgs::UInt8 magnetic_calibration_status;
	magnetic_calibration_status.data = 0;

	std_msgs::UInt8 magnetic_calibration_progress;
	magnetic_calibration_progress.data = 0;

	std_msgs::UInt8 magnetic_calibration_error;
	magnetic_calibration_error.data = 0;

	// get data from com port //
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet;
	raw_sensors_packet_t raw_sensors_packet;
	magnetic_calibration_status_packet_t magnetic_calibration_status_packet;
	int bytes_received;

	if (OpenComport(const_cast<char*>(com_port.c_str()), baud_rate))
	{
		printf("Could not open serial port: %s \n",com_port.c_str());
		exit(EXIT_FAILURE);
	}

	// Start thread to request magnetic calibration status
	std::thread thread1(RequestMagneticCalibrationStatus);

	an_decoder_initialise(&an_decoder);

	// Debug Logging
	std::ofstream debug_file;
	if (should_log_for_debug == true)
	{
		time_t now = time(0); // time in seconds since Jan 1 1970
		tm *ltm = localtime(&now);
		std::string filename = std::to_string(1900 + ltm->tm_year) + "_" +
								std::to_string(1 + ltm->tm_mon) + "_" +
								std::to_string(ltm->tm_mday) + "_" +
								std::to_string(ltm->tm_hour) + "h_" +
								std::to_string(ltm->tm_min) + "m" +
								".ANPP";
		std::string dir_name = "../../../gps_debug_log";
		mkdir(dir_name.c_str(),0777);
		debug_file.open(dir_name + "/" + filename);
		RequestDebugMessages();
	}

	// Loop continuously, polling for packets
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

		// Read Data from Port
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			// increment the decode buffer length by the number of bytes received //
			an_decoder_increment(&an_decoder, bytes_received);

			// decode all the packets in the buffer //
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				// debug logging
				if (should_log_for_debug == true)
				{
					for (int i=0; i<AN_PACKET_HEADER_SIZE; i++) {
						debug_file << an_packet->header[i];
					}
					for (int i=0; i < an_packet->length; i++) {
						debug_file << an_packet->data[i];
					}
				}

				// system state packet //
				if (an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// NavSatFix
						nav_sat_fix_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						nav_sat_fix_msg.header.frame_id=nav_sat_frame_id;
						nav_sat_fix_msg.status.status = system_state_packet.filter_status.b.gnss_fix_type;
						
						nav_sat_fix_msg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude=system_state_packet.height;
						nav_sat_fix_msg.position_covariance={pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
							0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
							0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};

						// Heading
						heading.data = system_state_packet.orientation[2];

						// Twist
						twist_msg.linear.x=system_state_packet.velocity[0];
						twist_msg.linear.y=system_state_packet.velocity[1];
						twist_msg.linear.z=system_state_packet.velocity[2];
						twist_msg.angular.x=system_state_packet.angular_velocity[0];
						twist_msg.angular.y=system_state_packet.angular_velocity[1];
						twist_msg.angular.z=system_state_packet.angular_velocity[2];

						// IMU
						imu_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						imu_msg.header.frame_id=imu_frame_id;
						// Convert roll, pitch, yaw from radians to quaternion format //
						float phi = system_state_packet.orientation[0] / 2.0f;
						float theta = system_state_packet.orientation[1] / 2.0f;
						float psi = system_state_packet.orientation[2] / 2.0f;
						float sin_phi = sinf(phi);
						float cos_phi = cosf(phi);
						float sin_theta = sinf(theta);
						float cos_theta = cosf(theta);
						float sin_psi = sinf(psi);
						float cos_psi = cosf(psi);
						imu_msg.orientation.x=-cos_phi * sin_theta * sin_psi + sin_phi * cos_theta * cos_psi;
						imu_msg.orientation.y=cos_phi * sin_theta * cos_psi + sin_phi * cos_theta * sin_psi;
						imu_msg.orientation.z=cos_phi * cos_theta * sin_psi - sin_phi * sin_theta * cos_psi;
						imu_msg.orientation.w=cos_phi * cos_theta * cos_psi + sin_phi * sin_theta * sin_psi;

						imu_msg.angular_velocity.x=system_state_packet.angular_velocity[0]; // These the same as the TWIST msg values
						imu_msg.angular_velocity.y=system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.z=system_state_packet.angular_velocity[2];
						imu_msg.linear_acceleration.x=system_state_packet.body_acceleration[0];
						imu_msg.linear_acceleration.y=system_state_packet.body_acceleration[1];
						imu_msg.linear_acceleration.z=system_state_packet.body_acceleration[2];

						// System Status
						system_status.data = 0;

						if (system_state_packet.system_status.b.system_failure) {
							system_status.data = system_status.data | 1;
						}
						if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
							system_status.data = system_status.data | 1 << 1;
						}
						if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
							system_status.data = system_status.data | 1 << 2;
						}
						if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
							system_status.data = system_status.data | 1 << 3;
						}
						if (system_state_packet.system_status.b.pressure_sensor_failure) {
							system_status.data = system_status.data | 1 << 4;
						}
						if (system_state_packet.system_status.b.gnss_failure) {
							system_status.data = system_status.data | 1 << 5;
						}
						if (system_state_packet.system_status.b.accelerometer_over_range) {
							system_status.data = system_status.data | 1 << 6;
						}
						if (system_state_packet.system_status.b.gyroscope_over_range) {
							system_status.data = system_status.data | 1 << 7;
						}
						if (system_state_packet.system_status.b.magnetometer_over_range) {
							system_status.data = system_status.data | 1 << 8;
						}
						if (system_state_packet.system_status.b.pressure_over_range) {
							system_status.data = system_status.data | 1 << 9;
						}
						if (system_state_packet.system_status.b.minimum_temperature_alarm) {
							system_status.data = system_status.data | 1 << 10;
						}
						if (system_state_packet.system_status.b.maximum_temperature_alarm) {
							system_status.data = system_status.data | 1 << 11;
						}
						if (system_state_packet.system_status.b.low_voltage_alarm) {
							system_status.data = system_status.data | 1 << 12;
						}
						if (system_state_packet.system_status.b.high_voltage_alarm) {
							system_status.data = system_status.data | 1 << 13;
						}
						if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
							system_status.data = system_status.data | 1 << 14;
						}
						if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
							system_status.data = system_status.data | 1 << 15;
						}

						// Filter Status
						filter_status.data = 0;
						if (system_state_packet.filter_status.b.orientation_filter_initialised) {
							filter_status.data = filter_status.data | 1;
						}
						if (system_state_packet.filter_status.b.ins_filter_initialised) {
							filter_status.data = filter_status.data | 1 << 1;
						}
						if (system_state_packet.filter_status.b.heading_initialised) {
							filter_status.data = filter_status.data | 1 << 2;
						}
						if (system_state_packet.filter_status.b.utc_time_initialised) {
							filter_status.data = filter_status.data | 1 << 3;
						}
						if (system_state_packet.filter_status.b.internal_gnss_enabled) {
							filter_status.data = filter_status.data | 1 << 9;
						}
						if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
							filter_status.data = filter_status.data | 1 << 10;
						}
						if (system_state_packet.filter_status.b.velocity_heading_enabled) {
							filter_status.data = filter_status.data | 1 << 11;
						}
						if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
							filter_status.data = filter_status.data | 1 << 12;
						}
						if (system_state_packet.filter_status.b.external_position_active) {
							filter_status.data = filter_status.data | 1 << 13;
						}
						if (system_state_packet.filter_status.b.external_velocity_active) {
							filter_status.data = filter_status.data | 1 << 14;
						}
						if (system_state_packet.filter_status.b.external_heading_active) {
							filter_status.data = filter_status.data | 1 << 15;
						}

						//publish
						nav_sat_fix_pub.publish(nav_sat_fix_msg);
						twist_pub.publish(twist_msg);
						imu_pub.publish(imu_msg);
						system_status_pub.publish(system_status);
						filter_status_pub.publish(filter_status);
						heading_pub.publish(heading);
					}
				}

				// raw sensor packet
				if (an_packet->id == packet_id_raw_sensors)
				{
					// copy all the binary data into the typedef struct for the packet //
					// this allows easy access to all the different values             //
					if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
					{
						magnetometers.x = raw_sensors_packet.magnetometers[0];
						magnetometers.y = raw_sensors_packet.magnetometers[1];
						magnetometers.z = raw_sensors_packet.magnetometers[2];

						// publish
						magnetometers_pub.publish(magnetometers);
					}
				}

				if (an_packet->id == packet_id_magnetic_calibration_status)
				{
					// copy all the binary data into the typedef struct for the packet //
					// this allows easy access to all the different values             //
					if(decode_magnetic_calibration_status_packet(&magnetic_calibration_status_packet, an_packet) == 0)
					{
						magnetic_calibration_status.data = magnetic_calibration_status_packet.magnetic_calibration_status;
						magnetic_calibration_progress.data = magnetic_calibration_status_packet.magnetic_calibration_progress_percentage;
						magnetic_calibration_error.data = magnetic_calibration_status_packet.local_magnetic_error_percentage;

						// publish
						magnetic_calibration_status_pub.publish(magnetic_calibration_status);
						magnetic_calibration_progress_pub.publish(magnetic_calibration_progress);
						magnetic_calibration_error_pub.publish(magnetic_calibration_error);
					}
				}



				// Magnetic Calibration progress status
				// if (an_packet->id ==)

				// ---NOT WORKING (only packet id 20 and 28 are transmited/decoded)---
				// Quaternion orientation standard deviation packet //
				// if (an_packet->id == packet_id_quaternion_orientation_standard_deviation)
				// {
				// 	// copy all the binary data into the typedef struct for the packet //
				// 	// this allows easy access to all the different values             //
				// 	if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
				// 	{
				// 		// IMU
				// 		imu_msg.orientation_covariance[0] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[0], 2);
				// 		imu_msg.orientation_covariance[4] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[1], 2);
				// 		imu_msg.orientation_covariance[8] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[2], 2);
				// 	}
				// }

				// ---NOT WORKING (only packet id 20 and 28 are transmited/decoded)---
				// Euler orientation standard deviation packet //
				// if (an_packet->id == packet_id_euler_orientation_standard_deviation)
				// {
				// 	// copy all the binary data into the typedef struct for the packet //
				// 	// this allows easy access to all the different values             //
				// 	if(decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
				// 	{
				// 		heading_variance.data = pow(euler_orientation_standard_deviation_packet.standard_deviation[2], 2);
				// 	}
				// }


				// Ensure that you free the an_packet when your done with it //
				// or you will leak memory                                   //
				an_packet_free(&an_packet);				
			}
		}
	}

	if (should_log_for_debug == true)
	{
		debug_file.close();
	}
}

