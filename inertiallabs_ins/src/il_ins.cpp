#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <cav_msgs/DriverStatus.h>
#include <sensor_msgs/Imu.h>

// Inertial Labs source header
#include "ILDriver.h"

// Publishers

struct Context {
	ros::Publisher publishers[3];
	std::string imu_frame_id;
};

cav_msgs::DriverStatus msg_discovery_gnss;
cav_msgs::DriverStatus msg_discovery_imu;

ros::Time last_gnss_discovery_pub;
ros::Time last_imu_discovery_pub;

void publish_device(IL::INSDataStruct *data, void* contextPtr)
{
	Context* context = reinterpret_cast<Context*>(contextPtr);
	static int seq=0;
	seq++;

	gps_common::GPSFix msg_GPSFix;
	sensor_msgs::Imu msg_imu;

	ros::Time timestamp = ros::Time::now();
	double g = 9.80655;
	double deg_to_rad = 0.0174533;

	if (context->publishers[0].getNumSubscribers() > 0)
	{
		msg_GPSFix.header.seq = seq;
		msg_GPSFix.header.stamp = timestamp;
		msg_GPSFix.header.frame_id = "inertiallabs_gnss";

		msg_GPSFix.latitude = data->Latitude;
		msg_GPSFix.longitude = data->Longitude;
		msg_GPSFix.altitude = data->Altitude;

		msg_GPSFix.track = data->Heading;
		msg_GPSFix.speed = data->V_Hor;
		msg_GPSFix.climb = data->V_ver;

		msg_GPSFix.time = data->ms_gps;

		msg_GPSFix.gdop = data->GDOP;
		msg_GPSFix.pdop = data->PDOP;
		msg_GPSFix.hdop = data->HDOP;
		msg_GPSFix.vdop = data->VDOP;
		msg_GPSFix.tdop = data->TDOP;

		context->publishers[0].publish(msg_GPSFix);
	}

	if (context->publishers[1].getNumSubscribers() > 0)
	{
		msg_imu.header.seq = seq;
		msg_imu.header.stamp = timestamp;
		msg_imu.header.frame_id = "inertiallabs_imu";

		double roll = data->Roll;
		double pitch = data->Pitch;
		double yaw = data->Heading;

		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);

		msg_imu.orientation.w = cr * cp * cy + sr * sp * sy;
		msg_imu.orientation.x = sr * cp * cy - cr * sp * sy;
		msg_imu.orientation.y = cr * sp * cy + sr * cp * sy;
		msg_imu.orientation.z = cr * cp * sy - sr * sp * cy;

		msg_imu.linear_acceleration.x = data->Acc[0] * g;
		msg_imu.linear_acceleration.y = data->Acc[1] * g;
		msg_imu.linear_acceleration.z = data->Acc[2] * g;

		msg_imu.angular_velocity.x = data->Gyro[0] * deg_to_rad;
		msg_imu.angular_velocity.y = data->Gyro[1] * deg_to_rad;
		msg_imu.angular_velocity.z = data->Gyro[2] * deg_to_rad;

		context->publishers[1].publish(msg_imu);
	}

	if (context->publishers[2].getNumSubscribers() > 0)
	{	
		if (last_gnss_discovery_pub == ros::Time(0) || (ros::Time::now() - last_gnss_discovery_pub).toSec() > 0.8)
      	{
        	context->publishers[2].publish(msg_discovery_gnss);
			last_gnss_discovery_pub = ros::Time::now();
      	}
		if (last_imu_discovery_pub == ros::Time(0) || (ros::Time::now() - last_imu_discovery_pub).toSec() > 0.8)
      	{
        	context->publishers[2].publish(msg_discovery_imu);
			last_imu_discovery_pub = ros::Time::now();
      	}
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "il_ins");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Rate r(50);
	std::string port;
	IL::Driver ins;
	int ins_output_format;
	std::string imu_frame_id;
	Context context;

	last_gnss_discovery_pub = ros::Time::now();
	last_imu_discovery_pub = ros::Time::now();

	// Parameters

	np.param<std::string>("ins_url", port, "serial:/dev/ttyUSB0:115200");
    np.param<int>("ins_output_format", ins_output_format, 0x57);  // for OPVTA as ouput 

	// Initializing publishers

	context.publishers[0] = np.advertise<gps_common::GPSFix>("gnss_fix_fused", 10);
	context.publishers[1] = np.advertise<sensor_msgs::Imu>("imu_raw", 10);
	context.publishers[2] = np.advertise<cav_msgs::DriverStatus>("discovery", 10);

	msg_discovery_gnss.name = "/hardware_interface/gnss";
	msg_discovery_gnss.gnss = true;

	msg_discovery_imu.name = "/hardware_interface/imu";
	msg_discovery_imu.imu = true;

	ROS_INFO("connecting to INS at URL %s\n",port.c_str());

	auto il_err = ins.connect(port.c_str());
	
	if (il_err != 0)
	{
		ROS_FATAL("Could not connect to the INS on this URL %s\n",
				  port.c_str()
		);
		msg_discovery_gnss.status = cav_msgs::DriverStatus::FAULT;
		msg_discovery_imu.status = cav_msgs::DriverStatus::FAULT;
		exit(EXIT_FAILURE);
	}

	if (ins.isStarted())
	{
		ins.stop();
	}

	auto devInfo = ins.getDeviceInfo();
	auto devParams = ins.getDeviceParams();

	std::string SN(reinterpret_cast<const char *>(devInfo.IDN), 8);

	ROS_INFO("Found INS S/N %s\n", SN.c_str());

	context.imu_frame_id = SN;
	il_err = ins.start(ins_output_format);

	if (il_err != 0)
	{
		ROS_FATAL("Could not start the INS: %i\n", il_err);
		ins.disconnect();
		msg_discovery_gnss.status = cav_msgs::DriverStatus::FAULT;
		msg_discovery_imu.status = cav_msgs::DriverStatus::FAULT;
		exit(EXIT_FAILURE);
	}

	msg_discovery_gnss.status = cav_msgs::DriverStatus::OPERATIONAL;
	msg_discovery_imu.status = cav_msgs::DriverStatus::OPERATIONAL;
	ins.setCallback(&publish_device, &context);

	ROS_INFO("publishing at %d Hz\n", devParams.dataRate);
	ROS_INFO("rostopic echo the topics to see the data");

	ros::spin();

	std::cout << "Stopping INS... " << std::flush;

	ins.stop();

	std::cout << "Disconnecting... " << std::flush;

	ins.disconnect();

	std::cout << "Done." << std::endl;

	return 0;
}
