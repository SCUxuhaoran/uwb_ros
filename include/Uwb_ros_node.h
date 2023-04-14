#ifndef UWB_ROS_NODE_H
#define UWB_ROS_NODE_H

#include "ros/ros.h"
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "location.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

#define ANCHOR_NUM 3
#define ANCHOR_DIS_START 6
#define MAX_BUFF_SIZE 16

//vec2d anchorArray[ANCHOR_NUM] = {{0, 0}, {100, 0}, {50, 100}};
//vec2d anchorArray[ANCHOR_NUM] = {{0, 0}, {500, 0}, {240, -400}};
vec2d anchorArray[ANCHOR_NUM] = {{0, 200}, {0, -200}, {800, 0}};

const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
					0, 1e-3, 0, 0, 0, 0,
					0, 0, 1e6, 0, 0, 0,
					0, 0, 0, 1e6, 0, 0,
					0, 0, 0, 0, 1e6, 0,
					0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
					0, 1e-3, 1e-9, 0, 0, 0,
					0, 0, 1e6, 0, 0, 0,
					0, 0, 0, 1e6, 0, 0,
					0, 0, 0, 0, 1e6, 0,
					0, 0, 0, 0, 0, 1e-9};
 
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
					0, 1e-3, 0, 0, 0, 0,
					0, 0, 1e6, 0, 0, 0,
					0, 0, 0, 1e6, 0, 0,
					0, 0, 0, 0, 1e6, 0,
					0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
					0, 1e-3, 1e-9, 0, 0, 0,
					0, 0, 1e6, 0, 0, 0,
					0, 0, 0, 1e6, 0, 0,
					0, 0, 0, 0, 1e6, 0,
					0, 0, 0, 0, 0, 1e-9};

class Uwb_start_object
{
	public:
		Uwb_start_object();
		~Uwb_start_object();

		/* Read/Write data from ttyUSB */
		bool ReadFromUSB();
		bool WriteToUSB(unsigned char*);
		bool ReadAndWriteLoopProcess();

		/* This node Publisher topic and tf */
		void PublisherOdom();
		void publisherUwbSensor();
		void publisherUwbSensorRaw();

		serial::Serial Uwb_Serial; //声明串口对象

	private:
		int baud_data;
		string usart_port, robot_frame_id;
		bool publish_odom;

		/** Ros node define*/
		ros::NodeHandle n;
		ros::Time current_time, last_time;
		double dt; //used in loopProcess caculate time difference
		//use robot_pose_ekf to calculate position, using relative position
		double x; //current x position
		double y; //current y position
		double x_start; //start x position
		double y_start; //start y position
		double th; //used in PublishOdom TF tree
		double vx;
		double vy;
		double vth;
		float sampleFreq;

		ros::Subscriber uwb_position_sub;
		ros::Publisher uwb_position_pub, uwb_pub, uwb_pub_raw;

		tf::TransformBroadcaster odom_broadcaster;
		tf::TransformListener odom_listener;
};

#endif


