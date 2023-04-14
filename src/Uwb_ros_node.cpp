#include "Uwb_ros_node.h"

Uwb_start_object::Uwb_start_object()
{
	//set values and serial connection
	ros::NodeHandle nh_private("~");
	//nh_private.param<std::string>("usart_port", this->usart_port, "/dev/uwb"); 
	nh_private.param<std::string>("usart_port", this->usart_port, "/dev/ttyUSB2"); 
	nh_private.param<int>("baud_data", this->baud_data, 115200); 
	nh_private.param<std::string>("robot_frame_id", this->robot_frame_id, "base_link");
	nh_private.param<bool>("publish_odom", this->publish_odom, true); 

	//Create a boot node for the underlying driver layer of the robot base_controller
	//this->uwb_position_sub = n.subscribe(smoother_cmd_vel, 100, &Huanyu_start_object::cmd_velCallback, this);
	this->uwb_position_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	this->uwb_pub  = n.advertise<nav_msgs::Odometry>("/gps", 20);
	this->uwb_pub_raw  = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gps_raw", 20);
	//open serial device
	try{
		Uwb_Serial.setPort(this->usart_port);
		Uwb_Serial.setBaudrate(this->baud_data);
		serial::Timeout to = serial::Timeout::simpleTimeout(2000);
		Uwb_Serial.setTimeout(to);
		Uwb_Serial.open();
	}
	catch (serial::IOException& e){
		ROS_ERROR_STREAM("[XUHAORAN] Unable to open port ");
	}
	if(Uwb_Serial.isOpen()){
		ROS_INFO_STREAM("[XUHAORAN] Serial Port opened");
	}else{
	}
	try{
		vector<vec2d> result;
		unsigned char buffer[MAX_BUFF_SIZE];
		Uwb_Serial.read(buffer ,sizeof(buffer));
		switch(buffer[1])
		{
			case 'r':
			{
				int offset = 0;
				int radius[ANCHOR_NUM];
				for (int i = 0; i < ANCHOR_NUM; i++)
				{
					unsigned int dis = buffer[ANCHOR_DIS_START + offset + 1] << 8;
					dis = dis ^ buffer[ANCHOR_DIS_START + offset];
					offset = offset + 2;
					radius[i] = dis;
				}
				result = trilateration(anchorArray, radius, ANCHOR_NUM);
				//cout << setiosflags(ios::fixed) << setprecision(2) << result[0].x << "," << result[0].y << result[1].x << "," << result[1].y << endl;
				x_start = result[0].x;
				y_start = result[0].y;
				ROS_INFO("startup position x,y is set.");
				break;
			}
			default:
			{
				ROS_INFO("buffer[1] type Error");
				break;
			}
		}
	}catch (serial::IOException& e){
		ROS_ERROR_STREAM("[XUHAORAN] Unable to open port ");
	}

}

Uwb_start_object::~Uwb_start_object()
{
	Uwb_Serial.close();
}

bool Uwb_start_object::ReadFromUSB()
{
	vector<vec2d> result;
	unsigned char buffer[MAX_BUFF_SIZE];
	Uwb_Serial.read(buffer ,sizeof(buffer));
	bool res = false;
	switch(buffer[1])
	{
		case 'r':
		{
			int offset = 0;
			int radius[ANCHOR_NUM];
			for (int i = 0; i < ANCHOR_NUM; i++)
			{
				unsigned int dis = buffer[ANCHOR_DIS_START + offset + 1] << 8;
				dis = dis ^ buffer[ANCHOR_DIS_START + offset];
				offset = offset + 2;
				radius[i] = dis;
			}
			result = trilateration(anchorArray, radius, ANCHOR_NUM);
			//cout << setiosflags(ios::fixed) << setprecision(2) << result[0].x << "," << result[0].y << result[1].x << "," << result[1].y << endl;
			x = (result[0].x-x_start)/100;
			y = (result[0].y-y_start)/100;
			ROS_INFO("maxX1, maxY1, maxX2, maxY2 = [%f %f %f %f]", x, y, result[0].x, result[0].y);
			res = true;
			break;
		}
		default:
		{
			ROS_INFO("buffer[1] type Error");
			break;
		}
	}
	return res;

}


void Uwb_start_object::publisherUwbSensorRaw()
{
	geometry_msgs::PoseWithCovarianceStamped GpsSensorRaw;

	GpsSensorRaw.header.stamp = ros::Time::now(); 
	GpsSensorRaw.header.frame_id = "gps_pub_raw"; 

	GpsSensorRaw.pose.pose.position.x = 0; 
	GpsSensorRaw.pose.pose.position.y = 0; 
	GpsSensorRaw.pose.pose.position.z = 0; 
	memcpy(&GpsSensorRaw.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));	

	uwb_pub_raw.publish(GpsSensorRaw); 
}


void Uwb_start_object::publisherUwbSensor()
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	nav_msgs::Odometry gps;

	gps.header.stamp = ros::Time::now(); 
	gps.header.frame_id = "gps_pub"; 

	gps.pose.pose.position.x = x; 
	gps.pose.pose.position.y = y; 
	gps.pose.pose.position.z = 0.0;
	gps.pose.pose.orientation = odom_quat;

	gps.child_frame_id = this->robot_frame_id;
	gps.twist.twist.linear.x = 0.0;
 	gps.twist.twist.linear.y = 0.0;
	gps.twist.twist.linear.z = 0.0;

	memcpy(&gps.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));	
	memcpy(&gps.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));	

	uwb_pub.publish(gps); 
}


bool Uwb_start_object::ReadAndWriteLoopProcess()
{
	this->last_time = ros::Time::now();

	while(ros::ok())
	{
		this->current_time = ros::Time::now();
		this->dt = (current_time - last_time).toSec();
		this->sampleFreq = 1.0f/dt;
		//Get npu data include robot move speed and action status information
		if ( true == ReadFromUSB() )
		{
			//calculate uwb position
			//PublisherOdom();

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			nav_msgs::Odometry gps;

			gps.header.stamp = ros::Time::now(); 
			gps.header.frame_id = "gps_pub"; 

			gps.pose.pose.position.x = x; 
			gps.pose.pose.position.y = y; 
			gps.pose.pose.position.z = 0.0;
			gps.pose.pose.orientation = odom_quat;

			gps.child_frame_id = this->robot_frame_id;
			gps.twist.twist.linear.x = 0.0;
		 	gps.twist.twist.linear.y = 0.0;
			gps.twist.twist.linear.z = 0.0;

			memcpy(&gps.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));	
			memcpy(&gps.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));	

			uwb_pub.publish(gps); 


			ROS_INFO("[XUHAORAN] Published Odom! ");

		}
		this->last_time = current_time;    
		ros::spinOnce();

	}

}

void Uwb_start_object::PublisherOdom()
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	//publish the transform over tf
	if(publish_odom)
	{
		//listen to tf tree
		/**
		tf::StampedTransform transform_data;
		try{
			odom_listener.lookupTransform("odom_combined", "map", ros::Time(0), transform_data);
			ROS_INFO("x:%s, y:%s.", transform_data.getOrigin().x(), transform_data.getOrigin().y());
		}
		catch(tf::TransformException& ex){
			ROS_INFO("%s", ex.what());
		}
		**/

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "map";
		odom_trans.child_frame_id = "odom_trans";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		ROS_INFO("[XUHAORAN] Published Odom! ");
	}
	
}


int main(int argc, char** argv)
{
	/* Voltage thread fb*/

	ros::init(argc, argv, "uwb_controller");
	ROS_INFO("[XUHAORAN] uwb controller node start! ");

	Uwb_start_object Uwb_Control; 
	//Uwb_Control.ReadFromUSB();
	Uwb_Control.ReadAndWriteLoopProcess();
	
	return 0;
}



