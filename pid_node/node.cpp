//
//
//
//
//
//

#include <ros/ros.h>
#include "PID.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>

int main(int _argc, char** _argv){

	ros::init(_argc, _argv, "pid_position_node");

	ros::NodeHandle nh;

	std::string topicReferencePose;
	std::string topicCurrentPose;
	std::string topicSpeed;
	std::string basePidX;
	std::string basePidY;
	std::string basePidZ;
	std::string enablePidName;
	
	nh.param<std::string>("reference_pose",		topicReferencePose, 	"/ual_target_pose");
	nh.param<std::string>("current_pose", 		topicCurrentPose, 	"/uav_1/mavros/local_position/pose");
	nh.param<std::string>("speed_topic", 		topicSpeed, 		"/uav_1/mavros/setpoint_velocity/cmd_vel");
	nh.param<std::string>("base_name_pid_x", 	basePidX, 		"pid_node/pid_x");
	nh.param<std::string>("base_name_pid_y", 	basePidY, 		"pid_node/pid_y");
	nh.param<std::string>("base_name_pid_z", 	basePidZ, 		"pid_node/pid_z");
	nh.param<std::string>("enable_pid_name", 	enablePidName, 		"pid_node/enable");

	
	geometry_msgs::PoseStamped referencePose, lastPose;
		

	bool isEnabled = false;
	ros::ServiceServer enableServer = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(enablePidName, [&](std_srvs::SetBool::Request &_req, std_srvs::SetBool::Response &_res){
		if(_req.data)
			std::cout << "Enabled PID controller" << std::endl;
		else
			std::cout << "Disabled PID controller" << std::endl;
			
		isEnabled =_req.data;
		_res.success = true;		
		return true;
	});

	PID pidX(1,0,0, -2, 2, -10, 10); // 666 Ros param too?
	pidX.enableRosInterface(basePidX);

	PID pidY(1,0,0, -2, 2, -10, 10);
	pidY.enableRosInterface(basePidY);

	PID pidZ(1,0,0, -2, 2, -10, 10);
	pidZ.enableRosInterface(basePidZ);

	ros::Publisher speedPublisher = nh.advertise<geometry_msgs::TwistStamped>(topicSpeed,1);

	ros::Subscriber referencePoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>(topicReferencePose, 1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
		if(referencePose.pose.position.x != _msg->pose.position.x){
			pidX.reference(_msg->pose.position.x);
		}if(referencePose.pose.position.y != _msg->pose.position.y){
			pidY.reference(_msg->pose.position.y);
		}if(referencePose.pose.position.z != _msg->pose.position.z){
			pidZ.reference(_msg->pose.position.z);
		}

		referencePose = *_msg;
	});

	lastPose.header.stamp = ros::Time::now();
	ros::Subscriber currentPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>(topicCurrentPose, 1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
		if(isEnabled){		
			auto incT = _msg->header.stamp - lastPose.header.stamp; 
		
			float x = pidX.update(_msg->pose.position.x, incT.toSec());
			float y = pidY.update(_msg->pose.position.y, incT.toSec());
			float z = pidZ.update(_msg->pose.position.z, incT.toSec());

			geometry_msgs::TwistStamped speed;
			speed.header = _msg->header;
			speed.twist.linear.x = x;
			speed.twist.linear.y = y;
			speed.twist.linear.z = z;

			speedPublisher.publish(speed);
		}
		lastPose = *_msg;				
	});

	ros::spin();


}
