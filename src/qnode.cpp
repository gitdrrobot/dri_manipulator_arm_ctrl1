/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/dri_jaguar_manipulator_arm_ctrl1/qnode.hpp"
#include <dri_jaguar_manipulator_arm_ctrl1/MotorInfo.h>
#include <dri_jaguar_manipulator_arm_ctrl1/MotorInfoArray.h>
#include <dri_jaguar_manipulator_arm_ctrl1/MotorCmd.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dri_jaguar_manipulator_arm_ctrl1 {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"dri_jaguar_manipulator_arm_ctrl1");
	if ( ! ros::master::check() ) {
		return false;
	}
	msgCnt = 0;
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	motorInfo_pub_ = n.advertise<dri_jaguar_manipulator_arm_ctrl1::MotorInfoArray>("dri_jaguar_manipulator_arm_sensor", 1);
        motor_cmd_sub_ = n.subscribe<dri_jaguar_manipulator_arm_ctrl1::MotorCmd>("dri_jaguar_manipulator_arm_motor_cmd", 1, boost::bind(&QNode::cmdReceived, this, _1));


	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	
	return true;
}
void QNode::publisher(int pos[],int vel[],int pwm[], double temp[],int len)
{
	dri_jaguar_manipulator_arm_ctrl1::MotorInfoArray motorInfoArray;
	motorInfoArray.motorInfos.resize(len);
	for (uint32_t i = 0 ; i < len; ++i)
	{
	  motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
	  motorInfoArray.motorInfos[i].header.frame_id = "/drrobot_jaguar_motor";
	  motorInfoArray.motorInfos[i].encoder_pos = pos[i];
	  motorInfoArray.motorInfos[i].encoder_vel = vel[i];
	  motorInfoArray.motorInfos[i].motor_pwm = pwm[i];
	  motorInfoArray.motorInfos[i].motor_temperature = temp[i];
	  motorInfoArray.motorInfos[i].motorID = i;
	}

	//ROS_INFO("publish motor info array");
	motorInfo_pub_.publish(motorInfoArray);
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::cmdReceived(const dri_jaguar_manipulator_arm_ctrl1::MotorCmd::ConstPtr& cmd)
{
	int channel =  cmd->motorID;
	int cmdValue = cmd->motorCmd;
	int cmdCtrl = cmd->motorCtrl;
	std::cout << "Ros received motor command." << std::endl;
	emit cmdUpdated(channel,cmdValue,cmdCtrl);
}
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	emit loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace dri_jaguar_manipulator_arm_ctrl1
