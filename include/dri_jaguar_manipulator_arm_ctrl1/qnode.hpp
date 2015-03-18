/**
 * @file /include/dri_jaguar_arm_ctrl/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dri_jaguar_arm_ctrl_QNODE_HPP_
#define dri_jaguar_arm_ctrl_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <dri_jaguar_manipulator_arm_ctrl1/MotorCmd.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dri_jaguar_manipulator_arm_ctrl1 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void publisher(int pos[],int vel[],int pwm[], double temp[],int len);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

signals:
	void loggingUpdated();
    	void rosShutdown();
	void cmdUpdated(int,int,int);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher motorInfo_pub_;
	ros::Subscriber motor_cmd_sub_;
	int msgCnt;
    	QStringListModel logging_model;
	void cmdReceived(const dri_jaguar_manipulator_arm_ctrl1::MotorCmd::ConstPtr& cmd);
    
};

}  // namespace dri_jaguar_arm_ctrl

#endif /* dri_jaguar_arm_ctrl_QNODE_HPP_ */
