/**
 * @file /include/dri_jaguar_arm_ctrl/main_window.hpp
 *
 * @brief Qt based gui for dri_jaguar_manipulator_arm_ctrl1.
 *
 * @date November 2012
 **/
#ifndef dri_jaguar_manipulator_arm_ctrl_MAIN_WINDOW_H
#define dri_jaguar_manipulator_arm_ctrl_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "../include/dri_jaguar_manipulator_arm_ctrl1/ui_main_window.h"
#include "qnode.hpp"
#include <QtGui/QWidget>
#include <QtNetwork>
#include <QTimer>
#include "../include/dri_jaguar_manipulator_arm_ctrl1/drrobotprotocol.h"
#define MOTOR_NUM	4
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace dri_jaguar_manipulator_arm_ctrl1 {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QWidget {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/


    /******************************************
    ** Manual connections
    *******************************************/
    void cmdSend(int channel,int cmdValue,int motorCtrl);
    /******************************************
    ** GUI Own slot
    *******************************************/
    void connectToRobot();
    void processRobotData1();
    void processRobotData2();
    void sendPing();
    void motor1Up();
    void motor1Down();
    void motor1Stop();

    void motor2Up();
    void motor2Down();
    void motor2Stop();

    void motor3Up();
    void motor3Down();
    void motor3Stop();

    void motor4Up();
    void motor4Down();
    void motor4Stop();

    void motorStopAll();
    void sendQuery1();
    void sendQuery2();
private:
 struct MotorDriverData
    {

        double motAmp1;     // motor1 current value
        double motAmp2;     // motor2 current value
        int motPower1;      //motor1 output power -1000 ~ +1000
        int motPower2;      //motor2 output power -1000 ~ +1000
        int motEncP1;       //motor1 encoder position value
        int motEncP2;       //motor2 encoder position value
        int motEncS1;       //motor1 encoder velocity value
        int motEncS2;       //motor2 encoder velocity value
        double motTemp1;    //motor1 temperature value
        double motTemp2;    //motor2 temperature value
        double drvVoltage;  //driver board voltage, around 12V
        double batVoltage;  // main power battery voltage
        double reg5VVoltage;//driver broad 5V
        double ai3;         // A/D channel 3 raw A/D value, full range is 4095, will translate to motor 1temperature value
        double ai4;         // A/D channel 4 raw A/D value, full range is 4095, will translate to motor2 temperature value
        double intTemp;     // on driver baord temperature sensor reading
        double ch1Temp;     // channel 1 driver chip temperature
        double ch2Temp;     // channel 2 driver chip temperature
        int statusFlag ;     // motor driver baord status
        int mode1;          // channel 1 working mode, 0--open loop, 1 -- close speed control, 2,3 -position control, 4-torgue control
        int mode2;
     };
	Ui::MainWindowDesign ui;
	QNode qnode;

	QTcpSocket *tcpRobot1;
	QTcpSocket *tcpRobot2;
	QTimer pingTimer;
	QString receivedData1;
	QString receivedData2;
	void dealWithPackage1( QString received);
	void dealWithPackage2( QString received);
	
	MotorDriverData motorData1;
	MotorDriverData motorData2;
	
	double ad2Temperature(int value);
	int watchDogCnt;
	
};

class IP4Validator : public QValidator
{
    public:
        IP4Validator(QObject *parent=0) : QValidator(parent){}
        void fixup(QString &input) const {}
        State validate(QString &input, int &pos) const
        {
            if(input.isEmpty())
                return Acceptable;
            QStringList slist = input.split(".");
            int s = slist.size();
            if(s>4) return Invalid;
            bool emptyGroup = false;
            for(int i=0;i<s;i++)
            {
                bool ok;
                if(slist[i].isEmpty())
                {
                    emptyGroup = true;
                    continue;
                }
                int val = slist[i].toInt(&ok);
                if(!ok || val<0 || val>255)
                    return Invalid;
            }
            if(s<4 || emptyGroup)
                return Intermediate;

            return Acceptable;
            }
};

}  // namespace dri_jaguar_manipulator_arm_ctrl



#endif // dri_jaguar_manipulator_arm_ctrl_MAIN_WINDOW_H
