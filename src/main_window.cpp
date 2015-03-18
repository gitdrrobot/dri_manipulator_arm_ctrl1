/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/dri_jaguar_manipulator_arm_ctrl1/main_window.hpp"
#include "../include/dri_jaguar_manipulator_arm_ctrl1/drrobotprotocol.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dri_jaguar_manipulator_arm_ctrl1 {

using namespace Qt;

#define J1_UP_CMD       "!PR 1 -30\r"
#define J1_DOWN_CMD     "!PR 1 30\r"
#define J2_UP_CMD       "!PR 2 -30\r"
#define J2_DOWN_CMD     "!PR 2 30\r"
#define J3_UP_CMD       "!PR 1 -30\r"
#define J3_DOWN_CMD     "!PR 1 30\r"
#define J4_UP_CMD       "!G 2 250\r"
#define J4_DOWN_CMD     "!G 2 -250\r"
double resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
double tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
double FULLAD = 4095;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QWidget(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    ui.lineEditRobotIP->setInputMask("000.000.000.000");
    IP4Validator *ip4Validator = new IP4Validator();
    ui.lineEditRobotIP->setValidator(ip4Validator);

    connect( ui.pushButtonConnect, SIGNAL(clicked()),this, SLOT(connectToRobot()));
    connect( ui.pushButtonM1Down, SIGNAL(clicked()),this, SLOT(motor1Down()));
    connect( ui.pushButtonM1Up, SIGNAL(clicked()),this, SLOT(motor1Up()));
    connect( ui.pushButtonM2Down, SIGNAL(clicked()),this, SLOT(motor2Down()));
    connect( ui.pushButtonM2Up, SIGNAL(clicked()),this, SLOT(motor2Up()));
    connect( ui.pushButtonM3Down, SIGNAL(clicked()),this, SLOT(motor3Down()));
    connect( ui.pushButtonM3Up, SIGNAL(clicked()),this, SLOT(motor3Up()));
    connect( ui.pushButtonM4Down, SIGNAL(clicked()),this, SLOT(motor4Down()));
    connect( ui.pushButtonM4Up, SIGNAL(clicked()),this, SLOT(motor4Up()));
    connect( ui.pushButtonM1Stop, SIGNAL(clicked()),this, SLOT(motor1Stop()));
    connect( ui.pushButtonM2Stop, SIGNAL(clicked()),this, SLOT(motor2Stop()));
    connect( ui.pushButtonM3Stop, SIGNAL(clicked()),this, SLOT(motor3Stop()));
    connect( ui.pushButtonM4Stop, SIGNAL(clicked()),this, SLOT(motor4Stop()));
    connect(ui.pushButtonStopAll, SIGNAL(clicked()),this,SLOT(motorStopAll()));

    connect(ui.pushButtonQuery1, SIGNAL(clicked()),this,SLOT(sendQuery1()));
    connect(ui.pushButtonQuery2, SIGNAL(clicked()),this,SLOT(sendQuery2()));
    
    watchDogCnt = 2;
    pingTimer.setInterval(500);
    pingTimer.stop();
    QObject::connect(&qnode, SIGNAL(cmdUpdated(int,int,int)), this, SLOT(cmdSend(int,int,int)));

   ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}



/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/



/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {

    QSettings settings("Qt-Ros Package", "dri_jaguar_manipulator__arm_ctrl1");
    QString robot_url = settings.value("robot_url",QString("http://192.168.0.63")).toString();
    QString robot_port = settings.value("robot_port", QString("10001")).toString();
    ui.lineEditRobotIP->setText(robot_url);
    ui.lineEditRobotPort->setText(robot_port);
}

void MainWindow::WriteSettings() {

    QSettings settings("Qt-Ros Package", "dri_jaguar_manipulator_arm_ctrl1");
    settings.setValue("robot_url",ui.lineEditRobotIP->text());
    settings.setValue("robot_port",ui.lineEditRobotPort->text());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	tcpRobot1->close();
	tcpRobot2->close();
	QWidget::closeEvent(event);
}

void MainWindow::connectToRobot()
{
    bool ok = false;
    QString temp = ui.pushButtonConnect->text();

    if (temp == tr("Connect"))
    {

        QString addr = ui.lineEditRobotIP->text().trimmed();
        int port = ui.lineEditRobotPort->text().toInt(&ok,10);
        ui.pushButtonConnect ->setText(tr("Disconnect"));
        tcpRobot1 = new QTcpSocket(this);
        tcpRobot1->connectToHost(addr,port);
        connect(tcpRobot1,SIGNAL(readyRead()), this, SLOT(processRobotData1()));
        sendQuery1();

        tcpRobot2 = new QTcpSocket(this);
        tcpRobot2->connectToHost(addr,port + 1);
        connect(tcpRobot2,SIGNAL(readyRead()), this, SLOT(processRobotData2()));
	sendQuery2();

        connect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
        pingTimer.start();
        watchDogCnt = 2;
	if ( !qnode.init() ) {
		showNoMasterMessage();
	} else {
	
	}
    }
    else
    {
        pingTimer.stop();
        disconnect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
        ui.pushButtonConnect ->setText(tr("Connect"));
        tcpRobot1->close();
        tcpRobot2->close();
        watchDogCnt = 2;
        ui.frameMotorControl->setEnabled(false);
    }
}

void MainWindow::processRobotData1()
{
	QString received = "";
	qint64 count = 0;
	watchDogCnt = 0;
	while( (count = tcpRobot1->bytesAvailable()) > 0)
	{
	    received = tcpRobot1->readAll();
	    receivedData1.append(received);
	}
	if (receivedData1.endsWith("\r"))
	{
	    dealWithPackage1(receivedData1);
	    receivedData1 = "";
	}
}

void MainWindow::processRobotData2()
{
	QString received = "";
	qint64 count = 0;
	watchDogCnt = 0;
	while( (count = tcpRobot2->bytesAvailable()) > 0)
	{
	    received = tcpRobot2->readAll();
	    receivedData2.append(received);
	}
	if (receivedData2.endsWith("\r"))
	{
	    dealWithPackage2(receivedData2);
	    receivedData2 = "";
	}
}

void MainWindow:: dealWithPackage2(QString received)
{
  QString temp;
    QStringList rev = received.split("\r");
    for (int i = 0; i < rev.length(); i++)
    {
        received = rev.at(i);
        if (received.startsWith("A="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motAmp1 = strData.at(0).toDouble()/10;

                    motorData2.motAmp2 = strData.at(1).toDouble()/10;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("AI="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 3)
                {
                    motorData2.ai3 = strData.at(2).toDouble();
                    motorData2.motTemp1 = ad2Temperature(motorData2.ai3);
                    motorData2.ai4 = strData.at(3).toDouble();
                    motorData2.motTemp2 = ad2Temperature(motorData2.ai4);
                    temp.setNum(motorData2.motTemp1,'g',4);
                    ui.lineEditM3Temp->setText(temp);
                    temp.setNum(motorData2.motTemp2,'g',4);
                    ui.lineEditM4Temp->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("C="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motEncP1 = strData.at(0).toInt();
                    motorData2.motEncP2 = strData.at(1).toInt();
                    temp.setNum(motorData2.motEncP1);
                    ui.lineEditM3Pos->setText(temp);
                    temp.setNum(motorData2.motEncP2);
                    ui.lineEditM4Pos->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("P="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motPower1 = strData.at(0).toInt();
                    motorData2.motPower2 = strData.at(1).toInt();
                    temp.setNum(motorData2.motPower1);
                    ui.lineEditM3Power->setText(temp);
                    temp.setNum(motorData2.motPower2);
                    ui.lineEditM4Power->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("S="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motEncS1 = strData.at(0).toInt();
                    motorData2.motEncS2 = strData.at(1).toInt();
                    temp.setNum(motorData2.motEncS1);
                    ui.lineEditM3Vel->setText(temp);
                    temp.setNum(motorData2.motEncS2);
                    ui.lineEditM4Vel->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("T="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.ch1Temp = strData.at(0).toDouble();
                    motorData2.ch2Temp = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("V="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 2)
                {
                    motorData2.drvVoltage = strData.at(0).toDouble()/10;
                    motorData2.batVoltage = strData.at(1).toDouble()/10;
                    motorData2.reg5VVoltage = strData.at(2).toDouble()/1000;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("MMOD="))
        {
            received.remove(0,5);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.mode1 = strData.at(0).toInt();
                    motorData2.mode2 = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("FF="))
	{
		received.remove(0,3);
		received.remove(QChar('\r'),Qt::CaseInsensitive);

		try
		{
		    motorData2.statusFlag = received.toInt();
		    temp="";
		    if((motorData2.statusFlag & 0x01) != 0)
		    {
			temp.append("OverHeat+");
		    }
		    if((motorData2.statusFlag & 0x02) != 0)
		    {
			temp.append("OverVoltage+");
		    }
		    if((motorData2.statusFlag & 0x04) != 0)
		    {
			temp.append("UnderVol+");
		    }
		    if((motorData2.statusFlag & 0x08) != 0)
		    {
			temp.append("Short+");
		    }
		    if((motorData2.statusFlag & 0x10) != 0)
		    {
			temp.append("ESTOP+");
		    }

		    ui.lineEditChannel2State->setText(temp);
		}
		catch(...)
		{

		}
	}
    }

}
void MainWindow:: dealWithPackage1(QString received)
{
    QString temp;
    QStringList rev = received.split("\r");
    for (int i = 0; i < rev.length(); i++)
    {
        received = rev.at(i);

        if (received.startsWith("A="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motAmp1 = strData.at(0).toDouble()/10;

                    motorData1.motAmp2 = strData.at(1).toDouble()/10;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("AI="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 3)
                {
                    motorData1.ai3 = strData.at(2).toDouble();
                    motorData1.motTemp1 = ad2Temperature(motorData1.ai3);
                    motorData1.ai4 = strData.at(3).toDouble();

                    motorData1.motTemp2 = ad2Temperature(motorData1.ai4);
                    temp.setNum(motorData1.motTemp1,'g',4);
                    ui.lineEditM1Temp->setText(temp);
                    temp.setNum(motorData1.motTemp2,'g',4);
                    ui.lineEditM2Temp->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("C="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motEncP1 = strData.at(0).toInt();
                    motorData1.motEncP2 = strData.at(1).toInt();
                    temp.setNum(motorData1.motEncP1);
                    ui.lineEditM1Pos->setText(temp);
                    temp.setNum(motorData1.motEncP2);
                    ui.lineEditM2Pos->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("P="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motPower1 = strData.at(0).toInt();
                    motorData1.motPower2 = strData.at(1).toInt();
                    temp.setNum(motorData1.motPower1);
                    ui.lineEditM1Power->setText(temp);
                    temp.setNum(motorData1.motPower2);
                    ui.lineEditM2Power->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("S="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motEncS1 = strData.at(0).toInt();
                    motorData1.motEncS2 = strData.at(1).toInt();
                    temp.setNum(motorData1.motEncS1);
                    ui.lineEditM1Vel->setText(temp);
                    temp.setNum(motorData1.motEncS2);
                    ui.lineEditM2Vel->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("T="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.ch1Temp = strData.at(0).toDouble();
                    motorData1.ch2Temp = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("V="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 2)
                {
                    motorData1.drvVoltage = strData.at(0).toDouble()/10;
                    motorData1.batVoltage = strData.at(1).toDouble()/10;
                    motorData1.reg5VVoltage = strData.at(2).toDouble()/1000;
                    temp.setNum(motorData1.batVoltage,'g',4);
                    ui.lineEditBatVol->setText(temp);

                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("MMOD="))
        {
            received.remove(0,5);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.mode1 = strData.at(0).toInt();
                    motorData1.mode2 = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("FF="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);

            try
            {
                motorData1.statusFlag = received.toInt();
                temp="";
                if((motorData1.statusFlag & 0x01) != 0)
                {
                    temp.append("OverHeat+");
                }
                if((motorData1.statusFlag & 0x02) != 0)
                {
                    temp.append("OverVoltage+");
                }
                if((motorData1.statusFlag & 0x04) != 0)
                {
                    temp.append("UnderVol+");
                }
                if((motorData1.statusFlag & 0x08) != 0)
                {
                    temp.append("Short+");
                }
                if((motorData1.statusFlag & 0x10) != 0)
                {
                    temp.append("ESTOP+");
                }

                ui.lineEditChannel1State->setText(temp);
            }
            catch(...)
            {

            }
        }
    }
	//publish sensor data here
    int motorPos[4];
    int motorVel[4];
    int motorPWM[4];
    double motorTemperature[4];
    motorPos[0] = motorData1.motEncP1;
    motorPos[1] = motorData1.motEncP2;
    motorPos[2] = motorData2.motEncP1;
    motorPos[3] = motorData2.motEncP2;
	
    motorVel[0] = motorData1.motEncS1;
    motorVel[1] = motorData1.motEncS2;
    motorVel[2] = motorData2.motEncS1;
    motorVel[3] = motorData2.motEncS2;	

    motorPWM[0] = motorData1.motPower1;
    motorPWM[1] = motorData1.motPower2;
    motorPWM[2] = motorData2.motPower1;
    motorPWM[3] = motorData2.motPower2;	

    motorTemperature[0] = motorData1.motTemp1;
    motorTemperature[1] = motorData1.motTemp2;
    motorTemperature[2] = motorData2.motTemp1;
    motorTemperature[3] = motorData2.motTemp2;	

    qnode.publisher(motorPos,motorVel,motorPWM,motorTemperature,MOTOR_NUM) ;
    
}

double MainWindow::ad2Temperature(int adValue)
{
    //for new temperature sensor
               double tempM = 0;
               double k = (adValue / FULLAD);
               double resValue = 0;
               if (k != 0)
               {
                   resValue = (10000 / k -10000);      //AD value to resistor
               }
               else
               {
                   resValue = resTable[0];
               }


               int index = -1;
               if (resValue >= resTable[0])       //too lower
               {
                   tempM = -20;
               }
               else if (resValue <= resTable[24])
               {
                   tempM = 100;
               }
               else
               {
                   for (int i = 0; i < 24; i++)
                   {
                       if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
                       {
                           index = i;
                           break;
                       }
                   }
                   if (index >= 0)
                   {
                       tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
                   }
                   else
                   {
                       tempM = 0;
                   }

               }

               return tempM;
}

void MainWindow::motor1Down()
{
	tcpRobot1->write(J1_DOWN_CMD);
}

void MainWindow::motor1Up()
{
	tcpRobot1->write(J1_UP_CMD);
}


void MainWindow::motor2Down()
{
	tcpRobot1->write(J2_DOWN_CMD);
}

void MainWindow::motor2Up()
{
	tcpRobot1->write(J2_UP_CMD);
}


void MainWindow::motor3Down()
{
	tcpRobot2->write(J3_DOWN_CMD);
}

void MainWindow::motor3Up()
{
	tcpRobot2->write(J3_UP_CMD);
}


void MainWindow::motor4Down()
{
	tcpRobot2->write(J4_DOWN_CMD);
}

void MainWindow::motor4Up()
{
	tcpRobot2->write(J4_UP_CMD);
}


void MainWindow::motor1Stop()
{
   	tcpRobot1->write("!PR 1 0\r");
}

void MainWindow::motor2Stop()
{
   	tcpRobot1->write("!PR 2 0\r");
}

void MainWindow::motor3Stop()
{
   	tcpRobot2->write("!PR 1 0\r");
}

void MainWindow::motor4Stop()
{
   	tcpRobot2->write("!G 2 0\r");
}
void MainWindow::motorStopAll()
{
 if (ui.pushButtonStopAll->text() == "ESTOP_ALL")
    {
        tcpRobot1->write("!EX\r");
        tcpRobot2->write("!EX\r");
        ui.pushButtonStopAll->setText("Resume");
    }
    else
    {
        tcpRobot1->write("!MG\r");
        tcpRobot2->write("!MG\r");
        ui.pushButtonStopAll->setText("ESTOP_ALL");
    }
}

void MainWindow::sendQuery1()
{
	tcpRobot1->write("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
}

void MainWindow::sendQuery2()
{
	tcpRobot2->write("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
}


void MainWindow::cmdSend(int channel,int cmdValue,int motorCtrl)
{
	//make sure you well understand the cmdValue and know how the motor move
	QString temp = "";
	temp.setNum(cmdValue);

	if (channel == 0)
	{
		if (motorCtrl == 3)
		{
			temp = "!PR 1 " + temp + "\r";
			tcpRobot1->write(temp.toAscii());
		}

	}
	else if(channel == 1)
	{
		if (motorCtrl == 3)
		{
			temp = "!PR 2 " + temp + "\r";
			tcpRobot1->write(temp.toAscii());

		}
	}
	else if(channel == 2)
	{
		if (motorCtrl == 3)
		{
			temp = "!PR 1 " + temp + "\r";
			tcpRobot2->write(temp.toAscii());
		}
	}
	else if(channel == 3)
	{
		if (motorCtrl == 0)
		{
			temp = "!G 2 " + temp + "\r";
			tcpRobot2->write(temp.toAscii());

		}
	}
}

//////// send out command
void MainWindow::sendPing()
{
	
        ++watchDogCnt;
        if (watchDogCnt == 1)
        {
                ui.frameMotorControl->setEnabled(true);
                tcpRobot1->write("~MMOD\r");
                tcpRobot2->write("~MMOD\r");
        }
        else
        {
                if (watchDogCnt> 10)
                {
                    ui.frameMotorControl->setEnabled(false);
                    pingTimer.stop();
                    disconnect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
                    ui.pushButtonConnect ->setText(tr("Connect"));
                    tcpRobot1->close();
                    tcpRobot2->close();
                    watchDogCnt = 2;
                }
        }
}


}  // namespace dri_jaguar_arm_ctrl

