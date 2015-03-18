#ifndef DRROBOTPROTOCOL_H
#define DRROBOTPROTOCOL_H
#define RID			2
#define DID			4
#define LENGTHPOS	5
#define DATAPOS    	5		//please note the first data position is at 6,
#define LENGTHOFDATAPACKAGE 9

#define MOTORPWMCTRL	5
#define MOTORPWMCTRLALL	6
#define SETOUTPUTPORT	22

#define MOTORVELCTRL	26
#define MOTORVELCTRLALL	27
#define MOTORPOSCTRL	3
#define MOTORPOSCTRLALL	4

#define SERVOCTRL		28
#define SERVOCTRLALL	29

#define MOTORENABLE		0x1e
#define GENERALSENSOR	125
#define SPECIALSENSOR	124
#define MOTORSENSOR		123

#define STX0		94
#define STX1		2
#define ETX0		94
#define ETX1		13
#define FLAG		6

static const QByteArray PING = QByteArray::fromHex("5e020100ff0101855e0d");
static const QByteArray STX = QByteArray::fromHex("5e02");
static const QByteArray ETX = QByteArray::fromHex("5e0d");
//some robot setting
#define WHEEL_R	0.135
#define WHEEL_DIS 0.45
#define MOTOR_CNT	190
#define MOTOR_DIR	1
#define PWM0	16384
#define NOCONTROL	-32768
#define FULLCNT	32767
#define PI		3.14159
#endif // DRROBOTPROTOCOL_H
