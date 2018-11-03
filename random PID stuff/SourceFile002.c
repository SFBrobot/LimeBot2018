#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  bump,           sensorTouch)
#pragma config(Sensor, dgtl5,  autonGood,      sensorLEDtoVCC)
#pragma config(Sensor, dgtl6,  confirmAuton,   sensorDigitalIn)
#pragma config(Sensor, dgtl7,  autonBad,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl8,  dig0,           sensorDigitalIn)
#pragma config(Sensor, dgtl10, dig1,           sensorDigitalIn)
#pragma config(Sensor, dgtl12, dig2,           sensorDigitalIn)
#pragma config(Sensor, I2C_1,  FRDriveIME,     sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  FLDriveIME,     sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  FStrafeIME,     sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  BStrafeIME,     sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_5,  SlingshotIME,   sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           strafeBack,    tmotorVex393_HBridge, openLoop, reversed, encoderPort, I2C_4)
#pragma config(Motor,  port2,           driveFR,       tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port3,           driveFL,       tmotorVex393_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port4,           slingWIME,     tmotorVex393_MC29, openLoop, encoderPort, I2C_5)
#pragma config(Motor,  port5,            ,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,            ,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           slingWOIME,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           driveBR,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           driveBL,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          strafeFront,   tmotorVex393_HBridge, openLoop, encoderPort, I2C_3)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "Vex_Competition_Includes.c"
#include "pidTest.c"
#include "utils.c"

const float TICKS_PER_DEGREE_SPEED = (392.0/360.0);
const float TICKS_PER_DEGREE_TORQUE = (627.2/360.0);
const float ERROR_MULTIPLIER = 2;
const int ERROR_ACCEPTENCE = 25;

task autonSelection();
task blinkRedLEDHalfSec();
task blinkGreenLEDHalfSec();

pid *slingPID;

int auton = 0;
task usercontrol()
{
	initPid
	while(true)
	{
		initPid
	}
}
