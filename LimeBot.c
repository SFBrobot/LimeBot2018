#pragma config(Sensor, dgtl1,  bump,           sensorTouch)
#pragma config(Motor,  port1,           strafeBack,    tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           driveFR,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           driveFL,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,            ,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,            ,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,            ,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,            ,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           driveBR,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           driveBL,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          strafeFront,   tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "Vex_Competition_Includes.c"

int drive = 0;
//int bumpSwitchState = 0;
//int numberOfDriveModes = 4;
// Drive functions
void LDrive(int pwr)
{
	motor[driveBL] = pwr;
	motor[driveFL] = pwr;
}
void RDrive(int pwr)
{
	motor[driveBR] = pwr;
	motor[driveFR] = pwr;
}
void Strafe(int pwr)
{
	motor[strafeBack] = pwr;
	motor[strafeFront] = pwr;
}

// Auton functions
void moveDegrees(int motorDegrees)
{

}
void rotDegrees(int motorDegrees)
{

}
void strafeDegrees(int motorDegrees)
{

}

// Drive modes
void tank(int left, int right, int strafe)
{
	LDrive(left);
	RDrive(right);
	motor[strafeBack] = (strafe - (left - right));
	motor[strafeFront] = (strafe + (left - right));
}
void arcade(int mov, int rot, int strafe)
{
	LDrive(mov + rot);
	RDrive(mov - rot);
	motor[strafeBack] = (strafe - rot);
	motor[strafeFront] = (strafe + rot);
}

void auton()
{

}

// VEX Functions/Tasks
void pre_auton()
{

}
task autonomous()
{
	//auton();
}
task usercontrol()
{
	while(true)
	{
		if(SensorValue[bump] == 1)
		{
			while(SensorValue[bump] == 1)
			{

			}
			drive++;
		}
		if(drive>=4)
		{
			drive=0;
		}
		if(drive == 0)
		{
			arcade(vexRT[Ch3], vexRT[Ch1], vexRT[Ch4]);
		}
		else if(drive == 1)
		{
			arcade(vexRT[Ch2], vexRT[Ch1], vexRT[Ch4]);
		}
		else if(drive == 2)
		{
			arcade(vexRT[Ch2], vexRT[Ch4], vexRT[Ch1]);
		}
		else if(drive == 3)
		{
			tank(vexRT[Ch3], vexRT[Ch2], vexRT[Ch1]);
		}
	}
}
