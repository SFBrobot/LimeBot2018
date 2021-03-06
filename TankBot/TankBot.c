#pragma config(Motor,  port2,           LDrive,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           RDrive,        tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "Vex_Competition_Includes.c"


void pre_auton() { }

void arcade(int mov, int rot)
{
	motor[LDrive] = (mov + rot);
	motor[RDrive] = (mov - rot);
}

task autonomous
{

}

task usercontrol
{
	while(true)
	{
		arcade(vexRT[Ch1], vexRT[Ch2]);
	}
}
