float targetPoint;

#define MULTIPLIER 100
#define MAX 180
#define THRESHOLD 5 //ticks

void InitPID()
{
	targetPoint = 0;
	ResetDriveI2CEncoders();
}

void setTargetPoint(bool go, int direction, currerntPoint)
{
	if(go)
	{
		if(direction>0)
		{
			targetpoint++;
		}
		else if(direction<0)
		{
			targetpoint--;
		}
		if(targetPoint>currentPoint+MAX)
		{
			targetPoint += MAX;
		}
		else if(targetPoint<currentPoint-MAX)
		{
			targetPoint -= MAX;
		}
	}
}



void PID(int currentPoint)
{
	int power;
	if(targetPoint > currentPoint-THRESHOLD || targetPoint < currentPoint+THRESHOLD)
	{
		power = MULTIPLIER * (targetPoint - currentPoint);
	}
	if(power > 127)
	{
		power = 127;
	}
	else if(power < -127)
	{
		power = -127;
	}

}
