//Originally by Lennie Peroni, graduated from SFHS 2017
//Converted from PROS robot code to ROBOTC by Benjamin Cronin, 2018

#include "utils.h"

typedef struct
{
	double kP,
    kI,
    kD,
    prop,
    integ,
    deriv,
    integLim,
    result;
  int targ,
    val,
    valLast,
    err,
    errLast,
    time,
    timeLast;
  short thresh,
    out;
  bool bIsEnabled,
    bIsOnTarg;
}pid;

//Initializes a PID instance with the given values
void initializePID(pid *inPID, double kp, double ki, double kd, short threshold, double integMax)
{
	inPID->targ = 0;
  inPID->val = 0;
  inPID->err = 0;
  inPID->time = 0;
  inPID->kP = kp;
  inPID->kI = ki;
  inPID->kD = kd;
  inPID->thresh = threshold;
  inPID->integ = 0;
  inPID->bIsEnabled = true;
  inPID->integLim = integMax;
}

//Sets the target of where the PID should be
//inPID: PID controller to set the target of
//target: encoder value that you want to achive
int targetSet(pid *inPID, int target)
{
	inPID->integ = 0;
	return (inPID->targ) = target;
}

//Updates the PID values
//inPID: PID controller to update
//sensorVal: current value of the encoder
//time: sets the given time as the last time the PID updated
//Will return the drive power needed
int pidUpdate(pid *inPID, int sensorVal, int time)
{
	//sets valLast of the passed PID with the val
  (inPID->valLast) = (inPID->val);
  //same for the error
  (inPID->errLast) = (inPID->err);
  //same for the time
  (inPID->timeLast) = (inPID->time);
  //updates the val
  (inPID->val) = sensorVal;
  //updates the err from the targ minus the val
  (inPID->err) = (inPID->targ) - (inPID->val);
  //if the absolute value of the error is less than the threshold of the PID
  if(abs(inPID->err) < (inPID->thresh))
  {
  	//the PID is on target, therefore we can turn the motor off? CHECK THIS
    (inPID->bIsOnTarg) = true;
  }
  //Sets the proportion of the error we're correcting by multiplying the error with the proportional multiplier
  (inPID->prop) = (inPID->err) * (inPID->kP);
  //sets the integ: moves the final position farther away to get it closer to the target by multiplying the error times the integral multiplier
  (inPID->integ) += (inPID->err) * (inPID->kI);
  //if the integral is greater than the limit
  if(abs(inPID->integ) > (inPID->integLim))
  {
  	//set the integral to the negitive of the current integral times the integral limit
    (inPID->integ) = (inPID->integLim) * sign(inPID->integ);
  }
  //sets the derivitive to the error minus the last error times the derivitive multiplier
  (inPID->deriv) = ((inPID->err) - (inPID->errLast)) * (inPID->kD);
  //sets the result as the proportion plus the integral times the derivitive
  (inPID->result) = (inPID->prop) + (inPID->integ) + (inPID->deriv);
  //if the PID is on target (from if(abs(inPID->err) < (inPID->thresh))), set the value of out to 0
  if(inPID->bIsOnTarg)
  {
    (inPID->out) = 0;
  }
  //otherwise round the result and set out to it
  else
  {
    (inPID->out) = round(inPID->result);
  }
  //return the value of out
  return (inPID->out);
}

//Gets the current motor output from the PID
//why is this a bool?
bool getMtrPwr(pid *inPID)
{
  return (inPID->out);
}
