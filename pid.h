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

void initializePID(pid *inPID, double kp, double ki, double kd, short threshold, double integMax);

int targetSet(pid *inPID, int target);

int pidUpdate(pid *inPID, int sensorVal, int time);

short getMtrPwr(pid *inPID);
