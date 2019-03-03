#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1


class PID
{
  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection){
		myOutput = Output;
		myInput = Input;
		mySetpoint = Setpoint;
		inAuto = false;

		PID::SetOutputLimits(0, 255);				//default output limit corresponds to
													//the arduino pwm limits

		SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

		PID::SetControllerDirection(ControllerDirection);
		PID::SetTunings(Kp, Ki, Kd, POn);

		lastTime = millis() - SampleTime;
	};//   Setpoint.  Initial tuning parameters are also set here.
                                   
    PID(double* Input, double* Output, double* Setpoint,double Kp, double Ki, double Kd, int ControllerDirection)
		:PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection){

	};     
	
    void SetMode(int Mode){
		bool newAuto = (Mode == AUTOMATIC);
		if (newAuto && !inAuto)
		{  /*we just went from manual to auto*/
			PID::Initialize();
		}
		inAuto = newAuto;
	};               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute() {
		if (!inAuto) return false;
		unsigned long now = millis();
		unsigned long timeChange = (now - lastTime);
		if (timeChange >= SampleTime)
		{
			/*Compute all the working error variables*/
			double input = *myInput;
			double error = *mySetpoint - input;
			double dInput = (input - lastInput);
			outputSum += (ki * error);

			/*Add Proportional on Measurement, if P_ON_M is specified*/
			if (!pOnE) outputSum -= kp * dInput;

			if (outputSum > outMax) outputSum = outMax;
			else if (outputSum < outMin) outputSum = outMin;

			/*Add Proportional on Error, if P_ON_E is specified*/
			double output;
			if (pOnE) output = kp * error;
			else output = 0;

			/*Compute Rest of PID Output*/
			output += outputSum - kd * dInput;

			if (output > outMax) output = outMax;
			else if (output < outMin) output = outMin;
			*myOutput = output;

			/*Remember some variables for next time*/
			lastInput = input;
			lastTime = now;
			return true;
		}
		else return false;
	};                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double Min, double Max)
	{
		if (Min >= Max) return;
		outMin = Min;
		outMax = Max;

		if (inAuto)
		{
			if (*myOutput > outMax) *myOutput = outMax;
			else if (*myOutput < outMin) *myOutput = outMin;

			if (outputSum > outMax) outputSum = outMax;
			else if (outputSum < outMin) outputSum = outMin;
		}
	}; // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double Kp, double Ki, double Kd) {
		SetTunings(Kp, Ki, Kd, pOn);
	};         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double Kp, double Ki, double Kd, int POn)
	{
		if (Kp < 0 || Ki < 0 || Kd < 0) return;

		pOn = POn;
		pOnE = POn == P_ON_E;

		dispKp = Kp; dispKi = Ki; dispKd = Kd;

		double SampleTimeInSec = ((double)SampleTime) / 1000;
		kp = Kp;
		ki = Ki * SampleTimeInSec;
		kd = Kd / SampleTimeInSec;

		if (controllerDirection == REVERSE)
		{
			kp = (0 - kp);
			ki = (0 - ki);
			kd = (0 - kd);
		}
	};

	void SetControllerDirection(int Direction){
		if (inAuto && Direction != controllerDirection)
		{
			kp = (0 - kp);
			ki = (0 - ki);
			kd = (0 - kd);
		}
		controllerDirection = Direction;
	};	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int NewSampleTime)
	{
		if (NewSampleTime > 0)
		{
			double ratio = (double)NewSampleTime
				/ (double)SampleTime;
			ki *= ratio;
			kd /= ratio;
			SampleTime = (unsigned long)NewSampleTime;
		}
	};              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double GetKp() { return  dispKp; };						  // These functions query the pid for interal values.
	double GetKi() { return  dispKi; };						  //  they were created mainly for the pid front-end,
	double GetKd() { return  dispKd; };						  // where it's important to know what is actually 
	int GetMode() { return  inAuto ? AUTOMATIC : MANUAL; };						  //  inside the PID.
	int GetDirection() { return controllerDirection; };					  //

  private:
	void Initialize() {
		outputSum = *myOutput;
		lastInput = *myInput;
		if (outputSum > outMax) outputSum = outMax;
		else if (outputSum < outMin) outputSum = outMin;
	};
	
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	double dispKi;				//   format for display purposes
	double dispKd;				//
    
	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	double outputSum, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;
};
#endif

