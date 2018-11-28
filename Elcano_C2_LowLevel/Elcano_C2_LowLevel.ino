//#include "Settings.h"
#include "pin_settings.h"
#include "trike_settings.h"
#include "PID_v1.h"  
#include <SPI.h>
#include <Servo.h>
#include <SD.h>
#include "ElcanoSerial.h"
#include "Brakes.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include "history.h"
#include "Status.h"

//Added include vehicle
#include "Vehicle.h"
using namespace elcano;

long startTime;

#define PID_CALCULATE_TIME 50
Vehicle myTrike = Vehicle();
ThrottleController throttle(MIN_ACC_OUT, MAX_ACC_OUT, PID_CALCULATE_TIME, SelectAB, SelectCD, DAC_CHANNEL);
SteeringController steer(WHEEL_MAX_LEFT_US, WHEEL_MAX_RIGHT_US, PID_CALCULATE_TIME, STEER_OUT_PIN);

#define LOOP_TIME_MS 100
#define ERROR_HISTORY 20 //number of errors to accumulate
#define ULONG_MAX 0x7FFFFFFF



unsigned long MinTickTime_ms;
unsigned long MaxTickTime_ms;

float SpeedCyclometer_revPs = 0.0;

#define IRQ_NONE 0
#define IRQ_FIRST 1
#define IRQ_SECOND 2
#define IRQ_RUNNING 3
#define NO_DATA 0xffffffff
volatile byte InterruptState = IRQ_NONE;  // Tells us if we have initialized.


volatile unsigned long TickTime = 0;  // Time from one wheel rotation to the next gives speed.
volatile unsigned long OldTick = 0;

//All of these still need to be defined
//They should be stored in trike settings probably
int calibratedWheelMaxLeft_us; 
int calibratedWheelStraight_us;
int calibratedWheelMaxRight_us; 

int calibratedWheelSensorMaxLeft; 
int calibratedWheelSensorStraight;
int calibratedWheelSensorMaxRight; 

hist history;


//These definatly stay in this code 
#define LOOP_TIME_MS 100
int baud = 9600;

ParseState TxStateHiLevel, RxStateHiLevel, RC_State;   // @@@ cant find parseState Library
SerialData TxDataHiLevel, RxDataHiLevel, RC_Data;	   // @@@ cant find SerailData

void setup(){
	//attach servo--	--in steeringController Constructor
	//setup spi--		--in throttleController Constructor

	//setup serial
	Serial.begin(baud);
	Serial1.begin(baud);
	Serial2.begin(baud);
	Serial3.begin(baud);
	//Pid params--		--in Controller Constructors
	//setupWheelRev		
		//minTickTime = circum *1000/max_speed
		//maxTickTime = circum *1000/min_speed
		//TickTime = millis();
		//OldTick = TickTime;
		//Istate = 0;
		//history.oldSpeed & olderSpeed = NO_DATA
	
	attachInterrupt(digitalPinToInterrupt(IRPT_WHEEL), WheelRev, RISING);//pin 3 on Mega
	
	//Set up to give data to highlevel
	TxDataHiLevel.clear();
	TxStateHiLevel.dt = &TxDataHiLevel;
	TxStateHiLevel.input = &Serial2;  // not used
	TxStateHiLevel.output = &Serial3;
	TxStateHiLevel.capture = MsgType::sensor;

  // SPI: set the slaveSelectPin as an output:
  pinMode (SelectAB, OUTPUT);
  pinMode (SelectCD, OUTPUT);
  pinMode (10, OUTPUT);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  // initialize SPI:
  // The following line should not be neccessary. It uses a system library.
  PRR0 &= ~4; // turn off PRR0.PRSPI bit so power isn't off
  SPI.begin();

  // ******** Initialized but unused since PID has some issues ************
  speedPID.SetOutputLimits(MIN_ACC_OUT, MAX_ACC_OUT); //useful if we want to change the limits on what values the output can be set to
  speedPID.SetSampleTime(PID_CALCULATE_TIME); //useful if we want to change the compute period
  speedPID.SetMode(AUTOMATIC); //initializes PID controller and allows it to run Compute
  steerPID.SetOutputLimits(WHEEL_MAX_LEFT_US, WHEEL_MAX_RIGHT_US); //useful if we want to change the limits on what values the output can be set to
  steerPID.SetSampleTime(PID_CALCULATE_TIME); //useful if we want to change the compute period
  steerPID.SetMode(AUTOMATIC); //initializes PID controller and allows it to run Compute
 
  // **********************************************************

  for (int channel = 0; channel < 4; channel++)
  {
    DAC_Write(channel, 0); // reset did not clear previous states
  }
  // ******* START: System Test and Calibration Cycle ******** \\
  delay(100);
  testBrakes();
  delay(1000);
  // ******* END: System Test and Calibration Cycle ******** \\
  
  for (int i = 0; i < ERROR_HISTORY; i++)
  {
    speed_errors[i] = 0;
  }

  setupWheelRev(); // WheelRev4 addition
    // Setting up data for sending to high level
    Serial.begin(9600); 
    Serial3.begin(baudrate);
    Serial2.begin(baudrate);
    Serial1.begin(baudrate);
    TxDataHiLevel.clear();
    TxStateHiLevel.dt  = &TxDataHiLevel;
    TxStateHiLevel.input = &Serial2;  // not used
    TxStateHiLevel.output = &Serial3;
    TxStateHiLevel.capture = MsgType::sensor;
  
    //setup for receiving data from High level
    RxDataHiLevel.clear();
    RxStateHiLevel.dt  = &RxDataHiLevel;
    RxStateHiLevel.input = &Serial3;
    RxStateHiLevel.output = &Serial2; // not used
    RxStateHiLevel.capture = MsgType::drive;

    // receive data indirectly from RC unit.
    RC_Data.clear();
    RC_State.dt  = &RC_Data;
    RC_State.input = &Serial1;
    RC_State.output = &Serial1;  // not used
    RC_State.capture = MsgType::sensor;

  speedCyclometerInput_mmPs = 0;

  // Sweep
  pinMode(SWEEP_PIN, INPUT);
  
}

void loop()
{
	unsigned long timeStart_ms = millis();
	static long int desired_speed_cmPs, desired_angle;
	static bool e_stop = 0, auto_mode = 0;
	if (auto_mode){
		ParseStateError r = RxStateHiLevel.update();
		if (r == ParseStateError::success) {
			desired_speed_cmPs = RxDataHiLevel.speed_cmPs;
			desired_angle = RxDataHiLevel.angle_mDeg;
		}
	}
	computeSpeed(&history);
	computeAngle();
	TxDataHiLevel.speed_cmPs = (myTrike.getSpeed() + 5) / 10;
	TxDataHiLevel.write(TxStateHiLevel.output);

	ParseStateError r = RC_State.update();
	if (r == ParseStateError::success) {
		e_stop = RC_Data.number & 0x01;
		auto_mode = RC_Data.number & 0x02;
		if (!auto_mode)
		{
			desired_speed_cmPs = RC_Data.speed_cmPs;
			desired_angle = RC_Data.angle_mDeg;
		}
	}
	if (e_stop){
		myTrike.eStop();
	}
	else{ 
		steer.setDesiredTurn(convertHLToTurn(desired_angle));
		myTrike.move(desired_angle, desired_speed_cmPs);
	}

  // DO NOT INSERT ANY LOOP CODE BELOW THIS POINT !!

   unsigned long delay_ms = millis() - (timeStart_ms + LOOP_TIME_MS);
  // Did we spend long enough in the loop that we should immediately start
  // the next pass?
  if(delay_ms > 0L)
  {
    // No, pause til the next loop start time.
    delay(delay_ms);
  }
}

void setupWheelRev(){
 MinTickTime_ms = (WHEEL_CIRCUM_MM * 1000.0) / MAX_SPEED_mmPs;
 MaxTickTime_ms = (WHEEL_CIRCUM_MM * 1000.0) / MIN_SPEED_mmPs;
 TickTime = millis();
 OldTick = TickTime;
 InterruptState = IRQ_NONE;
 history.oldSpeed_mmPs = history.olderSpeed_mmPs = NO_DATA;
 attachInterrupt (digitalPinToInterrupt(IRPT_WHEEL), WheelRev, RISING);//pin 3 on Mega
}

void WheelRev(){
 unsigned long tick;
 noInterrupts();
 tick = millis();
 if(InterruptState != IRQ_RUNNING){
   InterruptState++;
 }
 
 if((tick - TickTime) > MinTickTime_ms){
   OldTick = TickTime;
   TickTime = tick;
 }
 interrupts();
}

void computeSpeed(struct hist *data){
	unsigned long WheelRev_ms = TickTime - OldTick;
	float SpeedCyclometer_revPs = 0.0; //revolutions per sec
	if ((InterruptState == 0) || (InterruptState == 1))
	{ // No data
		throttle.setSpeedInput_mmPs(0);
		SpeedCyclometer_revPs = 0;
	}
	else if (InterruptState == 2)
	{ //  first computed speed
		SpeedCyclometer_revPs = 1000.0 / WheelRev_ms;
		double speed = WHEEL_CIRCUM_MM * SpeedCyclometer_revPs;
		throttle.setSpeedInput_mmPs(WHEEL_CIRCUM_MM * SpeedCyclometer_revPs);
		data->oldSpeed_mmPs = data->olderSpeed_mmPs = speed;
		data->oldTime_ms = OldTick;
		data->nowTime_ms = TickTime;  // time stamp for oldSpeed_mmPs
	}
	else if (InterruptState == 3){ //  new data for second and following computed speeds
		if (TickTime == data->nowTime_ms){//no new data
		  //check to see if stopped first
			unsigned long timeStamp = millis();
			if ((timeStamp - data->nowTime_ms) > MaxTickTime_ms){ // too long without getting a tick
				throttle.setSpeedInput_mmPs(0);
				SpeedCyclometer_revPs = 0;
				if ((timeStamp - data->nowTime_ms) > (2 * MaxTickTime_ms)){
					InterruptState = 1;  //  Invalidate old data
					data->oldSpeed_mmPs = NO_DATA;
					data->olderSpeed_mmPs = NO_DATA;
				}
				return;
			}
			if (data->oldSpeed_mmPs >throttle.getSpeedInput_mmPs()){ 
				throttle.setSpeedInput_mmPs(throttle.getSpeedInput_mmPs() + data->olderSpeed_mmPs*(1 - timeStamp + data->nowTime_ms));
				if (throttle.getSpeedInput_mmPs() < 0){
					throttle.setSpeedInput_mmPs(0);
				}
				SpeedCyclometer_revPs = (throttle.getSpeedInput_mmPs() / WHEEL_CIRCUM_MM);
			}
		}
		else // data is different from last
		{
			//update time block
			data->olderTime_ms = data->oldTime_ms;
			data->oldTime_ms = data->nowTime_ms;
			data->nowTime_ms = TickTime;

			//update speed block
			data->olderSpeed_mmPs = data->oldSpeed_mmPs;
			data->oldSpeed_mmPs = throttle.getSpeedInput_mmPs();
			SpeedCyclometer_revPs = (1000.0 / WheelRev_ms);
			throttle.setSpeedInput_mmPs(WHEEL_CIRCUM_MM * SpeedCyclometer_revPs);

			data->oldTickMillis = data->tickMillis;
			data->tickMillis = millis();

			data->currentSpeed_kmPh = throttle.getSpeedInput_mmPs() / 260.0;
			myTrike.distance_mm += ((data->oldTime_ms - data->olderTime_ms) / 1000.0) * (data->oldSpeed_mmPs);

			if (data->TickTime_ms - data->OldTick_ms > 1000)
			{
				data->currentSpeed_kmPh = 0;
			}
		}
	}
}
/*************************** START HIGH LEVEL PROCESSING SECTION ********************************/



int convertHLToTurn(int turnValue)
{
	// TO DO: FIGURE OUT WHAT MIN MAX VALUES ARE INPUTTED
	return map(turnValue, -TURN_MAX_DEG, TURN_MAX_DEG, WHEEL_MAX_LEFT_US, WHEEL_MAX_RIGHT_US);
	//return map(turnValue, -TURN_MAX_DEG, TURN_MAX_DEG, calibratedWheelSensorMaxLeft, calibratedWheelSensorMaxRight);
}

int convertHLToSpeed(int speedValue)
{
	// TO DO: FIGURE OUT WHAT MIN MAX VALUES ARE INPUTTED
   // return map(speedValue, 0, MAX_SPEED_CMS, MIN_ACC_OUT, MAX_ACC_OUT);
}

/*---------------------------SteeringPID--------------------------------------*/
// Computer Steering PID
// Precondition: None
// Postcondition: None
void SteeringPID(int desiredValue)
{
  desiredTurn_us = desiredValue;
  // Input into PID is microseconds and output is US
  steerPID.Compute();
  Serial.print("  PID STEER OUT = ");
  Serial.println(PIDSteeringOutput_us);
  engageSteering((int)PIDSteeringOutput_us);
}
/*------------------------------------ThrottlePID--------------------------------*/
// Compute PID 
// Precondition: None
// Postcondition: None
void ThrottlePID(int desiredValue)
{
  desiredSpeed_mmPs = desiredValue;
  Serial.print("SPEED MMPS = ");
  Serial.print(speedCyclometerInput_mmPs);
  Serial.print(" DESIRED SPEED = ");
  Serial.print(desiredSpeed_mmPs);
  // Input into PID is PWM and output is PWM
  if(desiredSpeed_mmPs < (speedCyclometerInput_mmPs + 10))
  {
     brake.Stop();
  }
  else
  {
    speedPID.Compute();
    Serial.print("  PID THROTTLE OUT = ");
    Serial.println(PIDThrottleOutput_pwm);
    currentThrottlePWM = (int)PIDThrottleOutput_pwm;
    brake.Release();
    engageWheel(currentThrottlePWM);
  }
}

void computeAngle()
{
  int left = analogRead(A2);
  int right = analogRead(A3);

   steerAngleUS = map(analogRead(A3), calibratedWheelSensorMaxLeft, calibratedWheelSensorMaxRight, calibratedWheelMaxLeft_us, calibratedWheelMaxRight_us);
 }

/*************************** END HIGH LEVEL PROCESSING SECTION ********************************/
