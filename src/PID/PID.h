#ifndef PID_h
#define PID_h

#include "../math_utils.h"

namespace Locomotion {

template <class T>
class _PID
{


public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
    _PID(T*, T*, T*,        // * constructor.  links the PID to the Input, Output, and
        T, T, T, int);     //   Setpoint.  Initial tuning parameters are also set here

    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    void Compute(unsigned long now);                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(T, T); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



  //available but not commonly used functions ********************************************************
    void SetTunings(T, T,       // * While most users will set the tunings once in the
                    T);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(unsigned long);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100



  //Display functions ****************************************************************
	T GetKp();						  // These functions query the pid for interal values.
	T GetKi();						  //  they were created mainly for the pid front-end,
	T GetKd();						  // where it's important to know what is actually
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

	void Reset();

private:
	void Initialize();

	T dispKp;				// * we'll hold on to the tuning parameters in user-entered
	T dispKi;				//   format for display purposes
	T dispKd;				//

	T kp;                  // * (P)roportional Tuning Parameter
    T ki;                  // * (I)ntegral Tuning Parameter
    T kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

    T *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    T *myOutput;             //   This creates a hard link between the variables and the
    T *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	T ITerm, lastInput;

	unsigned long SampleTime;
	T outMin, outMax;
	bool inAuto;
};


/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
template <class T>
_PID<T>::_PID(T* Input, T* Output, T* Setpoint, T Kp, T Ki, T Kd, int ControllerDirection)
{
	SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

  SampleTime = 100000;							//default Controller Sample Time is 0.1 seconds

  SetControllerDirection(ControllerDirection);
  SetTunings(Kp, Ki, Kd);

  lastTime = 0;
  inAuto = false;
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/
 template <class T>
void _PID<T>::Compute(unsigned long now)
{
   if(!inAuto) return;
   unsigned long timeChange = (now - lastTime);
   if(timeChange >= SampleTime)
   {
      /*Compute all the working error variables*/
	  double input = *myInput;
      T error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      T dInput = (input - lastInput);

      /*Compute PID Output*/
      T output = kp * error + ITerm- kd * dInput;

	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
   }
}

template <class T>
void _PID<T>::Reset()
{
	ITerm = 0;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
 template <class T>
void _PID<T>::SetTunings(T Kp, T Ki, T Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   T SampleTimeInSec = ((T)SampleTime) / 1000000.0;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Microseconds, at which the calculation is performed
 ******************************************************************************/
 template <class T>
void _PID<T>::SetSampleTime(unsigned long NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      T ratio  = (T)NewSampleTime
                      / (T)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
 template <class T>
void _PID<T>::SetOutputLimits(T Min, T Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
 template <class T>
void _PID<T>::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
 template <class T>
void _PID<T>::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
 template <class T>
void _PID<T>::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
template <class T> T _PID<T>::GetKp(){ return  dispKp; }
template <class T> T _PID<T>::GetKi(){ return  dispKi;}
template <class T> T _PID<T>::GetKd(){ return  dispKd;}
template <class T> int _PID<T>::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
template <class T> int _PID<T>::GetDirection(){ return controllerDirection;}


typedef _PID<real_t> PID;

};

#endif
