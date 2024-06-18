/**
@addtogroup app
@{
@file       StartRace.c

Handles the complete start Race process

On Entry it shall wait 3s
then the system will start driving until it reaches the start line

if the starline is detected it'll jump to the run Race state and start time messurement

if the startline is not detected it'll jump in the error state

@version    %$Id: StartRace.c Buehler
* @}
 /**************************************************************************************************/

 /* INCLUDES ***************************************************************************************/
#include <stdio.h>
#include "app/StartRace.h"
#include "service/LineSensor.h"
#include "service/DriveControl.h"
#include "os/SoftTimer.h"
#include "hal/TickTimer.h"
#include "service/Buzzer.h"
#include "service/Button.h"
#include "app/StateHandler.h"
#include "app/Driving.h"
#include "service/Display.h"

#include "app/CalibrationXXX.h"

 /* CONSTANTS **************************************************************************************/
#define STARTTIME 3000U
#define STARTLINETIME 2u
#define START_RACE_SPEED 20U
#define START_RACE_OVER_LINE_THRESHOLD 180U
#define START_RACE_NO_LINE_THRESHOLD 90U
 /* MACROS *****************************************************************************************/

 /* TYPES ******************************************************************************************/

 /* PROTOTYPES *************************************************************************************/
/** This function is used to check if a sensor is over a line
* @param[in] value sensor values
* @param[in] *parameterSet Passes the struct with the set parameters
* @return Bool true if sucessfull
*/
static Bool sensorOverLine(UInt16 value);

/** This function is used to check if a sensor is not over a line 
* @param[in] value sensor values
* @param[in] *parameterSet Passes the struct with the set parameters
* @return Bool true if sucessfull
*/
static Bool sensorNoLine(UInt16 value);
 /* VARIABLES **************************************************************************************/
StartRace_Timemeassure gRaceTime;
static SoftTimer gStartTimer;

 /* EXTERNAL FUNCTIONS *****************************************************************************/

 /* INTERNAL FUNCTIONS *****************************************************************************/


/**
 * Implementation of the StartRace_Process method
*/
Events StartRace_process(void) 
{

  //SoftTimer gStartTimer;
  static int Test = 0;
  static int driveForwardFlag = 0;

  if(!Test){
      SoftTimer_init(&gStartTimer);
      SoftTimerHandler_register(&gStartTimer);
      SoftTimer_start(&gStartTimer, STARTTIME);

      Test = 1;
  }

  //SoftTimer StartLineTimer;
  LineSensorValues SensorValues;
  Events ReturnValue = EV_NO_EVENT;
  Bool ErrorOccured = false;
  Bool StartLineDetected = true;
  
  //**********Entry: Wait 3s*****************
  //Start Timer 
  if (SOFTTIMER_IS_EXPIRED(&gStartTimer) && !driveForwardFlag)
  {
      //SoftTimer_start(&gStartTimer, STARTTIME);
      driveForwardFlag = 1;
      SoftTimer_Stop(&gStartTimer);

      Display_gotoxy(0, 2);
      Display_write("TimExpired", 10);

      if (SOFTTIMER_RET_SUCCESS != SoftTimerHandler_unRegister(&gStartTimer))
      {
        ErrorOccured = true;
        Display_gotoxy(0, 3);
        Display_write("ErrRet", 10);
      }
  }
  //Start driving
  if (driveForwardFlag)
  {
      DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, START_RACE_SPEED, DRIVE_CONTROL_FORWARD);
      DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, START_RACE_SPEED, DRIVE_CONTROL_FORWARD);

    LineSensor_read(&SensorValues);

    /*Checks if startline is detected
    * Startline is detected, if the middle sensor and the sensor on the ouside are detecting a line
    * The sensors in between shall not see a line!
    * 
    * Starline:  -|- 
    * notice the gap between those two lines 
    */
    if ((START_RACE_OVER_LINE_THRESHOLD < SensorValues.value[LINESENSOR_LEFT]) && (START_RACE_OVER_LINE_THRESHOLD < SensorValues.value[LINESENSOR_RIGHT])) /* if Startline detected */
    {
      //retEvent =  EV_STARTENDLINE_DETECTED;

      StartLineDetected = true;
      driveForwardFlag = 0;
      //Driving_stopDriving();
      DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0u, DRIVE_CONTROL_FORWARD);
      DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0u, DRIVE_CONTROL_BACKWARD);
      // Display_gotoxy(0, 4);
      // Display_write("StopDriv", 10);
    }
  }

  if (StartLineDetected && !ErrorOccured)
  {
    //Notify User with buzzer
    Buzzer_beep(BUZZER_NOTIFY);

    //Set return Value
    ReturnValue = EV_STARTENDLINE_DETECTED;
    Test = 0;

  //   /*Start Timemessure
  //   * Time messurement is started done by saving the current vallue of the tick Counter
  //   * At the end of the race 
  //   * 
  //   */
  //   //gRaceTime.StartTime = TickTimer_get();
  }
  else
  {
    // to Error State
    //ReturnValue = EV_NO_EVENT;
    ReturnValue = EV_STARTENDLINE_DETECTED_TIMEOUT;
  }


  return ReturnValue;   
}
