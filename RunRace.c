/***************************************************************************************************
  (c) NewTec GmbH 2024   -   www.newtec.de
***************************************************************************************************/
/**
 * @file       RunRace.c
 *
 * Module description comes here.
 */
 /**************************************************************************************************/

 /* INCLUDES ***************************************************************************************/
#include <stdio.h>
#include "RunRace.h"
#include "app/StateHandler.h"
#include "app/RedetectTrack.h"
#include "service/Display.h"


// #include "app/StateHandler.h"
#include "service/DriveControl.h"
//#include "TickTimer.h"


 /* CONSTANTS **************************************************************************************/
 //#define TRESHOLD_LINE 2000
 #define TRESHOLD_LINE 500

 /* MACROS *****************************************************************************************/

 /* TYPES ******************************************************************************************/

 /* PROTOTYPES *************************************************************************************/
    static SoftTimer gTimer;


 /* VARIABLES **************************************************************************************/


 /* EXTERNAL FUNCTIONS *****************************************************************************/




// Implementation of the RunRace_Process method
Events RunRace_process(void) 
{
    Events retEvent = EV_NO_EVENT;
    static int firstTimeFlag = 1;
    if (firstTimeFlag)
    {
      firstTimeFlag = 0;
      SoftTimer_init(&gTimer);
      SoftTimerHandler_register(&gTimer);
      SoftTimer_start(&gTimer, MAX_LAP_TIME);
    }

    LineSensorValues sensorValues;
    LineSensor_read(&sensorValues);
    Driving_followLine(&sensorValues);

    if (SOFTTIMER_IS_EXPIRED(&gTimer))
    {
      retEvent =  EV_LAPTIME_TIMEOUT;
      firstTimeFlag = 0;
      Display_gotoxy(0, 2);
      Display_write("TimExpired", 10);
    }
    LineSensor_read(&sensorValues);
    if ((TRESHOLD_LINE < sensorValues.value[LINESENSOR_LEFT]) && (TRESHOLD_LINE < sensorValues.value[LINESENSOR_RIGHT])) /* if Endline detected */ //hier noch reinmachen, dass in der Mitte nichts erkannt wird?
    {
      retEvent =  EV_STARTENDLINE_DETECTED;
      firstTimeFlag = 0;

      Display_gotoxy(0, 3);
      Display_write("StartEndDet", 12);
      
    }
    if ((TRESHOLD_LINE >= sensorValues.value[LINESENSOR_LEFT]) && (TRESHOLD_LINE >= sensorValues.value[LINESENSOR_MIDDLE_LEFT]) && (TRESHOLD_LINE >= sensorValues.value[LINESENSOR_MIDDLE]) && (TRESHOLD_LINE >= sensorValues.value[LINESENSOR_MIDDLE_RIGHT]) && (TRESHOLD_LINE >= sensorValues.value[LINESENSOR_RIGHT]))
    {
      retEvent = EV_TRACK_LOST;
      //for debugging
      DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0, DRIVE_CONTROL_FORWARD);
      DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0, DRIVE_CONTROL_FORWARD);
    }

    return retEvent;
}


/* INTERNAL FUNCTIONS *****************************************************************************/
