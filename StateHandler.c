/***************************************************************************************************
  (c) NewTec GmbH 2024   -   www.newtec.de
***************************************************************************************************/
/**
 * @file       StateHandler.c
 *
 * Module description comes here.
 */
/**************************************************************************************************/

/* INCLUDES ***************************************************************************************/
#include "app/StateHandler.h"

#include "app/Calibration.h"
#include "app/Error.h"
#include "app/Init.h"
#include "app/ModeSwitch.h"
#include "app/RaceDone.h"
#include "app/RedetectTrack.h"
#include "app/RunRace.h"
#include "app/StartRace.h"
#include "app/Wait.h"
#include "app/CalibrationXXX.h"

//for debugging
#include "service/Display.h"
#include "service/Button.h"

/* CONSTANTS **************************************************************************************/

/* MACROS *****************************************************************************************/

/* TYPES ******************************************************************************************/

/* PROTOTYPES *************************************************************************************/

/* VARIABLES **************************************************************************************/
static States gCurrentState = ST_INIT;
static Events gCurrentEvent = EV_NO_EVENT;
static Errors gErrorID;

/* EXTERNAL FUNCTIONS *****************************************************************************/


void StateHandler_process(void)
{
    Events RetEv;
    Events DumEv;

    switch (gCurrentState)
    { 
    case ST_INIT: 
        Init_process(); //---------
        gCurrentState = ST_CALIBRATION;

        break;

    case ST_WAIT:
        Display_gotoxy(0, 1);
        Display_write("enterWait", 10);
        gCurrentEvent = Wait_process(); //---------
        if (EV_PUSH_BUTTON_B_PRESSED == gCurrentEvent)
        {
            gCurrentState = ST_MODE_SWITCH;
        }
        if (EV_PUSH_BUTTON_A_PRESSED == gCurrentEvent)
        {
            //gCurrentState = ST_START_RACE;
            gCurrentState = ST_RUN_RACE;
        }
        if (EV_PUSH_BUTTON_C_PRESSED == gCurrentEvent)
        {
            gCurrentState = ST_CALIBRATION;
        }
        break;

    case ST_MODE_SWITCH:
        Display_gotoxy(0, 1);
        Display_write("B-ModeSw", 8);
        RetEv = ModeSwitch_process();
        if (EV_MODE_SWITCH_SUCCESSFUL == RetEv)//---------hier Rückgabewerte: mode_switch_successful
        {
            gCurrentState = ST_WAIT;
        }
        if (EV_MODE_SWITCH_FAILED == RetEv)
        {
            gCurrentState = ST_ERROR;
            gErrorID = ER_PARAMETER;
        }
        break;

    case ST_CALIBRATION:
        //Display_gotoxy(0, 1);
        //Display_write("entCalib", 4);
        //CalibrateState_initVariables();
        //RetEv = Calibration_process();
        RetEv = CalibrateState_doCalibration();
        if (EV_CALIBRATION_SUCCESSFUL == RetEv)//---------hier Rückgabewerte: calibration_successful
        {
            gCurrentState = ST_WAIT;
            Display_gotoxy(0, 6);
            Display_write("Calib success", 14);
        }
        if (EV_CALIBRATION_FAILED == RetEv)
        {
            //gCurrentState = ST_ERROR;
            gErrorID = ER_CALIBRATION;
            Display_gotoxy(0, 6);
            Display_write("Calib error", 12);
        }
        break;

    case ST_RUN_RACE: 
            //Display_clear();
            Display_gotoxy(0, 1);
            Display_write("enterRunRace", 13);
        switch (RunRace_process())
        {
            case EV_STARTENDLINE_DETECTED:
                //gCurrentState = ST_RACE_DONE;
                gCurrentState = ST_WAIT;
                DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0, DRIVE_CONTROL_FORWARD);
                DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0, DRIVE_CONTROL_FORWARD);
                break;

            case EV_TRACK_LOST:
                //gCurrentState = ST_REDETECT_TRACK;
                gCurrentState = ST_WAIT;
                break;

            case EV_LAPTIME_TIMEOUT:
                //gCurrentState = ST_ERROR;
                gCurrentState = ST_WAIT;
                gErrorID = ER_RACETIME;
                //for debugging
                DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0, DRIVE_CONTROL_FORWARD);
                DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0, DRIVE_CONTROL_FORWARD);
                break;

            default:
                break;
        }     
        break;

    case ST_START_RACE:
        Display_gotoxy(0, 1);
        Display_write("entStRace", 10);
        RetEv = StartRace_process();
        if (EV_STARTENDLINE_DETECTED == RetEv)//---------hier Rückgabewerte: EV_STARTLINE_DETECTED
        {
            gCurrentState = ST_RUN_RACE;
        }
        if (EV_STARTENDLINE_DETECTED_TIMEOUT == RetEv)
        {
            gCurrentState = ST_ERROR;
            gErrorID = ER_STARTLINE_TIMER;
        }
        break;

    case ST_RACE_DONE:
        DumEv = RaceDone_process();//---------
        break;

    case ST_ERROR:
        Error_process(gErrorID);//---------
        break;

    case ST_REDETECT_TRACK:
    RetEv = RedetectTrack_process();
        if(EV_TRACK_REDETECTED == RetEv)//---------hier Rückgabewerte: EV_TRACK_REDETECTED
        {
            gCurrentState = ST_RUN_RACE;
        }
        if(EV_TRACK_REDETECTED_TIMEOUT == RetEv)
        {
            gCurrentState = ST_ERROR;
            gErrorID = ER_REDETECT_LINE;
        }
        break;

    default:
        break;
  }
}

/* INTERNAL FUNCTIONS *****************************************************************************/
