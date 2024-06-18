/***************************************************************************************************
  (c) Auriga
***************************************************************************************************/
/**
 * @file       CalibrateState.c
 *
 * Calibrates the robot's line sensor
 * This Module is a State of the Robots statemachine.
 * The CalibrateState consists of substates.
 */
/**************************************************************************************************/

/* INCLUDES ***************************************************************************************/
#include "app/CalibrationXXX.h"
#include "app/Calibration.h"
#include "Common/Types.h"
//#include "app/App.h"
//#include "app/Debug.h"
//#include "app/ParameterSets.h"
#include "os/SoftTimer.h"
#include "os/ErrorHandler.h"
#include "service/DriveControl.h"
#include "service/LineSensor.h"
#include "service/Display.h"
/* CONSTANTS **************************************************************************************/
#define CALIB_TIMEOUT_MS 6000u /**< Calibration Timeout time in milliseconds used to handle lost of track or staing to long in one state of the internal statemachine*/

/* MACROS *****************************************************************************************/

/* TYPES ******************************************************************************************/
typedef enum tag_CalibrationState
{
    CALIBRATION_STATE_INIT,                         /**< Initial calibration state.           */
    CALIBRATION_STATE_TURN_RIGHT_UNTIL_LEFT_SENSOR, /**< State TURN_RIGHT_UNTIL_LEFT_SENSOR.  */
    CALIBRATION_STATE_TURN_LEFT_UNTIL_RIGHT_SENSOR, /**< State TURN_LEFT_UNTIL_RIGHT_SENSOR.  */
    CALIBRATION_STATE_CENTER_ON_LINE,               /**< State CENTER_ON_LINE.                */
    CALIBRATION_STATE_FINISHED,                     /**< State FINISHED.                      */
    CALIBRATION_STATE_TIMEOUT                       /**< State TIMEOUT.                       */
} CalibrationState;

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

/** This function is used to check if a sensor is between a line and whitespace (on the edge of a line) 
* @param[in] value sensor values
* @param[in] *parameterSet Passes the struct with the set parameters
* @return Bool true if sucessfull
*/
static Bool sensorBetweenLine(UInt16 value);

/* VARIABLES **************************************************************************************/

/** Calibration state of local state machine */
static CalibrationState gState = CALIBRATION_STATE_INIT;

/** Timer used by calibration steps. */
static SoftTimer gTimer;

/** current ErrorHandlerErrorCode of the calibration state.*/
static ErrorHandlerErrorCode gErrorMsg;

/** Current FSMRobotEvent for the finite state machine of the robot. */
static Events gFSMRobotEvent;

/** Holds the pointer for the current selected parameterset*/
//static ParameterSet_t *gCSCurrentSelectedParameterSet;

/* EXTERNAL FUNCTIONS *****************************************************************************/
Events CalibrateState_doCalibration(void)
{
    /*
    if (gCSCurrentSelectedParameterSet == NULL)
    {
        return FSM_ROBOT_EVENT_ERROR;
    }*/

    gErrorMsg = 0;
    gFSMRobotEvent = EV_NO_EVENT;
    static int Test = 0;
    if(!Test){
        gState = CALIBRATION_STATE_INIT;
        SoftTimer_init(&gTimer);
        gErrorMsg = 0;
        gFSMRobotEvent = EV_NO_EVENT;
        SoftTimerHandler_register(&gTimer);
        SoftTimer_start(&gTimer, 0U);

        Test = 1;
    }
    LineSensorValues values;

    //Debug_ShowLineSensorValues(&values);

    switch (gState)
    {
    case CALIBRATION_STATE_INIT:
        Display_clear();
        Display_gotoxy(0, 0);
        Display_write("CalibInit", 10U);
        if (SOFTTIMER_IS_EXPIRED(&gTimer))
        {
            gState = CALIBRATION_STATE_TURN_RIGHT_UNTIL_LEFT_SENSOR;
            SoftTimer_start(&gTimer, 5000U);
            LineSensor_startCalibration();
        }
        break;

    case CALIBRATION_STATE_TURN_RIGHT_UNTIL_LEFT_SENSOR:
        Display_write("TurnR", 6U);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_FORWARD);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_BACKWARD);
        if (SOFTTIMER_IS_EXPIRED(&gTimer))
        {
            gState = CALIBRATION_STATE_TIMEOUT;
        }

        LineSensor_read(&values);

        if (values.calibrated[LINESENSOR_LEFT] && sensorOverLine(values.value[LINESENSOR_LEFT]))
        {
            SoftTimer_restart(&gTimer);
            gState = CALIBRATION_STATE_TURN_LEFT_UNTIL_RIGHT_SENSOR;
        }
        break;

    case CALIBRATION_STATE_TURN_LEFT_UNTIL_RIGHT_SENSOR:
        Display_clear();
        Display_gotoxy(0, 0);
        Display_write("TurnL", 6U);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_BACKWARD);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_FORWARD);

        if (SOFTTIMER_IS_EXPIRED(&gTimer))
        {
            gState = CALIBRATION_STATE_TIMEOUT;
        }

        LineSensor_read(&values);

        if (values.calibrated[LINESENSOR_RIGHT] && sensorOverLine(values.value[LINESENSOR_RIGHT]))
        {
            if (!LineSensor_getCalibrationState())
            {
                /* restart sequence, some sensors not yet calibrated. */
                gState = CALIBRATION_STATE_TURN_RIGHT_UNTIL_LEFT_SENSOR;
            }
            else
            {
                SoftTimer_restart(&gTimer);
                gState = CALIBRATION_STATE_CENTER_ON_LINE;
            }
        }
        break;

    case CALIBRATION_STATE_CENTER_ON_LINE:
        Display_clear();
        Display_gotoxy(0, 0);
        Display_write("TurnS", 6U);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 15U, DRIVE_CONTROL_FORWARD);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 15U, DRIVE_CONTROL_BACKWARD);

        if (SOFTTIMER_IS_EXPIRED(&gTimer))
        {
            gState = CALIBRATION_STATE_TIMEOUT;
        }

        LineSensor_read(&values);

        /* stop if only middle sensor sees a line */

        if (sensorNoLine(values.value[LINESENSOR_LEFT]) 
        && sensorBetweenLine(values.value[LINESENSOR_MIDDLE_LEFT]) 
        && sensorOverLine(values.value[LINESENSOR_MIDDLE])
        && sensorBetweenLine(values.value[LINESENSOR_MIDDLE_RIGHT]) 
        && sensorNoLine(values.value[LINESENSOR_RIGHT]))
        {
            DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0u, DRIVE_CONTROL_FORWARD);
            DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0u, DRIVE_CONTROL_BACKWARD);

            SoftTimer_Stop(&gTimer);
            if (SOFTTIMER_RET_SUCCESS != SoftTimerHandler_unRegister(&gTimer))
            {
                gErrorMsg = ERRORHANDLER_CALIBRATE_TIMER_UNINIT_FAIL;
                gState = CALIBRATION_STATE_TIMEOUT;
                gFSMRobotEvent = EV_CALIBRATION_FAILED;
            }
            gState = CALIBRATION_STATE_FINISHED;
        }
        break;

    case CALIBRATION_STATE_TIMEOUT:
        DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0u, DRIVE_CONTROL_FORWARD);   /* Set speed of left motor to zero. This stops the motor. */
        DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0u, DRIVE_CONTROL_BACKWARD); /* Set speed of right motor to zero. This stops the motor. */
        LineSensor_stopCalibration();
        gErrorMsg = ERRORHANDLER_CALIBRATE_TIMEOUT;
        gFSMRobotEvent = EV_CALIBRATION_FAILED;
        break;

    case CALIBRATION_STATE_FINISHED:
        Display_clear();
        Display_gotoxy(0, 0);
        Display_write("Fininshed", 10U);
        LineSensor_stopCalibration();
        //SoftTimerHandler_unRegister(&gTimer);
        gErrorMsg = 0;
        gFSMRobotEvent = EV_CALIBRATION_SUCCESSFUL;
        gState = CALIBRATION_STATE_INIT;

        Test = 0;
        break;
    default:
        break;
    }
    //*currentErrorHandlerMsg = gErrorMsg;
    return gFSMRobotEvent;
}
/*
void CalibrateState_updateParameters(void)
{
    gCSCurrentSelectedParameterSet = parameterSets_getSelectedParameterSet();
}*/

void CalibrateState_initVariables(void)
{
    //gCSCurrentSelectedParameterSet = parameterSets_getSelectedParameterSet();
    gState = CALIBRATION_STATE_INIT;
    SoftTimer_init(&gTimer);
    gErrorMsg = 0;
    gFSMRobotEvent = EV_NO_EVENT;
    SoftTimerHandler_register(&gTimer);
    SoftTimer_start(&gTimer, 0U);
}
/* INTERNAL FUNCTIONS *****************************************************************************/

static Bool sensorOverLine(UInt16 value)
{
    Bool ret;
    if (value < 180U)
    {
        ret = FALSE;
    }
    else
    {
        ret = TRUE;
    }
    return ret;
}

static Bool sensorNoLine(UInt16 value)
{
    Bool ret;
    if (value > 90U)
    {
        ret = FALSE;
    }
    else
    {
        ret = TRUE;
    }
    return ret;
}

static Bool sensorBetweenLine(UInt16 value)
{
    Bool ret;
    if ((value > 100U) && (value < 400U))
    {
        ret = TRUE;
    }
    else
    {
        ret = FALSE;
    }
    return ret;
}
