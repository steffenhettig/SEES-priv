/***************************************************************************************************
  (c) FantaVier
***************************************************************************************************/
/**
 * @file       Calibration.c
 *
 * Calibrates the robot's line sensor
 * This Module is a State of the Robots statemachine.
 * The CalibrafirstTimeFlagate consists of substates.
 */
/**************************************************************************************************/

/* INCLUDES ***************************************************************************************/
#include "app/CalibrationXXX.h"
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
#define CALIB_OVER_LINE_THRESHOLD 180U
#define CALIB_NO_LINE_THRESHOLD 90U
#define CALIB_BETWEEN_LINE_UP_THRESHOLD 400U
#define CALIB_BETWEEN_LINE_DOWN_THRESHOLD 100U

#define CALIBRATION_SPEED_CALIBRATION 18U  

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

/* VARIABLES **************************************************************************************/

/** Calibration state of local state machine */
static CalibrationState gState = CALIBRATION_STATE_INIT;

/** Timer used by calibration steps. */
static SoftTimer gTimer;

/** current ErrorHandlerErrorCode of the calibration state.*/
static Errors gErrorID;

/** Current FSMRobotEvent for the finite state machine of the robot. */
static Events gRetEvent;

/** Holds the pointer for the current selected parameterset*/
//static ParameterSet_t *gCSCurrentSelectedParameterSet;

/* EXTERNAL FUNCTIONS *****************************************************************************/
Events CalibrationXXX_process(void)
{
    gRetEvent = EV_NO_EVENT;
    static int firstTimeFlag = 0;
    if(!firstTimeFlag){
        gState = CALIBRATION_STATE_INIT;
        SoftTimer_init(&gTimer);
        gErrorID = 0;
        gRetEvent = EV_NO_EVENT;
        SoftTimerHandler_register(&gTimer);
        SoftTimer_start(&gTimer, 0U);

        firstTimeFlag = 1;
    }
    LineSensorValues sensorValues;

    switch (gState)
    {
    case CALIBRATION_STATE_INIT:
        //Display_clear();
        //Display_gotoxy(0, 0);
        //Display_write("CalibInit", 10U);
        if (SOFTTIMER_IS_EXPIRED(&gTimer))
        {
            gState = CALIBRATION_STATE_TURN_RIGHT_UNTIL_LEFT_SENSOR;
            SoftTimer_start(&gTimer, 5000U);
            LineSensor_startCalibration();
        }
        break;

    case CALIBRATION_STATE_TURN_RIGHT_UNTIL_LEFT_SENSOR:
        Display_write("Right->Left", 12U);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_FORWARD);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_BACKWARD);
        if (SOFTTIMER_IS_EXPIRED(&gTimer))
        {
            gState = CALIBRATION_STATE_TIMEOUT;
        }

        LineSensor_read(&sensorValues);

        if (sensorValues.calibrated[LINESENSOR_LEFT] && (CALIB_OVER_LINE_THRESHOLD < sensorValues.value[LINESENSOR_LEFT]))
        {
            SoftTimer_restart(&gTimer);
            gState = CALIBRATION_STATE_TURN_LEFT_UNTIL_RIGHT_SENSOR;
        }
        break;

    case CALIBRATION_STATE_TURN_LEFT_UNTIL_RIGHT_SENSOR:
        Display_clear();
        Display_gotoxy(0, 0);
        Display_write("Left->Right", 12U);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_BACKWARD);
        DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, CALIBRATION_SPEED_CALIBRATION, DRIVE_CONTROL_FORWARD);

        if (SOFTTIMER_IS_EXPIRED(&gTimer))
        {
            gState = CALIBRATION_STATE_TIMEOUT;
        }

        LineSensor_read(&sensorValues);

        if (sensorValues.calibrated[LINESENSOR_RIGHT] && (CALIB_OVER_LINE_THRESHOLD < sensorValues.value[LINESENSOR_RIGHT]))
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

        LineSensor_read(&sensorValues);

        /* stop if only middle sensor sees a line */

        if ((CALIB_NO_LINE_THRESHOLD > sensorValues.value[LINESENSOR_LEFT]) 
        && ((CALIB_BETWEEN_LINE_DOWN_THRESHOLD < sensorValues.value[LINESENSOR_MIDDLE_LEFT]) && (CALIB_BETWEEN_LINE_UP_THRESHOLD > sensorValues.value[LINESENSOR_MIDDLE_LEFT]))
        && (CALIB_OVER_LINE_THRESHOLD < sensorValues.value[LINESENSOR_MIDDLE])
        && ((CALIB_BETWEEN_LINE_DOWN_THRESHOLD < sensorValues.value[LINESENSOR_MIDDLE_RIGHT]) && (CALIB_BETWEEN_LINE_UP_THRESHOLD > sensorValues.value[LINESENSOR_MIDDLE_RIGHT]))
        && (CALIB_NO_LINE_THRESHOLD > sensorValues.value[LINESENSOR_RIGHT]))
        {
            DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0u, DRIVE_CONTROL_FORWARD);
            DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0u, DRIVE_CONTROL_BACKWARD);

            SoftTimer_Stop(&gTimer);
            if (SOFTTIMER_RET_SUCCESS != SoftTimerHandler_unRegister(&gTimer))
            {
                gErrorID = ER_CALIBRATION;
                gState = CALIBRATION_STATE_TIMEOUT;
                gRetEvent = EV_CALIBRATION_FAILED;
            }
            gState = CALIBRATION_STATE_FINISHED;
        }
        break;

    case CALIBRATION_STATE_TIMEOUT:
        DriveControl_drive(DRIVE_CONTROL_MOTOR_LEFT, 0u, DRIVE_CONTROL_FORWARD);   /* Set speed of left motor to zero. This stops the motor. */
        DriveControl_drive(DRIVE_CONTROL_MOTOR_RIGHT, 0u, DRIVE_CONTROL_BACKWARD); /* Set speed of right motor to zero. This stops the motor. */
        LineSensor_stopCalibration();
        gErrorID = ER_CALIBRATION;
        gRetEvent = EV_CALIBRATION_FAILED;
        break;

    case CALIBRATION_STATE_FINISHED:
        Display_clear();
        Display_gotoxy(0, 0);
        Display_write("Fininshed", 10U);
        LineSensor_stopCalibration();
        //SoftTimerHandler_unRegister(&gTimer);
        gErrorID = ER_NO_ERROR;
        gRetEvent = EV_CALIBRATION_SUCCESSFUL;
        gState = CALIBRATION_STATE_INIT;

        firstTimeFlag = 0;
        break;

    default:
        break;
    }
    return gRetEvent;
}
/* INTERNAL FUNCTIONS *****************************************************************************/
