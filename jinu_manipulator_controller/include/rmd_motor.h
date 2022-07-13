#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <linux/types.h>
#include <math.h>
#include "Common.h"
#include "rmd_log.h"
#include "rmd_can.h"
#include "rmd_utils.h"

#define CMD_DQ_SAVE                 0x06

#define CMD_SOLENOID_CONTROL        0xC0
#define CMD_CHECK_UVW               0xC1
#define CMD_DQ_ALIGN                0xC2
#define CMD_INIT_CONTROL            0xC3
#define CMD_FRICTION_COMPENSATION   0xC4
#define CMD_MU_DISK_OFFSET          0xC5
#define CMD_HOME_ZERO_OFFSET        0xC6
#define CMD_DQ_MANUAL               0xC8

#define CMD_FOC_CURRENT_CONTROL     0xD0
#define CMD_FOC_AUTO_TUNE_CURRENT   0xD1
#define CMD_FOC_GAIN                0xD2
#define CMD_FOC_OPENLOOP_CONTROL    0xD3
#define CMD_FOC_AUTO_TUNE_POSITION  0xD4
#define CMD_POS_GAIN                0xD5
#define CMD_CURRENT_NULLING         0xD6
#define CMD_MEASURE_L               0xE0
#define CMD_SINE_WAVE_TEST          0xE1
#define CMD_DQ_SHIFT                0xE3
#define CMD_PWM_DEADTIME            0xE5
#define CMD_MU150_CMD               0xEE

typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    NON_CTR:1;  // Nonius Count err //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;      // Reply Status //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    KP:16;
        unsigned    KD:16;
    }b;
    unsigned char B[7];
}mSTAT;

typedef struct _MOVE_JOINT_{
    float			RefAngleCurrent;	// reference to move at this step
    float			RefAngleCurrentOut;	// for display
    float			AngleCurrentOut;    // for display

    float			RefAngleDelta;		// reference of the past step
    float			RefAngleToGo;		// goal position - initial position
    float			RefAngleInitial;	// initial position
    unsigned long	GoalTimeCount;		// the time at which the goal is reached
    unsigned long	CurrentTimeCount;	// current time count
    char			MoveFlag;			// move flag
    int             Profile;
    // ---------------------
    float           STP_MaxAcc;
    float           STP_Speed;
    int             STP_Type;           // 0: reach target speed    1: not reach target speed
    int             STP_AccTimeCnt;
    float           STP_acctime;
    float           STP_goaltime;
    float           STP_acc;
    float           acctime;
    float           goaltime;

} MOVE_JOINT, *pMOVE_JOINT;

class rmd_motor
{
public:
    rmd_motor();

    unsigned char ref_data[8];
    unsigned char enc_data[8];
    unsigned char torque_data[8];

    int     BOARD_ID;
    int     CAN_CHANNEL;
    int     ID_SEND_REF;
    int     ID_RCV_ENC;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_SEND_GENERAL;

    // HRRLAB - CAN ID ADD VARIABLE
    int     ID_RMD_COMMAND;

    bool    ReferenceOutEnable;
    bool    ConnectionStatus;
    mSTAT   CurrentStatus;

    // Joint Variables--------
    int     DIR;
    float   PPR;    // Encoder pulse per one rotation(axis)
    int     TPC;    // Torque per current
    int     gear_ratio;
    int     direction;
    float   tic2rad;
    float   zero_offset;
    float   th_dot_joint;
    float   th_joint;
    float   th_motor;
    float   last_th_motor;
    float   th_incremental;
    bool    first_loop_updateTheta = true;
    bool    second_loop_updateTheta = true;

    //--- Home joint set
    float   homeJointReady;
    float   homeJointGround;
    float   homeJointHomming;
    float   homeJointLift;

    float   homeJointOffset_mult;
    float   homeJointOffset_mult_old;
    float   homeManualOffset;
    bool    FindHomeFlag;
    float   zeroManualOffset;

    // Control reference
    float   RefPosOut;
    float   RefCurOut;

    // Converting reference
    float   torqueToHex;

    // Sensor information
    int     EncoderValue;
    int     TorqueCommandEncoderValue;

    float   MeasuredPosition_deg;
    float   MeasuredVelocity_deg;
    float   MeasuredCurrent_A;

    char    BoardTemperature;

    int     _torque_ctrl_torque_fdback;
    float     _torque_ctrl_speed_fdback;
    int     _torque_ctrl_encoder_fdback;

    // -----------------------

    float   RefPosition_deg;
    float   RefCurrent_A;

    int     FOC_PGain;
    int     FOC_IGain;
    int     POS_PGain;
    int     POS_IGain;
    int     POS_DGain;

    MOVE_JOINT  MoveJoints;

    void    Joint_SetMoveJoint(float angle, float timeMs, int mode);
    void    Joint_SetMoveJointSTrapi(float angle, float speed, int mode);
    void    Joint_MoveJoint();
    void    Set_Torque(float torque);
    

    void    Board_ReferenceOutEnable(bool _refEnable);
    void    Board_SendReference(void);
    void    Board_GetEncData(void);
    
    void    Board_GetEncData2(void);
    void    Board_SetTorqueDataX(void);
    void    Board_GetTorqueData2(void);
    void    UpdateTheta(void);
    float   GetTheta();
    float   GetThetaDot();

    void    Board_SetCANIDs(int bno, int can_ch);
    void    Board_CANCheck(void);

    void    Board_InitControl();
    void    Board_DQAlign();
    void    Board_CurrentNulling();
    void    Board_SetHomeZeroOffset(int offset, int now_zero = 0);

    void    Board_RequestGain_FOC();
    void    Board_SetGain_FOC(int pgain, int igain, int mode);
    void    Board_RequestGain_POS();
    void    Board_SetGain_POS(int pgain, int igain, int dgain, int mode);
    void    Board_DQ_Shift(int dir);
    void    Board_DQ_Manual();
};

#endif // RMD_MOTOR_H
