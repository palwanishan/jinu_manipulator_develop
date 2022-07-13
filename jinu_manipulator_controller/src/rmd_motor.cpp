#include "rmd_motor.h"

rmd_motor::rmd_motor()
{
    MoveJoints.MoveFlag = false;
    CurrentStatus.B[0] = CurrentStatus.B[1] = CurrentStatus.B[2] = 0;
    homeManualOffset = 0;
}

void rmd_motor::Joint_SetMoveJoint(float angle, float timeMs, int mode){
    if(MoveJoints.MoveFlag == true){
//        FILE_LOG(logWARNING) << "Joint_SetMoveJoint working now..[BNO: " << BOARD_ID << "]";
        return;
    }
    MoveJoints.RefAngleCurrent = RefPosition_deg;
    MoveJoints.Profile = 0;
    MoveJoints.RefAngleInitial = MoveJoints.RefAngleCurrent;
    if(mode == 0){  // abs
        MoveJoints.RefAngleToGo = angle;
        MoveJoints.RefAngleDelta = MoveJoints.RefAngleToGo - MoveJoints.RefAngleInitial;
    }else{          // rel
        MoveJoints.RefAngleToGo = MoveJoints.RefAngleInitial + angle;
        MoveJoints.RefAngleDelta = angle;
    }

    MoveJoints.GoalTimeCount = (ulong)(timeMs/(float)RT_MS);
    MoveJoints.CurrentTimeCount = 0;
    MoveJoints.MoveFlag = true;
}

void rmd_motor::Joint_SetMoveJointSTrapi(float angle, float speed, int mode){
    if(MoveJoints.MoveFlag == true){
        FILE_LOG(logWARNING) << "RBJoint_SetMoveJointSTrapi working now..[BNO: " << BOARD_ID << "]";
        return;
    }
    MoveJoints.RefAngleCurrent = RefPosition_deg;
    MoveJoints.Profile = 1;
    MoveJoints.RefAngleInitial = MoveJoints.RefAngleCurrent;
    if(mode == 0){  // abs
        MoveJoints.RefAngleToGo = angle;
        MoveJoints.RefAngleDelta = MoveJoints.RefAngleToGo - MoveJoints.RefAngleInitial;
    }else{          // rel
        MoveJoints.RefAngleToGo = MoveJoints.RefAngleInitial + angle;
        MoveJoints.RefAngleDelta = angle;
    }

    MoveJoints.STP_Speed = fabs(speed);
    if(MoveJoints.STP_Speed * MoveJoints.STP_Speed / MoveJoints.STP_MaxAcc >= fabs(MoveJoints.RefAngleDelta)){
        MoveJoints.STP_Type = 1;
        MoveJoints.STP_acctime = sqrt(fabs(MoveJoints.RefAngleDelta)/MoveJoints.STP_MaxAcc);
        MoveJoints.STP_goaltime = MoveJoints.STP_acctime * 2;
        if(MoveJoints.STP_acctime < 0.005){
            MoveJoints.STP_AccTimeCnt = 1;
            MoveJoints.GoalTimeCount = 2;
        }else{
            MoveJoints.STP_AccTimeCnt = (uint)(1000.0 * MoveJoints.STP_acctime /(float)RT_MS);
            MoveJoints.GoalTimeCount = (uint)(1000.0 * MoveJoints.STP_goaltime /(float)RT_MS);
        }
        MoveJoints.acctime = MoveJoints.STP_AccTimeCnt*(float)RT_MS / 1000.0;
        MoveJoints.STP_acc = fabs(MoveJoints.RefAngleDelta) / (MoveJoints.acctime*MoveJoints.acctime);
    }else{
        MoveJoints.STP_Type = 0;
        MoveJoints.STP_acctime = MoveJoints.STP_Speed / MoveJoints.STP_MaxAcc;
        MoveJoints.STP_goaltime = MoveJoints.STP_acctime * 2 + (fabs(MoveJoints.RefAngleDelta) - MoveJoints.STP_Speed*MoveJoints.STP_acctime) / MoveJoints.STP_Speed;
        MoveJoints.STP_AccTimeCnt = (uint)(1000.0 * MoveJoints.STP_acctime /(float)RT_MS);
        MoveJoints.GoalTimeCount = (uint)(1000.0 * MoveJoints.STP_goaltime /(float)RT_MS);
        MoveJoints.acctime = MoveJoints.STP_AccTimeCnt*(float)RT_MS / 1000.0;
        MoveJoints.goaltime = MoveJoints.GoalTimeCount*(float)RT_MS / 1000.0;
        if(MoveJoints.acctime < 2){
            MoveJoints.STP_acc = MoveJoints.STP_MaxAcc;
        }else{
            MoveJoints.STP_acc = fabs(MoveJoints.RefAngleDelta) / (MoveJoints.goaltime*MoveJoints.acctime - MoveJoints.acctime*MoveJoints.acctime);
        }
        FILE_LOG(logWARNING) << MoveJoints.RefAngleDelta << ", " << MoveJoints.goaltime << ", " << MoveJoints.acctime;
    }
    MoveJoints.CurrentTimeCount = 0;
    MoveJoints.MoveFlag = true;
}

void rmd_motor::Joint_MoveJoint(){

    if(MoveJoints.MoveFlag){
        MoveJoints.CurrentTimeCount++;

        if(MoveJoints.GoalTimeCount <= MoveJoints.CurrentTimeCount)
        {
            MoveJoints.GoalTimeCount = MoveJoints.CurrentTimeCount = 0;
            MoveJoints.RefAngleCurrent = MoveJoints.RefAngleToGo;
            RefPosition_deg = MoveJoints.RefAngleCurrent;
            MoveJoints.MoveFlag = false;
        }
        else
        {
            MoveJoints.RefAngleCurrent = MoveJoints.RefAngleInitial + MoveJoints.RefAngleDelta*0.5*
                (1.0f-cos(PI/(float)MoveJoints.GoalTimeCount*(float)MoveJoints.CurrentTimeCount));
            RefPosition_deg = MoveJoints.RefAngleCurrent;
        }
    }
    else{
    }
}

void rmd_motor::Board_ReferenceOutEnable(bool _refEnable){
    ReferenceOutEnable = _refEnable;
}

void rmd_motor::Board_SendReference(void){
    if(ReferenceOutEnable == true) {
        // resistor_constant : 3(1mohm), 1.5(2mohm), 1(3mohm)
        // Current(mA) = cur_ref/resistor_constant
        short cur_ref = (short)(RefCurOut*DIR/3.0);
        int pos_ref = (int)(RefPosOut*PPR);

        CurrentStatus.b.KP = POS_PGain;
        CurrentStatus.b.KD = POS_DGain;

        ref_data[0] = pos_ref & 0xFF;
        ref_data[1] = (pos_ref>>8) & 0xFF;
        ref_data[2] = (pos_ref>>16) & 0xFF;
        ref_data[3] = (unsigned char)(cur_ref&0xFF);
        ref_data[4] = (cur_ref>>8)&0xFF;
        ref_data[5] = POS_PGain & 0xFF;
        ref_data[6] = ((POS_PGain&0xF00) >> 4 & 0xF0) | ((POS_DGain&0xF00) >> 8 & 0x0F);
        ref_data[7] = POS_DGain & 0xFF;
    }
}

void rmd_motor::Board_GetEncData(void) {
    if(enc_data[1] == 0) {
        int temp_enc = (int)(enc_data[4] | (enc_data[5]<<8) | (enc_data[6]<<16) | (enc_data[7]<<24));  
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
        MeasuredPosition_deg = (float)EncoderValue/PPR + zeroManualOffset;
    }
    else {
        int temp_enc = (int)(enc_data[1] | (enc_data[2]<<8) | ((enc_data[3]&0xF0)<<12));
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
        MeasuredPosition_deg = (float)EncoderValue/PPR + zeroManualOffset;
    }
}

void rmd_motor::Board_GetEncData2(void) {
    if(enc_data[7] == 0) {
        int temp_enc = (int)(enc_data[1] | (enc_data[2]<<8) | (enc_data[3]<<16) | (enc_data[4]<<24));
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
    }
    else {
        int temp_enc = (int)(enc_data[1] | (enc_data[2]<<8) | ((enc_data[3]&0xF0)<<12));
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
    }    
    MeasuredPosition_deg = (float)EncoderValue/PPR + zeroManualOffset;
}

void rmd_motor::Board_SetTorqueDataX(void) {
    // int temp_torque = (int)(torque_data[2] | (torque_data[3]<<8));
    // _torque_ctrl_torque_fdback = temp_torque;
    int temp_speed = (int)(torque_data[4] | (torque_data[5]<<8));
    if(temp_speed & 0x80000) temp_speed |= 0xFFF00000;
    if(temp_speed > 30000) temp_speed -= 65535;
    _torque_ctrl_speed_fdback = 0.01 * temp_speed / gear_ratio * direction;
    
    // _torque_ctrl_encoder_fdback = temp_enc;

    int temp_enc = (int)(torque_data[6] | (torque_data[7]<<8));
    // if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
    th_motor = temp_enc / tic2rad;
    if(first_loop_updateTheta){
        th_joint = zero_offset;
        last_th_motor = th_motor;
        first_loop_updateTheta = false;
        th_incremental = 0;
    }
    else{
        th_incremental = th_motor - last_th_motor;
        last_th_motor = th_motor;
    }
    
    if (th_incremental > 4) th_incremental -= 6.28319;
    else if (th_incremental < -4) th_incremental += 6.28319;
    th_joint += th_incremental / gear_ratio * direction;
}

void rmd_motor::UpdateTheta(void) {
    th_motor = _torque_ctrl_encoder_fdback / tic2rad;
    if(first_loop_updateTheta){
        th_joint = zero_offset;
        last_th_motor = th_motor;
        first_loop_updateTheta = false;
    }
    th_incremental = th_motor - last_th_motor;
    last_th_motor = th_motor;
    if (th_incremental > 4) th_incremental -= 6.28319;
    else if (th_incremental < -4) th_incremental += 6.28319;
    th_joint += th_incremental / gear_ratio * direction;
}

float rmd_motor::GetTheta() {
    return th_joint;
}

float rmd_motor::GetThetaDot() {
    return _torque_ctrl_speed_fdback;
}


void rmd_motor::Board_GetTorqueData2(void) {
    int temp_enc = (int)(torque_data[6] | (torque_data[7]<<8));
    if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
    TorqueCommandEncoderValue = temp_enc;
}

// HRRLAB - CAN ID SETTING
void rmd_motor::Board_SetCANIDs(int bno, int can_ch){
    BOARD_ID = bno;
    CAN_CHANNEL = can_ch;
    ID_RMD_COMMAND = 0x140+bno;
}

void rmd_motor::Board_CANCheck(void){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = 0x01;
    mb.dlc = 1;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_InitControl(){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_INIT_CONTROL;
    mb.data[1] = 0;
    mb.dlc = 2;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_DQAlign(){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_DQ_SAVE;
    mb.data[1] = 1;
    mb.data[2] = 1;
    mb.dlc = 3;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_CurrentNulling(){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_CURRENT_NULLING;
    mb.data[1] = 0;
    mb.dlc = 2;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_SetHomeZeroOffset(int offset, int now_zero){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_HOME_ZERO_OFFSET;
    mb.data[1] = 0;
    mb.data[2] = offset & 0xFF;
    mb.data[3] = (offset>>8) & 0xFF;
    mb.data[4] = (offset>>16) & 0xFF;
    mb.data[5] = (offset>>24) & 0xFF;
    mb.data[6] = now_zero;
    mb.dlc = 7;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_RequestGain_FOC(){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_FOC_GAIN;
    mb.data[1] = 1;
    mb.dlc = 2;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_SetGain_FOC(int pgain, int igain, int mode){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    if(mode == 0) {
        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.dlc = 6;
        rmd_can::write_general_msg(mb);

        usleep(100*1000);

        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        rmd_can::write_general_msg(mb);
    }
    else if(mode == 1) {
        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.dlc = 6;
        rmd_can::write_general_msg(mb);
    }
    else if(mode == 2) {
        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        rmd_can::write_general_msg(mb);
    }
}

void rmd_motor::Board_RequestGain_POS(){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_POS_GAIN;
    mb.data[1] = 1;
    mb.dlc = 2;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_SetGain_POS(int pgain, int igain, int dgain, int mode){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;

    if(mode == 0) {
        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.data[6] = dgain & 0xFF;
        mb.data[7] = (dgain>>8) & 0xFF;
        mb.dlc = 8;
        rmd_can::write_general_msg(mb);

        usleep(10*1000);

        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        rmd_can::write_general_msg(mb);
    }
    else if(mode == 1) {
        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.data[6] = dgain & 0xFF;
        mb.data[7] = (dgain>>8) & 0xFF;
        mb.dlc = 8;
        rmd_can::write_general_msg(mb);
    }
    else if(mode == 2) {
        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        rmd_can::write_general_msg(mb);
    }
}

void rmd_motor::Board_DQ_Shift(int dir){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_DQ_SHIFT;
    mb.data[1] = dir;
    mb.dlc = 2;
    rmd_can::write_general_msg(mb);
}

void rmd_motor::Board_DQ_Manual(){
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = CMD_DQ_MANUAL;
    mb.dlc = 1;
    rmd_can::write_general_msg(mb);
}