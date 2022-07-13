#include "motor_controller.h"

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[8];

Motor_Controller::Motor_Controller(){
  _DEV_MC[3].Board_SetCANIDs(3, 1);   _DEV_MC[3].direction =  1;    _DEV_MC[3].gear_ratio = 6;  _DEV_MC[3].zero_offset = 0;          _DEV_MC[3].torqueToHex = 31.25;     _DEV_MC[3].tic2rad = 10430.2197;     
  _DEV_MC[4].Board_SetCANIDs(4, 1);   _DEV_MC[4].direction = -1;    _DEV_MC[4].gear_ratio = 8;  _DEV_MC[4].zero_offset = -3.14;        _DEV_MC[4].torqueToHex = 52.5201;   _DEV_MC[4].tic2rad = 10430.2197;
  _DEV_MC[5].Board_SetCANIDs(5, 1);   _DEV_MC[5].direction =  1;    _DEV_MC[5].gear_ratio = 6;  _DEV_MC[5].zero_offset = 2.84;    _DEV_MC[5].torqueToHex = 71;        _DEV_MC[5].tic2rad = 10430.2197;
  _DEV_MC[6].Board_SetCANIDs(6, 2);   _DEV_MC[6].direction = -1;    _DEV_MC[6].gear_ratio = 1;  _DEV_MC[6].zero_offset = 0.94;       _DEV_MC[6].torqueToHex = 272;   _DEV_MC[6].tic2rad = 2607.4354;
  _DEV_MC[7].Board_SetCANIDs(7, 2);   _DEV_MC[7].direction = -1;    _DEV_MC[7].gear_ratio = 1;  _DEV_MC[7].zero_offset = 0;        _DEV_MC[7].torqueToHex = 521;   _DEV_MC[7].tic2rad = 2607.4354;
  _DEV_MC[8].Board_SetCANIDs(8, 2);   _DEV_MC[8].direction = -1;    _DEV_MC[8].gear_ratio = 1;  _DEV_MC[8].zero_offset = 0;        _DEV_MC[8].torqueToHex = 521;       _DEV_MC[8].tic2rad = 2607.4354;

  _DEV_MC[9].Board_SetCANIDs(9, 2);   _DEV_MC[9].direction = -1;    _DEV_MC[9].gear_ratio = 1;  _DEV_MC[9].zero_offset = 0;        _DEV_MC[9].torqueToHex = 625;       _DEV_MC[9].tic2rad = 2607.4354;
}

void Motor_Controller::EnableMotor(){
  for(uint8_t i=3; i<10; i++) {
    sharedData->rmd_motor_run_flag[i] = true;
    _DEV_MC[i].first_loop_updateTheta = true;
    _DEV_MC[i].second_loop_updateTheta = true;
    _DEV_MC[i].ref_data[0] = 0x88 & 0xFF;
    _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[4] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[5] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
  }
}


VectorXd Motor_Controller::GetJointTheta(){
  for(uint8_t i=3; i<9; i++) th_joint[i-3] = _DEV_MC[i].GetTheta();      
  return th_joint;
}


VectorXd Motor_Controller::GetThetaDot(){
  for(uint8_t i=3; i<9; i++) th_dot[i-3] = _DEV_MC[i].GetThetaDot();      
  return th_dot;
}


void Motor_Controller::SetTorque(VectorXd tau){
  for(uint8_t i=3; i<9; i++) {
    long param = _DEV_MC[i].direction * _DEV_MC[i].torqueToHex * tau[i-3];
    _DEV_MC[i].ref_data[0] = 0xa1 & 0xFF;
    _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[4] = (param     ) & 0xFF;
    _DEV_MC[i].ref_data[5] = (param >> 8) & 0xFF;
    _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
  }
}