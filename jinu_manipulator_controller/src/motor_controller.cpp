#include "motor_controller.h"

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[8];

Motor_Controller::Motor_Controller(){
  _DEV_MC[3].Board_SetCANIDs(3, 1);   _DEV_MC[3].DIR =  1;    _DEV_MC[3].gear_ratio = 6;  _DEV_MC[3].zeroManualOffset = 0;      _DEV_MC[3].torqueToHex = 31.25;     
  _DEV_MC[4].Board_SetCANIDs(4, 1);   _DEV_MC[4].DIR = -1;    _DEV_MC[4].gear_ratio = 8;  _DEV_MC[4].zeroManualOffset = 0;      _DEV_MC[4].torqueToHex = 52.5201;   
  _DEV_MC[5].Board_SetCANIDs(5, 1);   _DEV_MC[5].DIR =  1;    _DEV_MC[5].gear_ratio = 6;  _DEV_MC[5].zeroManualOffset = 0;      _DEV_MC[5].torqueToHex = 71;   
  _DEV_MC[6].Board_SetCANIDs(6, 2);   _DEV_MC[6].DIR = -1;    _DEV_MC[6].gear_ratio = 1;  _DEV_MC[6].zeroManualOffset = -0.5;   _DEV_MC[6].torqueToHex = -271.739; 
  _DEV_MC[7].Board_SetCANIDs(7, 2);   _DEV_MC[7].DIR = -1;    _DEV_MC[7].gear_ratio = 1;  _DEV_MC[7].zeroManualOffset = 0.3;    _DEV_MC[7].torqueToHex = -520.833; 
  _DEV_MC[8].Board_SetCANIDs(8, 2);   _DEV_MC[8].DIR = -1;    _DEV_MC[8].gear_ratio = 1;  _DEV_MC[8].zeroManualOffset = 0.3;    _DEV_MC[8].torqueToHex = -625; 
  int encoder_resolution = 36000;    
  for(uint8_t i=3; i<9; i++) {
    _DEV_MC[i].PPR = _DEV_MC[i].DIR*encoder_resolution*_DEV_MC[i].gear_ratio/360.0;
  }
}

void Motor_Controller::EnableMotor(){
  for(uint8_t i=3; i<9; i++) {  
    sharedData->rmd_motor_run_flag[i] = true;
  }
  for(uint8_t i=3; i<6; i++) {
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

VectorXd Motor_Controller::GetTheta(){
  for(uint8_t i=0; i<3; i++)  { theta_balanced_rad_0xA1[i] = (_DEV_MC[i+3]._torque_ctrl_encoder_fdback - 65534/2) / tic2radX + _DEV_MC[i+3].zeroManualOffset;    }
  // for(uint8_t i=0; i<3; i++)  { theta_balanced_rad_0xA1[i] = theta_rad[i];    }
  for(uint8_t i=3; i<6; i++)  { theta_balanced_rad_0xA1[i] = (16382/2 - _DEV_MC[i+3]._torque_ctrl_encoder_fdback) / tic2radL + _DEV_MC[i+3].zeroManualOffset;    }
  return theta_balanced_rad_0xA1;
}

void Motor_Controller::ReadTheta(){
  for(uint8_t i=3; i<6; i++) {
    _DEV_MC[i].ref_data[0] = 0x92 & 0xFF;
    _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[4] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[5] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
  }
  for(uint8_t i=0; i<3; i++){
    thetas[i] = _DEV_MC[i+3].MeasuredPosition_deg;
    theta_deg[i] = _DEV_MC[i+3].MeasuredPosition_deg;
    theta_rad[i] = _DEV_MC[i+3].MeasuredPosition_deg * deg2rad;
    theta_raw[i] = _DEV_MC[i+3].EncoderValue;
  }
}

void Motor_Controller::SetTorque(VectorXd tau){
  for(uint8_t i=3; i<9; i++) {
    long param = _DEV_MC[i].torqueToHex * tau[i-3];
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

void Motor_Controller::SetPosition(VectorXd theta){
  for(uint8_t i=6; i<9; i++) {
    long param = -5729.57 * theta[i-3];
    _DEV_MC[i].ref_data[0] = 0xa3 & 0xFF;
    _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[4] = (param      ) & 0xFF;
    _DEV_MC[i].ref_data[5] = (param >>  8) & 0xFF;
    _DEV_MC[i].ref_data[6] = (param >> 16) & 0xFF;
    _DEV_MC[i].ref_data[7] = (param >> 24) & 0xFF;
  }
}    