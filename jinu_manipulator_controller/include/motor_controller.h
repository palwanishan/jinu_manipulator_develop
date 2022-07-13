#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "shared_memory.h"
#include "dynamics.h"

#define   tic2radL   2607.435432674516
#define   tic2radX   10430.21970545193

class Motor_Controller{

public:   

  int count;
  bool first_loop = true;

  float thetas[6];  
  VectorXd theta_rad = VectorXd::Zero(6);
  VectorXd theta_deg = VectorXd::Zero(6);
  VectorXd theta_raw = VectorXd::Zero(6);
  VectorXd theta_raw_0xA1 = VectorXd::Zero(6);
  VectorXd theta_balanced_0xA1 = VectorXd::Zero(6);
  VectorXd theta_balanced_rad_0xA1 = VectorXd::Zero(6);


  VectorXd th_joint = VectorXd::Zero(6);
  VectorXd last_th_joint = VectorXd::Zero(6);
  VectorXd th_dot_joint = VectorXd::Zero(6);
  VectorXd th_motor = VectorXd::Zero(6);
  VectorXd last_th_motor = VectorXd::Zero(6);
  VectorXd th_incremental = VectorXd::Zero(6);
  VectorXd th_dot = VectorXd::Zero(6);


  Motor_Controller();
  //~Motor_Controller();

  VectorXd GetThetaX();
  VectorXd GetThetaL();
  VectorXd GetTheta();
  VectorXd GetJointTheta();
  VectorXd GetThetaDot();
  void ReadTheta();    
  void SetTorque(VectorXd tau);  
  void SetPosition(VectorXd theta);  
  void EnableMotor();
};


#endif // MOTOR_CONTROLLER_H