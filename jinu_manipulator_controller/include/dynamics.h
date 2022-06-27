#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "Common.h"

#include <eigen3/Eigen/Dense>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;


#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
#define g           9.81;       

#define L1 0.16
#define L2 0.25
#define L3 0.25
#define L4 0.00
#define L5 0.10
#define L6 0.06

#define m_Link1 0.44951
#define m_Link2 0.91713
#define m_Link3 0.81213
#define m_Link4 0.174
#define m_Link5 0.31085
#define m_Link6 0.28174

#define m_Arm 2.94536 // (m_Link1~6 합친거)
#define M1 2.94536 // m_Arm
#define M2 2.49585 //m_Link2+m_Link3+m_Link4+m_Link5+m_Link6;
#define M3 1.57872 //m_Link3+m_Link4+m_Link5+m_Link6;
#define M4 0.76659 //m_Link4+m_Link5+m_Link6;
#define M5 0.59259 //m_Link5+m_Link6;
#define M6 0.28174 //m_Link6;
#define inner_dt 0.001



namespace Dynamics
{
    class JMDynamics
    {

        const std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        double dt = 0.002;
        double time = 0;
        double trajectory;

        double cnt_time = 0;
        unsigned int cnt = 0;   

        Vector3d ee_position, ee_velocity, pre_ee_position, ref_ee_position, initial_ee_position, hmd_position;
        Vector3d gain_p, gain_d, gain_w;

        Vector3d ee_rotation_x, ee_rotation_y, ee_rotation_z, ref_ee_rotation_x, ref_ee_rotation_y, ref_ee_rotation_z;
        Vector3d ee_orientation_error, ee_force, ee_momentum;

        Matrix3d ee_rotation;
        Matrix3d ref_ee_rotation;

        Quaterniond ref_ee_quaternion;  
        Quaterniond ee_quaternion;  
        Quaterniond hmd_quaternion;

        VectorXd initial_pose = VectorXd::Zero(6);
        
        MatrixXd A0 = MatrixXd::Zero(4,4);  MatrixXd A1 = MatrixXd::Zero(4,4); MatrixXd A2 = MatrixXd::Zero(4,4); 
        MatrixXd A3 = MatrixXd::Zero(4,4);
        MatrixXd A4 = MatrixXd::Zero(4,4);  MatrixXd A5 = MatrixXd::Zero(4,4); MatrixXd A6 = MatrixXd::Zero(4,4);
        MatrixXd T00 = MatrixXd::Zero(4,4); MatrixXd T01 = MatrixXd::Zero(4,4); MatrixXd T02 = MatrixXd::Zero(4,4); 
        MatrixXd T03 = MatrixXd::Zero(4,4);
        MatrixXd T04 = MatrixXd::Zero(4,4); MatrixXd T05 = MatrixXd::Zero(4,4); MatrixXd T06 = MatrixXd::Zero(4,4);
        MatrixXd T12 = MatrixXd::Zero(4,4); MatrixXd T23 = MatrixXd::Zero(4,4); MatrixXd T34 = MatrixXd::Zero(4,4); 
        MatrixXd T45 = MatrixXd::Zero(4,4); MatrixXd T56 = MatrixXd::Zero(4,4);    
        Vector3d a0, a1, a2, a3, a4, a5, P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;

        MatrixXd Jacobian = MatrixXd::Zero(6,6);    
        VectorXd J1 = VectorXd::Zero(6); VectorXd J2 = VectorXd::Zero(6); VectorXd J3 = VectorXd::Zero(6);
        VectorXd J4 = VectorXd::Zero(6); VectorXd J5 = VectorXd::Zero(6); VectorXd J6 = VectorXd::Zero(6); 

        VectorXd gravity_compensation = VectorXd::Zero(6);
        VectorXd gripper_torque = VectorXd::Zero(2);
        VectorXd virtual_spring = VectorXd::Zero(6);

        // Temporary variables
        Quaterniond om_ee_quaternion;
        Vector3d om_ee_position;
        Matrix3d om_ee_rotation;
        MatrixXd om_A0 = MatrixXd::Zero(4,4); MatrixXd om_A1 = MatrixXd::Zero(4,4); MatrixXd om_A2 = MatrixXd::Zero(4,4); 
        MatrixXd om_A3 = MatrixXd::Zero(4,4); MatrixXd om_A4 = MatrixXd::Zero(4,4);  MatrixXd om_A5 = MatrixXd::Zero(4,4); 
        MatrixXd om_A6 = MatrixXd::Zero(4,4); MatrixXd om_T06 = MatrixXd::Zero(4,4);
        // End of Temporary variables


        std::vector<double> present_joint_angle_;

        

    public:
        JMDynamics();
        //~JMDynamics();
        VectorXd zero_th = VectorXd::Zero(6);
        VectorXd th = VectorXd::Zero(6);
        VectorXd ref_th = VectorXd::Zero(6);
        VectorXd last_th = VectorXd::Zero(6);
        VectorXd th_dot = VectorXd::Zero(6);
        VectorXd last_th_dot = VectorXd::Zero(6);

        VectorXd om_th = VectorXd::Zero(6);
        VectorXd joint_torque = VectorXd::Zero(6);

        VectorXd gain_p_joint_space = VectorXd(6);
        VectorXd gain_d_joint_space = VectorXd(6);

        VectorXd GetTorque();
        void SetTheta(VectorXd thetas);
        void CalculateRefEEPose();
        //void OM_joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg);4
        void SetOMTheta(VectorXd thetas);
        void GenerateTorque_JointSpacePD(double deltaT);
        void GenerateTorque_TaskSpacePD();

    };
}


#endif // DYNAMICS_H