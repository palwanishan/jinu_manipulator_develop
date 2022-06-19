#ifndef JINU_MANIPULATOR_CONTROL_PLUGIN_H
#define JINU_MANIPULATOR_CONTROL_PLUGIN_H


#include <eigen3/Eigen/Dense>
#include <ignition/math.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

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

using gazebo::physics::ModelPtr;
using gazebo::physics::LinkPtr;
using gazebo::sensors::ImuSensorPtr;
using gazebo::sensors::SensorPtr;
using gazebo::physics::JointPtr;
using gazebo::event::ConnectionPtr;
using gazebo::common::Time;

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

#define NUM_OF_JOINT_AND_TOOL 7

#define PI 3.14159265358979
#define M2R 2*PI/4096
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
#define g 9.81;

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

//*************** Trajectory Variables**************//    
double step_time = 0;
double cnt_time = 0;
unsigned int cnt = 0;
unsigned int start_flag = 0;

float qw = 1;
float qx = 0;
float qy = 0;
float qz = 0;


namespace gazebo
{
  class JM_simple : public ModelPlugin
  {
    ModelPtr model;
    LinkPtr Base, Link1, Link2, Link3, Link4, Link5, Link6;
    JointPtr Joint1, Joint2, Joint3, Joint4, Joint5, Joint6;

    LinkPtr gripper_link, gripper_link_sub;
    JointPtr gripper, gripper_sub;

    const std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    common::Time last_update_time;
    common::Time current_time;
    event::ConnectionPtr update_connection;
    double dt;
    double time = 0;
    double trajectory;

    Vector3d ee_position, ee_velocity, pre_ee_position, ref_ee_position, initial_ee_position;
    Vector3d gain_p, gain_d, gain_w;
    VectorXd gain_p_joint_space = VectorXd(6);
    VectorXd gain_d_joint_space = VectorXd(6);

    Vector3d ee_rotation_x, ee_rotation_y, ee_rotation_z, ref_ee_rotation_x, ref_ee_rotation_y, ref_ee_rotation_z;
    Vector3d ee_orientation_error, ee_force, ee_momentum;

    Matrix3d ee_rotation;
    Matrix3d ref_ee_rotation;

    Quaterniond ref_ee_quaternion;  
    Quaterniond ee_quaternion;  

    Quaterniond hmd_quaternion;

    VectorXd th = VectorXd::Zero(6);
    VectorXd ref_th = VectorXd::Zero(6);
    VectorXd last_th = VectorXd::Zero(6);
    VectorXd th_dot = VectorXd::Zero(6);
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
    VectorXd joint_torque = VectorXd::Zero(8);
    VectorXd virtual_spring = VectorXd::Zero(6);

    // Temporary variables
    Quaterniond om_ee_quaternion;
    Vector3d om_ee_position;
    Matrix3d om_ee_rotation;
    MatrixXd om_A0 = MatrixXd::Zero(4,4); MatrixXd om_A1 = MatrixXd::Zero(4,4); MatrixXd om_A2 = MatrixXd::Zero(4,4); 
    MatrixXd om_A3 = MatrixXd::Zero(4,4); MatrixXd om_A4 = MatrixXd::Zero(4,4);  MatrixXd om_A5 = MatrixXd::Zero(4,4); 
    MatrixXd om_A6 = MatrixXd::Zero(4,4); MatrixXd om_T06 = MatrixXd::Zero(4,4);
    // End of Temporary variables

    ros::NodeHandle node_handle;
    ros::Publisher pub_joint_state;
    ros::Subscriber sub_mode_selector;
    ros::Subscriber sub_open_manipulator_joint_state;
    ros::Publisher pub_ee_pose;
    ros::Publisher pub_ref_ee_pose;
    ros::Subscriber sub_hmd_tf; 

    std::vector<double> present_joint_angle_;

    VectorXd om_th = VectorXd::Zero(7);

    enum ControlMode
    {
      IDLE = 0,
      Motion_1,
      Motion_2,
      Motion_3
    };
    enum ControlMode control_mode;

    void Load(ModelPtr _model, sdf::ElementPtr);
    void UpdateAlgorithm();
    void GetLinks();
    void GetJoints();
    void InitROSPubSetting();

    void EncoderRead();
    void JointController();
    void ROSMsgPublish();

    void PostureGeneration();
    void Idle();
    void Motion1();
    void Motion2();
    void Motion3();
    void SwitchMode(const std_msgs::Int32Ptr & msg);
    void OMJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void GripperControl();
    void HMDTFCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  };
  GZ_REGISTER_MODEL_PLUGIN(JM_simple);
}

#endif  // end of the JINU_MANIPULATOR_CONTROL_PLUGIN_H