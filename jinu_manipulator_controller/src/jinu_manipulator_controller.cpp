#include <iostream>
#include <unistd.h>
#include "spi2can.h"
#include "Common.h"
#include "rmd_utils.h"
#include "jinu_manipulator_controller.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "dynamics.h"
#include "motor_controller.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"

static void *rt_motion_thread(void *arg);
pRBCORE_SHM sharedData;
rmd_motor _DEV_MC[9];
ROBOT_STATE_DATA ros_data;
Dynamics::JMDynamics jm_dynamics;
Motor_Controller motor_ctrl;
bool first_loop{true};
VectorXd om_th(6);
// FILE *Joint_Space_PD_data1;

void HMDTFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    jm_dynamics.hmd_position.x() = msg->pose.position.x;
    jm_dynamics.hmd_position.y() = msg->pose.position.y;
    jm_dynamics.hmd_position.z() = msg->pose.position.z;

    jm_dynamics.hmd_quaternion.x() = msg->pose.orientation.x;
    jm_dynamics.hmd_quaternion.y() = msg->pose.orientation.y;
    jm_dynamics.hmd_quaternion.z() = msg->pose.orientation.z;
    jm_dynamics.hmd_quaternion.w() = msg->pose.orientation.w;
  }


void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg){
    
    VectorXd om_th(6);
    for(int i = 0; i < msg->name.size(); i++)
    {
             if(!msg->name.at(i).compare("joint1")) om_th[0] = (msg->position.at(i));
        else if(!msg->name.at(i).compare("joint2")) om_th[1] = (msg->position.at(i));
        else if(!msg->name.at(i).compare("joint3")) om_th[2] = (msg->position.at(i));
        else if(!msg->name.at(i).compare("joint4")) om_th[3] = (msg->position.at(i));
        else if(!msg->name.at(i).compare("joint5")) om_th[4] = (msg->position.at(i));
        else if(!msg->name.at(i).compare("joint6")) om_th[5] = (msg->position.at(i));
    }
    jm_dynamics.SetOMTheta(om_th);
}

void SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg){
    jm_dynamics.gain_p_task_space[0] = msg->data.at(0);
    jm_dynamics.gain_p_task_space[1] = msg->data.at(1);
    jm_dynamics.gain_p_task_space[2] = msg->data.at(2);
}

void SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg){
    jm_dynamics.gain_w_task_space[0] = msg->data.at(0);
    jm_dynamics.gain_w_task_space[1] = msg->data.at(1);
    jm_dynamics.gain_w_task_space[2] = msg->data.at(2);
}

void SwitchGainP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    jm_dynamics.gain_p_joint_space[0] = msg -> data.at(0);
    jm_dynamics.gain_p_joint_space[1] = msg -> data.at(1);
    jm_dynamics.gain_p_joint_space[2] = msg -> data.at(2);
    jm_dynamics.gain_p_joint_space[3] = msg -> data.at(3);
    jm_dynamics.gain_p_joint_space[4] = msg -> data.at(4);
    jm_dynamics.gain_p_joint_space[5] = msg -> data.at(5);
}

void SwitchGainD(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    jm_dynamics.gain_d_joint_space[0] = msg -> data.at(0);
    jm_dynamics.gain_d_joint_space[1] = msg -> data.at(1);
    jm_dynamics.gain_d_joint_space[2] = msg -> data.at(2);
    jm_dynamics.gain_d_joint_space[3] = msg -> data.at(3);
    jm_dynamics.gain_d_joint_space[4] = msg -> data.at(4);
    jm_dynamics.gain_d_joint_space[5] = msg -> data.at(5);
}

void InitializePose(const std_msgs::BoolConstPtr &msg){
    if(msg->data) for(uint8_t i=3; i<9; i++) _DEV_MC[i].first_loop_updateTheta = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jinu_manipulator_controller");
    ros::Time::init();
    ros::Rate loop_rate(100);
    ros::NodeHandle node_handle_;
    ros::Publisher jinu_manipulator_joint_states_pub_;
    ros::Subscriber om_manipulator_joint_states_sub_;
    jinu_manipulator_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("jinu_manipulator/joint_states", 100);
    om_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, joint_states_callback);

    ros::Subscriber gain_p_sub_;
    gain_p_sub_ = node_handle_.subscribe("st_arm/gain_p", 10, SwitchGainP);
    ros::Subscriber gain_d_sub_;
    gain_d_sub_ = node_handle_.subscribe("st_arm/gain_d", 10, SwitchGainD);

    ros::Subscriber gain_task_space_p_sub_;
    gain_task_space_p_sub_ = node_handle_.subscribe("st_arm/gain_task_space_p", 10, SwitchGainTaskSpaceP);

    ros::Subscriber gain_task_space_w_sub_;
    gain_task_space_w_sub_ = node_handle_.subscribe("st_arm/gain_task_space_w", 10, SwitchGainTaskSpaceW);

    ros::Subscriber pose_initializer_sub_;
    pose_initializer_sub_ = node_handle_.subscribe("st_arm/initialize_pose", 10, InitializePose);

    ros::Subscriber virtual_box_pose_sub_;
    virtual_box_pose_sub_ = node_handle_.subscribe("unity/virtual_box_pose", 10, HMDTFCallback);

    // Joint_Space_PD_data1 = fopen("/home/rainbow/catkin_ws/src/data1.dat","w");

    spi2can::getInstance();

    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));

    pthread_t thread_motion;

    int thread_id_motion = generate_rt_thread(thread_motion, rt_motion_thread, "motion_thread", 3, 97, NULL);

    while(ros::ok())
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        std::vector<string> joints_name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        for (uint8_t i = 0; i<6; i ++)
        {
            msg.name.push_back(joints_name.at(i));
            msg.position.push_back(jm_dynamics.th[i]);
            // msg.position.push_back(jm_dynamics.th_joint[i]); 
            // msg.velocity.push_back(jm_dynamics.ref_th[i]);
            // msg.velocity.push_back(jm_dynamics.om_th[i]);
            msg.velocity.push_back(jm_dynamics.th_dot[i]);
            msg.effort.push_back(jm_dynamics.joint_torque[i]);
        }
        jinu_manipulator_joint_states_pub_.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void *rt_motion_thread(void *arg){
    unsigned long mpc_counter = 0;
    const long PERIOD_US = RT_MS * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_TIC;
    struct timespec TIME_TOC;
    int loop_count = 0;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    bool is_first_loop = true;

    while(true){
        clock_gettime(CLOCK_REALTIME, &TIME_TIC);

        if(loop_count < 2002) loop_count++;

        // if(loop_count == 1000){
        //     // motor_ctrl.SetTorque(jm_dynamics.zero_vector_6);
        // }
        if(!is_first_loop && loop_count > 1000){
            // jm_dynamics.GenerateTrajectory();
            // jm_dynamics.SetTheta(motor_ctrl.GetTheta());
            jm_dynamics.SetTheta(motor_ctrl.GetJointTheta());
            jm_dynamics.SetThetaDot(motor_ctrl.GetThetaDot());
            // jm_dynamics.CalculateRefEEPose();
            // jm_dynamics.GenerateTorque_GravityCompensation();
            // jm_dynamics.GenerateTorque_TaskSpacePD();
            jm_dynamics.GenerateTorque_VirtualBox();
            // jm_dynamics.GenerateTorque_JointSpacePD(1);
            motor_ctrl.SetTorque(jm_dynamics.GetTorque());
            // motor_ctrl.SetTorque(jm_dynamics.zero_vector_6);

            // fprintf(Joint_Space_PD_data1, "%lf %lf %lf \n", jm_dynamics.th_joint[0], jm_dynamics.ref_th[0], jm_dynamics.joint_torque[0]);         
            // if(loop_count > 1000000) fclose(Joint_Space_PD_data1);
        }

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if(is_first_loop){
            motor_ctrl.EnableMotor();
            timespec_add_us(&TIME_NEXT, 4 * 1000 * 1000);
            is_first_loop = false;
        }
        

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
            cout << "RT Deadline Miss, main controller :  " << timediff_us(&TIME_NEXT, &TIME_NOW)*0.001<<" ms"<< endl;
        }
    }
}
