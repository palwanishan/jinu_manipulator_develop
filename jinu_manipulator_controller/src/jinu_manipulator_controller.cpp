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

static void *rt_motion_thread(void *arg);
pRBCORE_SHM sharedData;
rmd_motor _DEV_MC[8];
ROBOT_STATE_DATA ros_data;
Dynamics::JMDynamics jm_dynamics;
Motor_Controller motor_ctrl;
bool first_loop{true};
VectorXd om_th(6);

void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg){
    
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

void SwitchGainP(const std_msgs::Float32ConstPtr &msg)
{
    jm_dynamics.gain_p_joint_space[3] = msg -> data;
    jm_dynamics.gain_p_joint_space[4] = msg -> data;
    jm_dynamics.gain_p_joint_space[5] = msg -> data;
}

void SwitchGainD(const std_msgs::Float32ConstPtr &msg)
{
    jm_dynamics.gain_d_joint_space[3] = msg -> data;
    jm_dynamics.gain_d_joint_space[4] = msg -> data;
    jm_dynamics.gain_d_joint_space[5] = msg -> data;
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
            msg.velocity.push_back(jm_dynamics.th_dot[i]);
            // msg.velocity.push_back(_DEV_MC[i+3].MeasuredPosition_deg);
            msg.effort.push_back(jm_dynamics.joint_torque[i]);  //_torque_ctrl_encoder_fdback
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

        if(loop_count < 1001) loop_count++;

        if(!is_first_loop && loop_count > 1000){
            //std::cout << "looping" << std::endl;
            // motor_ctrl.ReadTheta();
            jm_dynamics.SetTheta(motor_ctrl.GetTheta());
            jm_dynamics.GenerateTorque_JointSpacePD(1);
            motor_ctrl.SetTorque(jm_dynamics.GetTorque());
        }

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if(is_first_loop){
            motor_ctrl.EnableMotor();
            timespec_add_us(&TIME_NEXT, 5 * 1000 * 1000);
            is_first_loop = false;
            //std::cout << "Motors are enabled!" << std::endl;
        }
        

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
            cout << "RT Deadline Miss, main controller :  " << timediff_us(&TIME_NEXT, &TIME_NOW)*0.001<<" ms"<< endl;
        }
    }
}


