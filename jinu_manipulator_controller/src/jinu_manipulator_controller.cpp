#include <iostream>
#include <unistd.h>
#include "spi2can.h"
#include "Common.h"
#include "rmd_utils.h"
#include "jinu_manipulator_controller.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

pRBCORE_SHM sharedData;

rmd_motor _DEV_MC[8];

static void *rt_motion_thread(void *arg);

ROBOT_STATE_DATA ros_data;

bool first_loop{true};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jinu_manipulator_controller");
    ros::Time::init();
    ros::Rate loop_rate(100);
    ros::NodeHandle node_handle_;
    ros::Publisher jinu_manipulator_joint_states_pub_;
    jinu_manipulator_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
    
    _DEV_MC[3].Board_SetCANIDs(3, 1);   _DEV_MC[3].DIR =  1;    _DEV_MC[3].gear_ratio = 6;  _DEV_MC[3].zeroManualOffset = 40;
    _DEV_MC[4].Board_SetCANIDs(4, 1);   _DEV_MC[4].DIR = -1;    _DEV_MC[4].gear_ratio = 8;  _DEV_MC[4].zeroManualOffset = -105;
    _DEV_MC[5].Board_SetCANIDs(5, 1);   _DEV_MC[5].DIR =  1;    _DEV_MC[5].gear_ratio = 6;  _DEV_MC[5].zeroManualOffset = 140;
    _DEV_MC[6].Board_SetCANIDs(6, 2);   _DEV_MC[6].DIR = -1;    _DEV_MC[6].gear_ratio = 1;  _DEV_MC[6].zeroManualOffset = 40;
    _DEV_MC[7].Board_SetCANIDs(7, 2);   _DEV_MC[7].DIR = -1;    _DEV_MC[7].gear_ratio = 1;  _DEV_MC[7].zeroManualOffset = 15;
    _DEV_MC[8].Board_SetCANIDs(8, 2);   _DEV_MC[8].DIR = -1;    _DEV_MC[8].gear_ratio = 1;  _DEV_MC[8].zeroManualOffset = 15;

    int encoder_resolution = 36000;    

    // double DEG2RAD{0.01745329252};

    for(uint8_t i=3; i<9; i++) 
    {
        _DEV_MC[i].PPR = _DEV_MC[i].DIR*encoder_resolution*_DEV_MC[i].gear_ratio/360.0;
    }

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
            msg.position.push_back(_DEV_MC[i+3].MeasuredPosition_deg * 0.01745329252);
            msg.velocity.push_back(0.0);
            msg.effort.push_back(0.0);
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

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while(true){

        clock_gettime(CLOCK_REALTIME, &TIME_TIC);

        ///--------------------------------------------------------------------------
        /// Add your code here
        ///--------------------------------------------------------------------------
        // if(first_loop)
        // {
        //     for(uint8_t i=0; i<6; i++)
        //     {
        //         sharedData->rmd_motor_run_flag[i] = true;

        //         _DEV_MC[i].ref_data[0] = 0x88 & 0xFF;
        //         _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
        //         _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
        //         _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
        //         _DEV_MC[i].ref_data[4] = 0x00 & 0xFF;
        //         _DEV_MC[i].ref_data[5] = 0x00 & 0xFF;
        //         _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
        //         _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
        //     }
        //     first_loop = false;
        // }


        for(uint8_t i=3; i<9; i++)
        {
            sharedData->rmd_motor_run_flag[i] = true;

            _DEV_MC[i].ref_data[0] = 0x92 & 0xFF;
            _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
            _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
            _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
            _DEV_MC[i].ref_data[4] = 0x00 & 0xFF;
            _DEV_MC[i].ref_data[5] = 0x00 & 0xFF;
            _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
            _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
        }

        ///--------------------------------------------------------------------------
        /// code end
        ///--------------------------------------------------------------------------

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
            cout << "RT Deadline Miss, main controller :  " << timediff_us(&TIME_NEXT, &TIME_NOW)*0.001<<" ms"<< endl;
        }
    }
}