#include "dynamics.h"

namespace Dynamics
{
    JMDynamics::JMDynamics(){}    

    void JMDynamics::SetTheta(VectorXd thetas)
    {
        for(uint8_t i=0; i<6; i++)
        {
            th[i] = thetas[i];
        }
    }


    void JMDynamics::SetOMTheta(VectorXd thetas)
    {
        om_th = thetas;

        om_th[1] -= 1.57;
        om_th[2] += 0.85;
        om_th[3] += 0.30;
    }


    VectorXd JMDynamics::GetTorque()
    {   
        return joint_torque;
    }


    void JMDynamics::GenerateTorque_TaskSpacePD(){   
        gain_p << 100, 100, 100;
        gain_d << 5, 5, 5;
        gain_w << 1, 1, 1;

        cnt_time = cnt*inner_dt;   

        A0 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        A1 << cos(th[0]), 0, -sin(th[0]), 0,
            sin(th[0]), 0, cos(th[0]), 0,
            0, -1, 0, L1,
            0, 0, 0, 1;
        A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
            sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
            0, 0, 1, 0, 
            0, 0, 0, 1;
        A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
            sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
            0, 0, 1, 0,
            0, 0, 0, 1;
        A4 << sin(th[3]), 0, cos(th[3]), 0,
            -cos(th[3]), 0, sin(th[3]), 0,
            0, -1, 0, 0,
            0, 0, 0, 1;
        A5 << -sin(th[4]), 0, cos(th[4]), 0,
            cos(th[4]), 0, sin(th[4]), 0,
            0, 1, 0, L5,
            0, 0, 0, 1;
        A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
            cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
            0, 0, 1, 0, 
            0, 0, 0, 1;          
            
        T00 = A0;
        T01 = T00*A1;
        T02 = T01*A2;
        T03 = T02*A3;
        T04 = T03*A4;
        T05 = T04*A5;
        T06 = T05*A6;
    
        a0 << T00(0,2), T00(1,2), T00(2,2);
        a1 << T01(0,2), T01(1,2), T01(2,2);
        a2 << T02(0,2), T02(1,2), T02(2,2);
        a3 << T03(0,2), T03(1,2), T03(2,2);
        a4 << T04(0,2), T04(1,2), T04(2,2);
        a5 << T05(0,2), T05(1,2), T05(2,2);

        P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
        P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
        P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
        P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
        P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
        P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

        J1 << a0.cross(P6_P0), a0;
        J2 << a1.cross(P6_P1), a1;
        J3 << a2.cross(P6_P2), a2;
        J4 << a3.cross(P6_P3), a3;
        J5 << a4.cross(P6_P4), a4;
        J6 << a5.cross(P6_P5), a5;

        Jacobian << J1, J2, J3, J4, J5, J6;

        ee_position << T06(0,3), T06(1,3), T06(2,3);
        ee_velocity = (ee_position - pre_ee_position) / inner_dt;
        pre_ee_position = ee_position;

        ee_force(0) = gain_p(0) * (om_ee_position(0) - ee_position(0)); // - gain_d(0) * ee_velocity(0);
        ee_force(1) = gain_p(1) * (om_ee_position(1) - ee_position(1)); // - gain_d(1) * ee_velocity(1);
        ee_force(2) = gain_p(2) * (om_ee_position(2) - ee_position(2)); // - gain_d(2) * ee_velocity(2);

        ee_rotation = T06.block<3,3>(0,0);
        ee_rotation_x = ee_rotation.block<3,1>(0,0); 
        ee_rotation_y = ee_rotation.block<3,1>(0,1); 
        ee_rotation_z = ee_rotation.block<3,1>(0,2);

        ref_ee_rotation_x = om_ee_rotation.block<3,1>(0,0); 
        ref_ee_rotation_y = om_ee_rotation.block<3,1>(0,1); 
        ref_ee_rotation_z = om_ee_rotation.block<3,1>(0,2);

        ee_orientation_error  = ee_rotation_x.cross(ref_ee_rotation_x) 
                              + ee_rotation_y.cross(ref_ee_rotation_y) 
                              + ee_rotation_z.cross(ref_ee_rotation_z);

        ee_momentum << gain_w(0) * ee_orientation_error(0), 
                        gain_w(1) * ee_orientation_error(1), 
                        gain_w(2) * ee_orientation_error(2);

        virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);

        joint_torque = Jacobian.transpose() * virtual_spring;

        //std::cout << "joint 4: " << joint_torque[3] << "   joint 5: " << joint_torque[4] << "   joint 6: "<< joint_torque[5] << std::endl;
    }


    void JMDynamics::GenerateTrajectory(){
        float count_time = count * dt;     
        count++;   

        trajectory = 1 * (1 - cos(PI * (count_time/step_time)));

        // trajectory = 0.1 * sin(PI * (count_time/step_time));
    }


    void JMDynamics::GenerateTorque_JointSpacePD(double deltaT)
    {
        th_incremental = th - last_th;
        last_th = th;
        for (int i = 0; i < 6; i++) {
            if (th_incremental[i] > 4) th_incremental[i] -= 6.28319;
            else if (th_incremental[i] < -4) th_incremental[i] += 6.28319;

            if (i == 0) th_joint[i] += th_incremental[i] / 6;
            else if (i == 1) th_joint[i] += th_incremental[i] / 8;
            else if (i == 2) th_joint[i] += th_incremental[i] / 6;
            else th_joint[i] += th_incremental[i];
        }
        th_dot_joint = (th_joint - last_th_joint) / 0.002;
        last_th_joint = th_joint;

        // gain_p_joint_space << 0,    0,      0,      0,    0,    0.3;
        // gain_d_joint_space << 0,    0,      0,      0,    0,    0.01;
        gain_p_joint_space[5] = 0.3;
        gain_d_joint_space[5] = 0.01;

        // th_dot = (th - last_th) / 0.002;
        // last_th = th;

        ref_th << 0, 0, 0, 0, 0, 0;

        ref_th[0] = trajectory;

        for (int i = 0; i < 6; i++) 
        {
            // joint_torque[i] = gain_p_joint_space[i] * (om_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i];  
            // joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i];  

            joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th_joint[i]) - gain_d_joint_space[i] * th_dot_joint[i]; 
            // joint_torque[i] = gain_p_joint_space[i] * (om_th[i] - th_joint[i]) - gain_d_joint_space[i] * th_dot_joint[i]; 
        }
    }


    void JMDynamics::CalculateRefEEPose()
    {
        om_A0 << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        om_A1 << cos(om_th[0]), 0, -sin(om_th[0]), 0,
                sin(om_th[0]), 0, cos(om_th[0]), 0,
                0, -1, 0, L1,
                0, 0, 0, 1;
        om_A2 << cos(om_th[1]), -sin(om_th[1]), 0, L2*cos(om_th[1]),
                sin(om_th[1]), cos(om_th[1]), 0, L2*sin(om_th[1]),
                0, 0, 1, 0, 
                0, 0, 0, 1;
        om_A3 << cos(om_th[2]), -sin(om_th[2]), 0, L3*cos(om_th[2]), 
                sin(om_th[2]), cos(om_th[2]), 0, L3*sin(om_th[2]), 
                0, 0, 1, 0,
                0, 0, 0, 1;
        om_A4 << sin(om_th[3]), 0, cos(om_th[3]), 0,
                -cos(om_th[3]), 0, sin(om_th[3]), 0,
                0, -1, 0, 0,
                0, 0, 0, 1;
        om_A5 << -sin(om_th[4]), 0, cos(om_th[4]), 0,
                cos(om_th[4]), 0, sin(om_th[4]), 0,
                0, 1, 0, L5,
                0, 0, 0, 1;
        om_A6 << -sin(om_th[5]), -cos(om_th[5]), 0, -L6*sin(om_th[5]),
                cos(om_th[5]), -sin(om_th[5]), 0, L6*cos(om_th[5]),
                0, 0, 1, 0, 
                0, 0, 0, 1;          
            
        om_T06 = om_A0*om_A1*om_A2*om_A3*om_A4*om_A5*om_A6;

        om_ee_position = om_T06.block<3,1>(0,3);
        om_ee_rotation = om_T06.block<3,3>(0,0);
    }
}