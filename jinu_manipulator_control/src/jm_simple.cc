#include "jm_simple.h"

namespace gazebo
{
  void JM_simple::Load(ModelPtr _model, sdf::ElementPtr)
  {
    this->model = _model;
    GetLinks();
    GetJoints();
    InitROSPubSetting();
    this->last_update_time = this->model->GetWorld()->SimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JM_simple::UpdateAlgorithm, this));
    std::cout << "Load..." << std::endl;
  }


  void JM_simple::GetLinks()
  {
    this->Base  = this->model->GetLink("link1");
    this->Link1 = this->model->GetLink("link2");
    this->Link2 = this->model->GetLink("link3");
    this->Link3 = this->model->GetLink("link4");
    this->Link4 = this->model->GetLink("link5");
    this->Link5 = this->model->GetLink("link6");
    this->Link6 = this->model->GetLink("link7");
  }


  void JM_simple::GetJoints()
  {
    this->Joint1 = this->model->GetJoint("joint1");
    this->Joint2 = this->model->GetJoint("joint2");
    this->Joint3 = this->model->GetJoint("joint3");
    this->Joint4 = this->model->GetJoint("joint4");
    this->Joint5 = this->model->GetJoint("joint5");
    this->Joint6 = this->model->GetJoint("joint6");
    this->gripper = this->model->GetJoint("gripper");
    this->gripper_sub = this->model->GetJoint("gripper_sub");
  }


  void JM_simple::InitROSPubSetting()
  {
    pub_joint_state = node_handle.advertise<sensor_msgs::JointState>("jinu_manipulator/joint_states", 100);
    sub_mode_selector = node_handle.subscribe("jinu_manipulator/mode_selector", 1, &gazebo::JM_simple::SwitchMode, this); 
    gain = node_handle.subscribe("jinu_manipulator/gain", 1, &gazebo::JM_simple::SwitchGain, this); 
    
    sub_open_manipulator_joint_state = node_handle.subscribe("joint_states", 1, &gazebo::JM_simple::OMJointStatesCallback, this); 
    sub_hmd_tf = node_handle.subscribe("unity/hmd_tf", 1, &gazebo::JM_simple::HMDTFCallback, this);

    pub_ee_pose = node_handle.advertise<geometry_msgs::TransformStamped>("jinu_manipulator/ee_pose", 10);
    pub_ref_ee_pose = node_handle.advertise<geometry_msgs::TransformStamped>("jinu_manipulator/ref_ee_pose", 10);
  }


  void JM_simple::UpdateAlgorithm()
  {
    current_time = this->model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    last_update_time = current_time;
      
    EncoderRead();               
    ROSMsgPublish();
    PostureGeneration();
    JointController();      
    GripperControl();    
  }


  void JM_simple::EncoderRead()
  {
    th[0] = this->Joint1->Position(2);
    th[1] = this->Joint2->Position(1);
    th[2] = this->Joint3->Position(1);
    th[3] = this->Joint4->Position(1);
    th[4] = this->Joint5->Position(0);
    th[5] = this->Joint6->Position(2);
  }


  void JM_simple::ROSMsgPublish()
  {
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();        
    for (uint8_t i=0; i<6; i++)
    {
      joint_state_msg.name.push_back((std::string)joint_names.at(i));
      joint_state_msg.position.push_back((float)(th[i]*rad2deg));
      joint_state_msg.velocity.push_back((float)(th_dot[i]));
      joint_state_msg.effort.push_back((float)joint_torque[i]);
    }
    pub_joint_state.publish(joint_state_msg); 


    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.stamp = ros::Time::now();
    ee_quaternion = ee_rotation;
    tf_msg.transform.translation.x = ee_position(0);
    tf_msg.transform.translation.y = ee_position(1);
    tf_msg.transform.translation.z = ee_position(2);
    tf_msg.transform.rotation.x = ee_quaternion.x();
    tf_msg.transform.rotation.y = ee_quaternion.y();
    tf_msg.transform.rotation.z = ee_quaternion.z();
    tf_msg.transform.rotation.w = ee_quaternion.w();

    pub_ee_pose.publish(tf_msg); 

    tf_msg.header.stamp = ros::Time::now();
    ref_ee_quaternion = ref_ee_rotation;
    tf_msg.transform.translation.x = ref_ee_position(0);
    tf_msg.transform.translation.y = ref_ee_position(1);
    tf_msg.transform.translation.z = ref_ee_position(2);
    tf_msg.transform.rotation.x = ref_ee_quaternion.x();
    tf_msg.transform.rotation.y = ref_ee_quaternion.y();
    tf_msg.transform.rotation.z = ref_ee_quaternion.z();
    tf_msg.transform.rotation.w = ref_ee_quaternion.w();

    pub_ref_ee_pose.publish(tf_msg); 
  }


  void JM_simple::PostureGeneration()
  {
    if (control_mode == IDLE) Idle();
    else if (control_mode == Motion_1) Motion1(); 
    else if (control_mode == Motion_2) Motion2();
    else if (control_mode == Motion_3) Motion3();
    else Idle();
  }


  void JM_simple::JointController()
  {
    this->Joint1->SetForce(2, joint_torque(0)); 
    this->Joint2->SetForce(1, joint_torque(1)); 
    this->Joint3->SetForce(1, joint_torque(2));
    this->Joint4->SetForce(1, joint_torque(3));
    this->Joint5->SetForce(0, joint_torque(4)); 
    this->Joint6->SetForce(2, joint_torque(5));
  }


  void JM_simple::SwitchMode(const std_msgs::Int32Ptr & msg)
  {
    cnt = 0;
    if      (msg -> data == 0) control_mode = IDLE;
    else if (msg -> data == 1) control_mode = Motion_1;
    else if (msg -> data == 2) control_mode = Motion_2;  
    else if (msg -> data == 3) control_mode = Motion_3;  
    else                       control_mode = IDLE;    
  }


  void JM_simple::SwitchGain(const std_msgs::Int32Ptr & msg)
  {
    cnt = 0;
    if (msg -> data > 0) input_P = msg -> data;  
    if (msg -> data < 0) input_D = - msg -> data;  
    pre_data_x = 0;
    pre_data_y = 0;
    pre_data_z = 0;
  }


  void JM_simple::Idle()
  {
    gain_p_joint_space << 100, 100, 100, 30, 30, 30;
    gain_d_joint_space << 1, 1, 1, 0.1, 0.1, 0.1;

    step_time = 4; 
    
    cnt_time = cnt * inner_dt;
    cnt++;
    trajectory = 0.5 * (1 - cos(PI * (cnt_time / step_time)));

    th_dot = (th - last_th) / dt;
    last_th = th;
    
    if(cnt_time <= step_time)
    {
      ref_th[0] =   0 * trajectory * deg2rad;
      ref_th[1] = -60 * trajectory * deg2rad;
      ref_th[2] =  60 * trajectory * deg2rad;
      ref_th[3] =   0 * trajectory * deg2rad;
      ref_th[4] =   0 * trajectory * deg2rad;
      ref_th[5] =   0 * trajectory * deg2rad; 
    }

    for (int i = 0; i < 6; i++) {
      joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i];
    }
  }

  //  Infinity Drawer
  void JM_simple::Motion1()
  { 
    gain_p << input_P, input_P, input_P;     
    gain_d_joint_space << 3, 5, 3, 0.2, 0.1, 0.1;
    //std::cout<< input_D<<std::endl;
    gain_w << 10, 10, 10;

    step_time = 6;
    
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
    
    if (cnt<1) initial_ee_position << ee_position(0), ee_position(1), ee_position(2);    

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.0;
      ref_ee_position(1) = initial_ee_position(1) - 0.2*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2) + 0.1*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = qw; ref_ee_quaternion.x() = qx; ref_ee_quaternion.y() = qy; ref_ee_quaternion.z() = qz;      
    }

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0));
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1));
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2));

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();    

    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);
    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);
  
    gravity_compensation[1] = 1.5698*sin(th[1])*sin(th[2]) - 1.5698*cos(th[1])*cos(th[2]) - 2.9226*cos(th[1]) + 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[2] = 1.5698*sin(th[1])*sin(th[2]) - 1.5698*cos(th[1])*cos(th[2]) + 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[3] = 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[4] = 0.056114*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[5] = 0.056114*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));

    if (cnt<1) last_th = th;
    th_dot = (th - last_th) / dt;
    last_th = th;
    for (int i = 0; i < 6; i++) viscous_damping[i] = gain_d_joint_space[i] * th_dot[i];
    
    joint_torque =  Jacobian.transpose() * virtual_spring + gravity_compensation - viscous_damping;

    // float temp_x = ee_position(0);
    // float temp_y = ee_position(1);
    // float temp_z = ee_position(2);   
    // float ref_temp_x = ref_ee_position(0);
    // float ref_temp_y = ref_ee_position(1);
    // float ref_temp_z = ref_ee_position(2);

    // if (abs(temp_x - ref_temp_x) > pre_data_x) pre_data_x = abs(temp_x - ref_temp_x);
    // if (abs(temp_y - ref_temp_y) > pre_data_y) pre_data_y = abs(temp_y - ref_temp_y);  
    // if (abs(temp_z - ref_temp_z) > pre_data_z) pre_data_z = abs(temp_z - ref_temp_z);
    // else std::cout << "x=" << pre_data_x << "y=" << pre_data_y << "z=" << pre_data_z << std::endl;
    
    cnt++;
  }

  // Open-Manipulator EE Pose follower
  void JM_simple::Motion2()
  {
    gain_p << 500, 500, 500;
    //gain_d << 1, 10, 10;
    gain_w << 10, 10, 10;

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
    //ee_velocity = (ee_position - pre_ee_position) / inner_dt;
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
  }

  //HMD Orientation follower
  void JM_simple::Motion3()
  {
    gain_p << 400, 400, 400;
    gain_d << 5, 5, 5;
    gain_w << 10, 10, 10;

    cnt++;
    cnt_time = cnt * dt;   

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
    if (cnt<1) pre_ee_position = ee_position; 
    ee_velocity = (ee_position - pre_ee_position) / dt;
    pre_ee_position = ee_position;

    ref_ee_position = hmd_position;

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0)) - gain_d(0) * ee_velocity(0);
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1)) - gain_d(1) * ee_velocity(1);
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2)) - gain_d(2) * ee_velocity(2);

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    ref_ee_rotation = hmd_quaternion.normalized().toRotationMatrix();    

    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);

    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) 
                        + ee_rotation_y.cross(ref_ee_rotation_y) 
                        + ee_rotation_z.cross(ref_ee_rotation_z);
    
    ee_momentum << gain_w(0) * ee_orientation_error(0), 
                  gain_w(1) * ee_orientation_error(1), 
                  gain_w(2) * ee_orientation_error(2);


    gravity_compensation[1] = 1.5698*sin(th[1])*sin(th[2]) - 1.5698*cos(th[1])*cos(th[2]) - 2.9226*cos(th[1]) + 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[2] = 1.5698*sin(th[1])*sin(th[2]) - 1.5698*cos(th[1])*cos(th[2]) + 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[3] = 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[4] = 0.056114*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    gravity_compensation[5] = 0.056114*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);
    joint_torque = Jacobian.transpose() * virtual_spring; // + gravity_compensation;
  }


  void JM_simple::OMJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    for(int i = 0; i < msg->name.size(); i++)
    {
           if(!msg->name.at(i).compare("joint1")) om_th[0] = (msg->position.at(i));
      else if(!msg->name.at(i).compare("joint2")) om_th[1] = (msg->position.at(i));
      else if(!msg->name.at(i).compare("joint3")) om_th[2] = (msg->position.at(i));
      else if(!msg->name.at(i).compare("joint4")) om_th[3] = (msg->position.at(i));
      else if(!msg->name.at(i).compare("joint5")) om_th[4] = (msg->position.at(i));
      else if(!msg->name.at(i).compare("joint6")) om_th[5] = (msg->position.at(i));
      else if(!msg->name.at(i).compare("gripper")) om_th[6] = (msg->position.at(i));
    }

    om_th[1] -= 1.57;
    om_th[2] += 0.85;
    om_th[3] += 0.30;

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


  void JM_simple::GripperControl()
  {
    VectorXd gripper_th(2), ref_gripper_th(2);

    gripper_th[0] = this->gripper->Position(1);
    gripper_th[1] = this->gripper_sub->Position(1);

    ref_gripper_th[0] = 0;
    ref_gripper_th[1] = 0;

    gripper_torque[0] = 10 * (ref_gripper_th[0] - gripper_th[0]);
    gripper_torque[1] = 10 * (ref_gripper_th[1] - gripper_th[1]);

    this->gripper->SetForce(1, gripper_torque(0));
    this->gripper_sub->SetForce(1, gripper_torque(1));
  }


  void JM_simple::HMDTFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    hmd_position.x() = msg->pose.position.x;
    hmd_position.y() = msg->pose.position.y;
    hmd_position.z() = msg->pose.position.z;

    hmd_quaternion.x() = msg->pose.orientation.x;
    hmd_quaternion.y() = msg->pose.orientation.y;
    hmd_quaternion.z() = msg->pose.orientation.z;
    hmd_quaternion.w() = msg->pose.orientation.w;
  }
}
