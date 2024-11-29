#include "tocabi_controller/shhand_controller.h"
#include <iostream>
#include <fstream>
#include <string>

void print_array(float* arr, int len){
    for(int i = 0; i < len; i++){
        std::cout << arr[i] << " ";
    }
    std::cout << std::endl;
}

void waitForAllParams(ros::NodeHandle &nh, const std::vector<std::string> &param_names) {
    bool all_params_found = false;

    while (!all_params_found) {
        all_params_found = true;

        // 각 파라미터가 설정되었는지 확인
        for (const auto& param_name : param_names) {
            if (!nh.hasParam(param_name)) {
                std::cout << "Waiting for parameter: " << param_name << std::endl;
                all_params_found = false;
            }
        }

        // 모든 파라미터가 설정되지 않았으면 1초 대기
        if (!all_params_found) {
            ros::Duration(1.0).sleep();
        }
    }

    std::cout << "All parameters found!" << std::endl;
}

SHhandController::SHhandController(ros::NodeHandle nh_){
    std::cout << "initiate sh hand controller" << std::endl;
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // ROS 파라미터 서버에서 캘리브레이션 데이터를 가져옴

    aa_cal_pos = Eigen::MatrixXf(4,3);
    fe_cal_pos = Eigen::MatrixXf(4,3);

    // setting for pre_configuration
    if (!nh_.getParam("/dyros_glove/calibration/right/plate", pla_pos)) {
        ROS_ERROR("Failed to get calibration data for plate");
    }
    if (!nh_.getParam("/dyros_glove/calibration/right/pinch", pin_pos)) {
        ROS_ERROR("Failed to get calibration data for pinch");
    }
    if (!nh_.getParam("/dyros_glove/calibration/right/finger_flexion", ffe_pos)) {
        ROS_ERROR("Failed to get calibration data for finger_flexion");
    }
    if (!nh_.getParam("/dyros_glove/calibration/right/thumb_flexion", tfe_pos)) {
        ROS_ERROR("Failed to get calibration data for thumb_flexion");
    }
    if (!nh_.getParam("/dyros_glove/calibration/right/sphere", sph_pos)) {
        ROS_ERROR("Failed to get calibration data for sphere");
    }

    calculateCalibrationData(0);      // Calibration data를 기반으로 aa_cal_pos와 fe_cal_pos 계산

    
    ///////////////////////////////////////////////////////////// left 
    left_aa_cal_pos = Eigen::MatrixXf(4,3);
    left_fe_cal_pos = Eigen::MatrixXf(4,3);

    if (!nh_.getParam("/dyros_glove/calibration/left/plate", left_pla_pos)) {
        ROS_ERROR("Failed to get calibration data for plate");
    }
    if (!nh_.getParam("/dyros_glove/calibration/left/pinch", left_pin_pos)) {
        ROS_ERROR("Failed to get calibration data for pinch");
    }
    if (!nh_.getParam("/dyros_glove/calibration/left/finger_flexion", left_ffe_pos)) {
        ROS_ERROR("Failed to get calibration data for finger_flexion");
    }
    if (!nh_.getParam("/dyros_glove/calibration/left/thumb_flexion", left_tfe_pos)) {
        ROS_ERROR("Failed to get calibration data for thumb_flexion");
    }
    if (!nh_.getParam("/dyros_glove/calibration/left/sphere", left_sph_pos)) {
        ROS_ERROR("Failed to get calibration data for sphere");
    }

    calculateCalibrationData(1);
    /////////////////////////////////////////////////////////////////////////////////////////////////

    // sleep(30);


    // hand_state_pub = nh_.advertise<sensor_msgs::JointState>("/tocabi/handstates", 1);
    hand_open_sub = nh_.subscribe<std_msgs::Int32>("/mujoco_ros_interface/hand_open", 1, &SHhandController::hand_open_callback, this);
    sensor_glove_sub = nh_.subscribe<sensor_msgs::JointState>("/senseglove/0/rh/joint_states",1,&SHhandController::hand_callback, this);
    sensor_glove_sub_left = nh_.subscribe<sensor_msgs::JointState>("/senseglove/0/lh/joint_states",1,&SHhandController::left_hand_callback, this);
    std::cout << "sh hand controller finish" << std::endl;

}

void *SHhandController::PubThread(){        // 16번 루프마다 hand state publish, 실제로는 주석처리 되어 사용 x
    int pub_count = 0;
    while (true){
        pub_count++;
        if (pub_count % 16 == 0){
            // hand_state_msgs.header.stamp = ros::Time::now();
            // for(int i = 0; i < HAND_DOF; i++){
            //     hand_state_msgs.position[i] = shm_msgs_->hand_pos[i];
            //     hand_state_msgs.velocity[i] = shm_msgs_->hand_vel[i];
            //     hand_state_msgs.effort[i] = shm_msgs_->hand_acc[i];
            // }
            // hand_state_pub.publish(hand_state_msgs);

            // left
            hand_state_msgs.header.stamp = ros::Time::now();
            for(int i = 0; i < HAND_DOF; i++){
                hand_state_msgs.position[i] = shm_msgs_->left_hand_pos[i];
                hand_state_msgs.velocity[i] = shm_msgs_->left_hand_vel[i];
                hand_state_msgs.effort[i] = shm_msgs_->left_hand_acc[i];
            }
            hand_state_pub.publish(hand_state_msgs);
        }
    }
}

// 손가락의 굽힘 정도 계산 
void SHhandController::oneFingerFlexionCalculation(float distance, float flex[]) {
    // Constants
    const float l_p = 0.028;    // 28mm
    const float h = 0.0025;     
    const float l_1 = 0.01686;  //16.86mm
    const float l_2 = 0.02638;
    const float l_3 = 0.00638;
    const float l_4 = 0.03500;
    const float l_5 = 0.04000;
    const float l_6 = 0.00550;
    const float l_7 = 0.01000;
    const float l_8 = 0.03252;
    const float l_9 = 0.03420;
    const float l_10 = 0.01356;

    // Angles in radians
    const float alpha_prime = deg2rad(10.9); // 10.9 radian -> degree 
    const float gamma_0 = deg2rad(37.8);
    const float zeta_prime = deg2rad(31.9);
    const float kappa_0 = deg2rad(24.1);

    // Length of the sliding screw
    float d = distance;
    // if(d>=0.0432){
    //     d = 0.0431;
    // }
    // if(d<=0.0257){
    //     d=0.0256;
    // }
    float s_1 = std::sqrt(std::pow(d, 2) + std::pow(h, 2)); //0
    float alpha = std::acos((std::pow(l_1, 2) + std::pow(l_2, 2) - std::pow(s_1, 2)) / (2 * l_1 * l_2)); //0
    float beta = alpha + alpha_prime;
    float s_2 = std::sqrt(std::pow(l_3, 2) + std::pow(l_4, 2) - 2 * l_3 * l_4 * std::cos(beta));
    float gamma = std::acos((std::pow(l_5, 2) + std::pow(l_6, 2) - std::pow(s_2, 2)) / (2 * l_5 * l_6));
    // if((std::pow(l_5, 2) + std::pow(l_6, 2) - std::pow(s_2, 2)) / (2 * l_5 * l_6)<-1){
    //     gamma = std::acos(-1);
    // }
    // if((std::pow(l_5, 2) + std::pow(l_6, 2) - std::pow(s_2, 2)) / (2 * l_5 * l_6)>1){
    //     gamma = std::acos(1);
    // }
    float zeta = std::acos((std::pow(l_4, 2) + std::pow(s_2, 2) - std::pow(l_3, 2)) / (2 * l_4 * s_2))     
                    - std::acos((std::pow(l_6, 2) + std::pow(s_2, 2) - std::pow(l_5, 2)) / (2 * l_6 * s_2));

    // float x1 = (std::pow(l_4, 2) + std::pow(s_2, 2) - std::pow(l_3, 2)) / (2 * l_4 * s_2);
    // float x2 = (std::pow(l_6, 2) + std::pow(s_2, 2) - std::pow(l_5, 2)) / (2 * l_6 * s_2);
    // if(x1 < -1.0f){
    //     x1 = -1.0f;
    // }
    // if(x1 > 1.0f){
    //     x1=1.0f;
    // }
    // if(x2 < -1.0f){
    //     x2 = -1.0f;
    // }
    // if(x2 > 1.0f){
    //     x2=1.0f;
    // }
    // zeta = std::acos(x1) -std::acos(x2);

    float l_hk1 = std::sqrt(std::pow(l_6, 2) + std::pow(l_4, 2) - 2 * l_6 * l_4 * std::cos(zeta));
    float theta_hk1 = std::acos((std::pow(l_5, 2) + std::pow(l_3, 2) - std::pow(l_hk1, 2)) / (2 * l_5 * l_3));
    float eta = zeta + zeta_prime;
    float s_3 = std::sqrt(std::pow(l_7, 2) + std::pow(l_8, 2) - 2 * l_7 * l_8 * std::cos(eta));
    float kappa = std::acos((std::pow(l_10, 2) + std::pow(s_3, 2) - std::pow(l_9, 2)) / (2 * l_10 * s_3));
    float s_4 = std::sqrt(std::pow(l_7, 2) + std::pow(l_6, 2) - 2 * l_7 * l_6 * std::cos(eta));
    float kappa_2 = std::acos((std::pow(s_3, 2) + std::pow(l_p, 2) - std::pow(s_4, 2)) / (2 * s_3 * l_p));
    // std::cout<< "s1: " << s_1 << " " << "s2: " << s_2 <<" " << "s3: " << s_3 <<" "<< "s4: " <<s_4 << " " << "alpha: " <<alpha << " " << "beta: " <<beta << " " << "zeta: " <<zeta << " " << "eta: " <<eta << " "  <<std::endl; 
    // std::cout<< "theta_hk1: " << theta_hk1 << " " << "gamma: " << gamma <<" " << "gamma_0: " << gamma_0 <<" "<< "kappa: " <<kappa << " " << "kappa_0: " <<kappa_0 << " " << "kappa_2: " <<kappa_2 << " " <<std::endl; 
    //s3, s4, zeta, eta: nan 
    //theta_hk1, gamma, kappa, kappa_2: nan
    float theta_mcp = deg2rad(90) - (theta_hk1 - deg2rad(25)); 
    float theta_pip = gamma - gamma_0;
    float theta_dip = kappa + kappa_2 - kappa_0;

    // Heuristic compensation
    theta_mcp -= deg2rad(30);
    theta_pip += deg2rad(15);
    theta_dip -= deg2rad(50);

    flex[0] = theta_mcp;
    flex[1] = theta_pip;
    flex[2] = theta_dip;
    flex[3] = 0.8 * theta_mcp;
}

void SHhandController::kinematicsCalculation_RHand(float actuator_values[], float joint_values[]) {
    float DA11 = actuator_values[0];    // 검지 aa 동작 제어
    float DA12 = actuator_values[1];    // 검지 fe 동작 제어  length of the sliding screw 
    float DA21 = actuator_values[2];    // 엄지 aa 동작 제어
    float DA22 = actuator_values[3];    // 엄지 fe 동작 제어
    float DA31 = actuator_values[4];    // 중지
    float DA32 = actuator_values[5];    // 중지
    float DA41 = actuator_values[6];    // 약지
    float DA42 = actuator_values[7];    // 약지

    // 검지 
    float finger_1_flex[4]; // 4차원 배열 
    oneFingerFlexionCalculation(DA12, finger_1_flex); // DA12 값을 바탕으로 MCP,PIP,DIP 각도 계산 
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_1_flex[i])) {
            // std::cout<<"finger_1"<<std::endl;
            finger_1_flex[i] = 0; // 기본값 설정
        }
    }
    joint_values[0] = DA11; //AA 검지
    std::copy(finger_1_flex, finger_1_flex+3, joint_values+1);  //finger_1_flex 배열의 첫 세 요소를 joint_values 배열의 두 번째 요소 부터 복사   
    //joint_value[0]: DA11, joint_value[1]: MCP, joint_value[2]: PIP, joint_value[3]: DIP

    // 엄지
    float finger_2_flex[4];
    oneFingerFlexionCalculation(DA22, finger_2_flex);
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_2_flex[i])) {
            // std::cout<<"finger_2"<<std::endl;
            finger_2_flex[i] = 0; // 기본값 설정
        }
    }
    joint_values[4] = DA21;
    std::copy(finger_2_flex, finger_2_flex+3, joint_values+5);

    //중지
    float finger_3_flex[4];
    oneFingerFlexionCalculation(DA32, finger_3_flex);
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_3_flex[i])) {
            // std::cout<<"finger_3"<<std::endl;
            finger_3_flex[i] = 0; // 기본값 설정
        }
    }
    joint_values[8] = DA31;
    std::copy(finger_3_flex, finger_3_flex+3, joint_values+9);

    // 약지
    float finger_4_flex[4];
    oneFingerFlexionCalculation(DA42, finger_4_flex);
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_4_flex[i])) {
            // std::cout<<"finger_4"<<std::endl;
            finger_4_flex[i] = 0; // 기본값 설정
        }
    }
    joint_values[12] = DA41;
    std::copy(finger_4_flex, finger_4_flex+3, joint_values+13);

    // 각 손가락에 대해 oneFingerFlextionCalculation 함수에서 계산된 네번째 값을 joint_values 배열의 16번째 요소부터 복사 
    // 해당 네번째 값 = MCP*0.8,  
    float finger_act[] = {finger_1_flex[3], finger_2_flex[3], finger_3_flex[3], finger_4_flex[3]};
    std::copy(finger_act, finger_act+4, joint_values+16);
}

void SHhandController::kinematicsCalculation_LHand(float actuator_values[], float joint_values[]) {
    float DA11 = actuator_values[0];    // 첫번째 손가락 aa 동작 제어
    float DA12 = actuator_values[1];    // 첫번째 손가락 fe 동작 제어  length of the sliding screw 
    float DA21 = actuator_values[2];    // 두번째 손가락 aa 동작 제어
    float DA22 = actuator_values[3];    // 두번째 손가락 fe 동작 제어
    float DA31 = actuator_values[4];    // .. 
    float DA32 = actuator_values[5];
    float DA41 = actuator_values[6];
    float DA42 = actuator_values[7];

    // 첫번째 손가락 굽힘 동작 계산 
    float finger_1_flex[4]; // 4차원 배열 
    oneFingerFlexionCalculation(DA12, finger_1_flex); // DA12 값을 바탕으로 MCP,PIP,DIP 각도 계산 
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_1_flex[i])) {
            // std::cout<<"finger_1"<<std::endl;
            finger_1_flex[i] = 0; // 기본값 설정
        }
    }
    joint_values[0] = DA11; //AA 
    std::copy(finger_1_flex, finger_1_flex+3, joint_values+1);  //finger_1_flex 배열의 첫 세 요소를 joint_values 배열의 두 번째 요소 부터 복사   
    //joint_value[0]: DA11, joint_value[1]: MCP, joint_value[2]: PIP, joint_value[3]: DIP

    float finger_2_flex[4];
    oneFingerFlexionCalculation(DA22, finger_2_flex);
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_2_flex[i])) {
        //    std::cout<< "finger_2" <<std::endl;
            finger_2_flex[i] = 0.1; // 기본값 설정
        }
    }
    joint_values[4] = DA21;
    std::copy(finger_2_flex, finger_2_flex+3, joint_values+5);

    float finger_3_flex[4];
    oneFingerFlexionCalculation(DA32, finger_3_flex);
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_3_flex[i])) {
            // std::cout<<"finger_3"<<std::endl;
            finger_3_flex[i] = 0.1; // 기본값 설정
        }
    }
    joint_values[8] = DA31;
    std::copy(finger_3_flex, finger_3_flex+3, joint_values+9);

    float finger_4_flex[4];
    oneFingerFlexionCalculation(DA42, finger_4_flex);
    for (int i = 0; i < 4; i++) {
        if (std::isnan(finger_4_flex[i])) {
        //    std::cout<< "finger_4"<<std::endl;
            // finger_4_flex[i] = 0; // 기본값 설정
            finger_4_flex[i] = -0.1; // 기본값 설정
        }
    }
    joint_values[12] = DA41;
    std::copy(finger_4_flex, finger_4_flex+3, joint_values+13);

    // 각 손가락에 대해 oneFingerFlextionCalculation 함수에서 계산된 네번째 값을 joint_values 배열의 16번째 요소부터 복사 
    // 해당 네번째 값 = MCP*0.8,  
    float finger_act[] = {finger_1_flex[3], finger_2_flex[3], finger_3_flex[3], finger_4_flex[3]};
    std::copy(finger_act, finger_act+4, joint_values+16);
}


void SHhandController::hand_open_callback(const std_msgs::Int32ConstPtr &msg){
    std::cout << (msg->data ? "close" : "open") << std::endl;

    float sin_d = 0.027 + 0.015*msg->data; // 손가락의 굽힘 정도 (msg->data가 0이면 손을 펴는 동작, msg->data가 1이면 손가락을 굽히는 동작) / 1: sin_d = 0.037, 0: sin_d = 0.027
    float aa_value = 0.3*msg->data;     // 손가람의 벌림, 모음 정도 / 1: aa = 0.3,  0: aa = 0 
    // float sin_d_left = 0.024 + 0.011*msg->data;
    std::cout << sin_d << " " << aa_value << std::endl;
    

    // float Actuator_values[] = {aa_value, sin_d, aa_value*2, sin_d, 0.0, sin_d, -aa_value*2, sin_d}; // {2nd,1st,3th,4th}
    float Actuator_values[] = {aa_value, sin_d, aa_value, sin_d, 0.0, sin_d, -aa_value, sin_d}; // {2nd,1st,3th,4th}
    // float left_Actuator_values[] = {-aa_value*2, sin_d, aa_value*2, sin_d, 0.0, sin_d, -aa_value*2, sin_d}; // {첫번째 손가락 aa 동작 제어, 첫번째 손가락 FE 제어, 두번째 손가락 aa 제어, ...}
    float left_Actuator_values[] = {-aa_value, sin_d, -aa_value*3, sin_d, 0.0, sin_d, aa_value, sin_d}; // {2nd, 1st,3th,4th}
    
    // float Actuator_values[] = {aa_value*2, sin_d, aa_value*2, sin_d, 0.0, sin_d, -aa_value*2, sin_d}; // {2nd,1st,3th,4th}
    // float left_Actuator_values[] = {-aa_value*2, sin_d, -aa_value*2, sin_d, 0.0, sin_d, aa_value*2, sin_d}; // {2nd, 1st,3th,4th}
    float hand_command[HAND_DOF]; // 최종적으로 계산된 손가락 joint position command 저장 배열 
    float left_hand_command[HAND_DOF];
    kinematicsCalculation_RHand(Actuator_values, hand_command); // Actuator_value를 입력으로 받아, 최종 joint position command(hand_command) 계산 
    kinematicsCalculation_LHand(left_Actuator_values, left_hand_command);


    // print_array(Actuator_values, 8);
    // print_array(hand_command, HAND_DOF);
    // print_array(left_Actuator_values, 8);
    // print_array(left_hand_command, HAND_DOF);
    // std::copy(hand_command, hand_command + HAND_DOF, shm_msgs_->handCommand);

    float hand_init[HAND_DOF];
    
    std::copy(shm_msgs_->handCommand, shm_msgs_->handCommand + HAND_DOF, hand_init);
    std::copy(shm_msgs_->left_handCommand, shm_msgs_->left_handCommand + HAND_DOF, hand_init);
    ros::Time init = ros::Time::now();
    double t;
    do{
        t = (ros::Time::now()-init).toSec();
        for(int i = 0; i < HAND_DOF; i++){
            shm_msgs_->handCommand[i] = DyrosMath::cubic(t, 0.0, 0.5, hand_init[i], hand_command[i], 0.0, 0.0);
            shm_msgs_->left_handCommand[i] = DyrosMath::cubic(t, 0.0, 0.5, hand_init[i], left_hand_command[i], 0.0, 0.0);
        }
    }
    while(t < 0.5);
}

// 0.027(open,) < sin_d < 0.037(close),   0(open) < aa < 0.3 (close) 
void SHhandController::hand_callback(const sensor_msgs::JointStateConstPtr &msg){

    // ros::Time start_time = ros::Time::now();
    // std::cout << "right_hand_callback_check" <<std::endl;
    double aa1 = msg->position[16]; //엄지
    double aa2 = msg->position[0];  //검지
    double aa3 = msg->position[4];  //중지
    double aa4 = msg->position[12]; //약지
    double fe1 = msg->position[18]; //엄지
    double fe2 = msg->position[2];  //검지
    double fe3 = msg->position[6];  //중지
    double fe4 = msg->position[14]; //약지

    // double ps_fe1 = 0.027;
    // double ps_fe2 = 0.037;
    // double ps_aa1 = 0.0;
    // double ps_aa2 = 0.3;
    double ps_fe1 = 0.027;
    double ps_fe2 = 0.042;
    double ps_aa1 = 0.0;
    double ps_aa2 = 0.3;

    float aa1_value = ((ps_aa2 - ps_aa1)/(aa_cal_pos(0,2)-aa_cal_pos(0,0)))*(aa1 - aa_cal_pos(0,0));
    float aa2_value = ((ps_aa2 - ps_aa1)/(aa_cal_pos(1,2)-aa_cal_pos(1,0)))*(aa2 - aa_cal_pos(1,0));
    float aa3_value = ((ps_aa2 - ps_aa1)/(aa_cal_pos(2,2)-aa_cal_pos(2,0)))*(aa3 - aa_cal_pos(2,0));
    float aa4_value = ((ps_aa2 - ps_aa1)/(aa_cal_pos(3,2)-aa_cal_pos(3,0)))*(aa4 - aa_cal_pos(3,0));
    float fe1_value = ps_fe1 + ((ps_fe2 - ps_fe1)/(fe_cal_pos(0,2)-fe_cal_pos(0,0)))*(fe1 - fe_cal_pos(0,0));
    float fe2_value = ps_fe1 +((ps_fe2 - ps_fe1)/(fe_cal_pos(1,2)-fe_cal_pos(1,0)))*(fe2 - fe_cal_pos(1,0));
    float fe3_value = ps_fe1 +((ps_fe2 - ps_fe1)/(fe_cal_pos(2,2)-fe_cal_pos(2,0)))*(fe3 - fe_cal_pos(2,0));
    float fe4_value = ps_fe1 +((ps_fe2 - ps_fe1)/(fe_cal_pos(3,2)-fe_cal_pos(3,0)))*(fe4 - fe_cal_pos(3,0));
    
    if (aa1_value < 0) aa1_value = 0.01;
    if (aa2_value < 0) aa2_value = 0.01;
    if (aa3_value < 0) aa3_value = 0.01;
    if (aa4_value < 0) aa4_value = 0.01;
    if (fe1_value < 0) fe1_value = ps_fe1;
    if (fe2_value < 0) fe2_value = ps_fe1;
    if (fe3_value < 0) fe3_value = ps_fe1;
    if (fe4_value < 0) fe4_value = ps_fe1;

    
    // float Actuator_values[] = {aa1_value*2, fe1_value, aa2_value*2, fe2_value, 0.0, fe3_value, -aa4_value*2, fe4_value}; 
    float Actuator_values[] = {aa2_value, fe2_value, aa1_value, fe1_value, 0.0, fe3_value, -aa4_value, fe4_value}; // {검지, 엄지, 중지, 약지}
    float hand_command[HAND_DOF]; // 최종적으로 계산된 손가락 joint position command 저장 배열 
    // print_array(Actuator_values, 8);
    kinematicsCalculation_RHand(Actuator_values, hand_command); // Actuator_value를 입력으로 받아, 최종 joint position command(hand_command) 계산 
    // std::cout<<"right"<<std::endl;
    // print_array(hand_command, HAND_DOF);
    std::copy(hand_command, hand_command + HAND_DOF, shm_msgs_->handCommand);

    //// left example version
    /////////////////////////////////////////////////////////
    // float left_aa1_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(0,2)-left_aa_cal_pos(0,0)))*(aa1 - left_aa_cal_pos(0,0));
    // float left_aa2_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(1,2)-left_aa_cal_pos(1,0)))*(aa2 - left_aa_cal_pos(1,0));
    // float left_aa3_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(2,2)-left_aa_cal_pos(2,0)))*(aa3 - left_aa_cal_pos(2,0));
    // float left_aa4_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(3,2)-left_aa_cal_pos(3,0)))*(aa4 - left_aa_cal_pos(3,0));
    // float left_fe1_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(0,2)-left_fe_cal_pos(0,0)))*(fe1 - left_fe_cal_pos(0,0));
    // float left_fe2_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(1,2)-left_fe_cal_pos(1,0)))*(fe2 - left_fe_cal_pos(1,0));
    // float left_fe3_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(2,2)-left_fe_cal_pos(2,0)))*(fe3 - left_fe_cal_pos(2,0));
    // float left_fe4_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(3,2)-left_fe_cal_pos(3,0)))*(fe4 - left_fe_cal_pos(3,0));
    
    // if (left_aa1_value < 0) left_aa1_value = 0.01;
    // if (left_aa2_value < 0) left_aa2_value = 0.01;
    // if (left_aa3_value < 0) left_aa3_value = 0.01;
    // if (left_aa4_value < 0) left_aa4_value = 0.01;
    // if (left_fe1_value < 0) left_fe1_value = 0.01;
    // if (left_fe2_value < 0) left_fe2_value = 0.01;
    // if (left_fe3_value < 0) left_fe3_value = 0.01;
    // if (left_fe4_value < 0) left_fe4_value = 0.01;

    // float left_Actuator_values[] = {left_aa1_value*2, left_fe1_value, -left_aa2_value*2, left_fe2_value, 0.0, left_fe3_value, -left_aa4_value*2, left_fe4_value};

    // float left_hand_command[HAND_DOF];
    // kinematicsCalculation_LHand(left_Actuator_values, left_hand_command);
    // // std::cout<<"left"<<std::endl;
    // // print_array(left_hand_command, HAND_DOF);
    // std::copy(left_hand_command, left_hand_command+HAND_DOF, shm_msgs_->left_handCommand);

    //////////////////////////////////////////////////////////////////////////////
}

void SHhandController::left_hand_callback(const sensor_msgs::JointStateConstPtr &msg){
    // std::cout << "left_hand_callback_check" <<std::endl;
    double aa1 = msg->position[16];//엄지
    double aa2 = msg->position[0];//검지
    double aa3 = msg->position[4];//중지
    double aa4 = msg->position[12];//약지
    double fe1 = msg->position[18];//엄지
    double fe2 = msg->position[2];//검지
    double fe3 = msg->position[6];//중지
    double fe4 = msg->position[14];//약지

    double ps_fe1 = 0.027;
    double ps_fe2 = 0.042;
    double ps_aa1 = 0.0;
    double ps_aa2 = 0.3;

    float aa1_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(0,2)-left_aa_cal_pos(0,0)))*(aa1 - left_aa_cal_pos(0,0)); //엄지
    float aa2_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(1,2)-left_aa_cal_pos(1,0)))*(aa2 - left_aa_cal_pos(1,0)); //검지
    float aa3_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(2,2)-left_aa_cal_pos(2,0)))*(aa3 - left_aa_cal_pos(2,0)); //중지
    float aa4_value = ((ps_aa2 - ps_aa1)/(left_aa_cal_pos(3,2)-left_aa_cal_pos(3,0)))*(aa4 - left_aa_cal_pos(3,0)); //약지
    float fe1_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(0,2)-left_fe_cal_pos(0,0)))*(fe1 - left_fe_cal_pos(0,0)); //엄지
    float fe2_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(1,2)-left_fe_cal_pos(1,0)))*(fe2 - left_fe_cal_pos(1,0)); //검지
    float fe3_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(2,2)-left_fe_cal_pos(2,0)))*(fe3 - left_fe_cal_pos(2,0)); //중지
    float fe4_value = ps_fe1 +((ps_fe2 - ps_fe1)/(left_fe_cal_pos(3,2)-left_fe_cal_pos(3,0)))*(fe4 - left_fe_cal_pos(3,0)); //약지
    
    if (aa1_value < 0) aa1_value = 0.01;
    if (aa2_value < 0) aa2_value = 0.01;
    if (aa3_value < 0) aa3_value = 0.01;
    if (aa4_value < 0) aa4_value = 0.01;
    if (fe1_value < 0) fe1_value = ps_fe1;
    if (fe2_value < 0) fe2_value = ps_fe1;
    if (fe3_value < 0) fe3_value = ps_fe1;
    if (fe4_value < 0) fe4_value = ps_fe1;
    
    // float left_Actuator_values[] = {-aa1_value*2, fe1_value, -aa2_value*2, fe2_value, 0.0, fe3_value, aa4_value*2, fe4_value};
    float left_Actuator_values[] = {-aa2_value, fe2_value,-aa1_value*3, fe1_value, 0.0, fe3_value, aa4_value, fe4_value}; // {검지, 엄지, 중지, 약지}


    // no left_hand controller 
    float left_hand_command[HAND_DOF];
    // print_array(left_Actuator_values, 8);
    kinematicsCalculation_LHand(left_Actuator_values, left_hand_command);
    std::copy(left_hand_command, left_hand_command+HAND_DOF, shm_msgs_->left_handCommand);
}


void SHhandController::calculateCalibrationData(float hand) {
    int AA_DIRECTION = 1;
    if (hand == 0){ 
        std::cout << "calculate Calibration Data right" << std::endl;
   
        AA_DIRECTION = 1;
        aa_cal_pos(0, 0) = AA_DIRECTION * pla_pos[0];
        aa_cal_pos(0, 1) = 0;
        aa_cal_pos(0, 2) = AA_DIRECTION * pin_pos[0];

        aa_cal_pos(1, 0) = AA_DIRECTION * pla_pos[1];
        aa_cal_pos(1, 1) = 0;
        aa_cal_pos(1, 2) = AA_DIRECTION * sph_pos[1];

        aa_cal_pos(2, 0) = AA_DIRECTION * pla_pos[2];
        aa_cal_pos(2, 1) = 0;
        aa_cal_pos(2, 2) = AA_DIRECTION * sph_pos[2];

        aa_cal_pos(3, 0) = AA_DIRECTION * pla_pos[3];
        aa_cal_pos(3, 1) = 0;
        aa_cal_pos(3, 2) = AA_DIRECTION * sph_pos[3];

        fe_cal_pos(0, 0) = pla_pos[4];
        fe_cal_pos(0, 1) = pin_pos[4];
        fe_cal_pos(0, 2) = tfe_pos[4];

        fe_cal_pos(1, 0) = pla_pos[5];
        fe_cal_pos(1, 1) = pin_pos[5];
        fe_cal_pos(1, 2) = ffe_pos[5];

        fe_cal_pos(2, 0) = pla_pos[6];
        fe_cal_pos(2, 1) = pin_pos[6];
        fe_cal_pos(2, 2) = ffe_pos[6];

        fe_cal_pos(3, 0) = pla_pos[7];
        fe_cal_pos(3, 1) = pin_pos[7];
        fe_cal_pos(3, 2) = ffe_pos[7];
    }
    else{
        std::cout << "calculate Calibration Data left" << std::endl;
        AA_DIRECTION = 1;
        left_aa_cal_pos(0, 0) = AA_DIRECTION * left_pla_pos[0]; //plate 시 엄지 AA
        left_aa_cal_pos(0, 1) = 0;
        left_aa_cal_pos(0, 2) = AA_DIRECTION * left_pin_pos[0]; //pinch 시 엄지 AA

        left_aa_cal_pos(1, 0) = AA_DIRECTION * left_pla_pos[1]; //plate 시 검지 AA
        left_aa_cal_pos(1, 1) = 0;
        left_aa_cal_pos(1, 2) = AA_DIRECTION * left_sph_pos[1]; //sphere 시 검지 AA

        left_aa_cal_pos(2, 0) = AA_DIRECTION * left_pla_pos[2]; //plate 시 중지 AA
        left_aa_cal_pos(2, 1) = 0;
        left_aa_cal_pos(2, 2) = AA_DIRECTION * left_sph_pos[2]; //sphere 시 중지 AA

        left_aa_cal_pos(3, 0) = AA_DIRECTION * left_pla_pos[3]; //plate 시 약지 AA
        left_aa_cal_pos(3, 1) = 0;
        left_aa_cal_pos(3, 2) = AA_DIRECTION * left_sph_pos[3]; //sphere 시 약지 AA

        left_fe_cal_pos(0, 0) = left_pla_pos[4];    // plate 시 엄지 FE
        left_fe_cal_pos(0, 1) = left_pin_pos[4];    // pinch 시 엄지 FE
        left_fe_cal_pos(0, 2) = left_tfe_pos[4];    // thumb flexion 시 엄지 FE

        left_fe_cal_pos(1, 0) = left_pla_pos[5];    // plate 시 검지 FE
        left_fe_cal_pos(1, 1) = left_pin_pos[5];    // pinch 시 검지 FE
        left_fe_cal_pos(1, 2) = left_ffe_pos[5];    // finger flexion 시 검지 FE

        left_fe_cal_pos(2, 0) = left_pla_pos[6];    // plate 시 중지 FE
        left_fe_cal_pos(2, 1) = left_pin_pos[6];    // pinch 시 중지 FE
        left_fe_cal_pos(2, 2) = left_ffe_pos[6];    // finger flexion 시 중지 FE

        left_fe_cal_pos(3, 0) = left_pla_pos[7];    // plate 시 약지 FE
        left_fe_cal_pos(3, 1) = left_pin_pos[7];    // pinch 시 약지 FE
        left_fe_cal_pos(3, 2) = left_ffe_pos[7];    // finger flexion 시 약지 FE
    }
}


int main(int argc, char **argv)
{
    // :: ROS CUSTUM :: initialize ros
    ros::init(argc, argv, "shhand_controller");
    ros::NodeHandle nh("~");

    /////////////////////////////////////////////////////
    // parameter setting example
    // 16 0 4 12 18 2 6 14

    // // right hand 
    // std::vector<double> plate_array = {620, 566, 508, 520,361, 692,631, 695};
    // nh.setParam("/dyros_glove/calibration/right/plate", plate_array);
    // std::vector<double> pinch_array = {795,0,0,0,460,850,890,920};
    // nh.setParam("/dyros_glove/calibration/right/pinch", pinch_array);
    // std::vector<double> finger_array = {0,0,0,0,0,1019,1157,1136};
    // nh.setParam("/dyros_glove/calibration/right/finger_flexion", finger_array);
    // std::vector<double> thumb_array = {0,0,0,0,518,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/thumb_flexion", thumb_array);
    // std::vector<double> sphere_array = {0,525,464,473,0,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/sphere", sphere_array);


    // // left
    // std::vector<double> left_plate_array = {620, 566, 508, 520,361, 692,631, 695};
    // nh.setParam("/dyros_glove/calibration/left/plate", left_plate_array);
    // std::vector<double> left_pinch_array = {795,0,0,0,460,850,890,920};
    // nh.setParam("/dyros_glove/calibration/left/pinch", left_pinch_array);
    // std::vector<double> left_finger_array = {0,0,0,0,0,1019,1157,1136};
    // nh.setParam("/dyros_glove/calibration/left/finger_flexion", left_finger_array);
    // std::vector<double> left_thumb_array = {0,0,0,0,518,0,0,0};
    // nh.setParam("/dyros_glove/calibration/left/thumb_flexion", left_thumb_array);
    // std::vector<double> left_sphere_array = {0,525,464,473,0,0,0,0};
    // nh.setParam("/dyros_glove/calibration/left/sphere", left_sphere_array);

    // right hand ex2
    // std::vector<double> plate_array = {689, 543, 516, 528,875, 922,709, 800};
    // nh.setParam("/dyros_glove/calibration/right/plate", plate_array);
    // std::vector<double> pinch_array = {815,0,0,0,1208,982,965,1099};
    // nh.setParam("/dyros_glove/calibration/right/pinch", pinch_array);
    // std::vector<double> finger_array = {0,0,0,0,0,1266,1315,1317};
    // nh.setParam("/dyros_glove/calibration/right/finger_flexion", finger_array);
    // std::vector<double> thumb_array = {0,0,0,0,1287,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/thumb_flexion", thumb_array);
    // std::vector<double> sphere_array = {0,575,481,465,0,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/sphere", sphere_array);

    // right hand ex1
    // std::vector<double> plate_array = {675, 525, 480, 498,818, 767,611,872};
    // nh.setParam("/dyros_glove/calibration/right/plate", plate_array);
    // std::vector<double> pinch_array = {809,0,0,0,1270,994,1042,1128};
    // nh.setParam("/dyros_glove/calibration/right/pinch", pinch_array);
    // std::vector<double> finger_array = {0,0,0,0,0,1158,1393,1309};
    // nh.setParam("/dyros_glove/calibration/right/finger_flexion", finger_array);
    // std::vector<double> thumb_array = {0,0,0,0,1329,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/thumb_flexion", thumb_array);
    // std::vector<double> sphere_array = {0,559,475,438,0,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/sphere", sphere_array);


    // left
    // std::vector<double> left_plate_array = {349, 495, 508, 519,677, 741,682, 705};
    // nh.setParam("/dyros_glove/calibration/left/plate", left_plate_array);
    // std::vector<double> left_pinch_array = {182,0,0,0,952,1198,1234,1257};
    // nh.setParam("/dyros_glove/calibration/left/pinch", left_pinch_array);
    // std::vector<double> left_finger_array = {0,0,0,0,0,1352,1343,1325};
    // nh.setParam("/dyros_glove/calibration/left/finger_flexion", left_finger_array);
    // std::vector<double> left_thumb_array = {0,0,0,0,1081,0,0,0};
    // nh.setParam("/dyros_glove/calibration/left/thumb_flexion", left_thumb_array);
    // std::vector<double> left_sphere_array = {0,396,477,528,0,0,0,0};
    // nh.setParam("/dyros_glove/calibration/left/sphere", left_sphere_array);
    
    // ex4 right
    // std::vector<double> plate_array = {654, 541, 525, 538, 873, 820, 623, 677};
    // nh.setParam("/dyros_glove/calibration/right/plate", plate_array);
    // std::vector<double> pinch_array = {819, 546, 515, 535, 1316, 1086, 1035, 1164};
    // nh.setParam("/dyros_glove/calibration/right/pinch", pinch_array);
    // std::vector<double> finger_array = {0,0,0,0,0,1252,1439,1311};
    // nh.setParam("/dyros_glove/calibration/right/finger_flexion", finger_array);
    // std::vector<double> thumb_array = {0,0,0,0,1326,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/thumb_flexion", thumb_array);
    // std::vector<double> sphere_array = {0,639,500,433,0,0,0,0};
    // nh.setParam("/dyros_glove/calibration/right/sphere", sphere_array);

    // // ex4 left
    // std::vector<double> left_plate_array = {394, 433, 421, 458, 796, 723, 732, 786};
    // nh.setParam("/dyros_glove/calibration/left/plate", left_plate_array);
    // std::vector<double> left_pinch_array = {198,0,0,0,945, 1186, 1266, 1338};
    // nh.setParam("/dyros_glove/calibration/left/pinch", left_pinch_array);
    // std::vector<double> left_finger_array = {0,0,0,0,0,1320, 1324, 1353};
    // nh.setParam("/dyros_glove/calibration/left/finger_flexion", left_finger_array);
    // std::vector<double> left_thumb_array = {0,0,0,0,939,0,0,0};
    // nh.setParam("/dyros_glove/calibration/left/thumb_flexion", left_thumb_array);
    // std::vector<double> left_sphere_array = {0,322, 456, 544,0,0,0,0};
    // nh.setParam("/dyros_glove/calibration/left/sphere", left_sphere_array);

    // std::cout << "No Problem in setting parameter example " << std::endl;
    ///////////////////////////////////////
    SHhandController shhand_controller(nh);
    int shm_msg_id;
    std::cout << "init_shm in shhand_controller.cpp" <<std::endl;
    init_shm(shm_msg_key, shm_msg_id, &shhand_controller.shm_msgs_);

    ros::spin();

    deleteSharedMemory(shm_msg_id, shhand_controller.shm_msgs_);
}