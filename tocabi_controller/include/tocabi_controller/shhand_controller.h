#ifndef SHHAND_CONTROLLER_H
#define SHHAND_CONTROLLER_H

#include "shm_msgs.h"
#include "math_type_define.h"
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

class SHhandController {
public:
    SHhandController(ros::NodeHandle nh_);

    void *PubThread();
    static void *PubStarter(void *context) { return ((SHhandController *)context)->PubThread(); }

    void oneFingerFlexionCalculation(float distance, float flex[]);

    void kinematicsCalculation_RHand(float actuator_values[], float joint_values[]);
    void kinematicsCalculation_LHand(float actuator_values[], float joint_values[]);

    void hand_open_callback(const std_msgs::Int32ConstPtr &msg);

    float deg2rad(float degrees) { return degrees * M_PI / 180.0; }

    void hand_callback(const sensor_msgs::JointStateConstPtr &msg);
    void left_hand_callback(const sensor_msgs::JointStateConstPtr &msg);

    void calculateCalibrationData(float hand);

    ros::Publisher hand_state_pub;
    // ros::Publisher left_hand_state_pub;
    ros::Subscriber hand_open_sub;
    ros::Subscriber sensor_glove_sub;
    ros::Subscriber sensor_glove_sub_left;

    SHMmsgs *shm_msgs_;    
    sensor_msgs::JointState hand_state_msgs;
    // sensor_msgs::JointState left_hand_state_msgs;

    Eigen::MatrixXf aa_cal_pos;
    Eigen::MatrixXf fe_cal_pos;

    std::vector<float> pla_pos;
    std::vector<float> pin_pos;
    std::vector<float> ffe_pos;
    std::vector<float> tfe_pos;
    std::vector<float> sph_pos;


    Eigen::MatrixXf left_aa_cal_pos;
    Eigen::MatrixXf left_fe_cal_pos;

    std::vector<float> left_pla_pos;
    std::vector<float> left_pin_pos;
    std::vector<float> left_ffe_pos;
    std::vector<float> left_tfe_pos;
    std::vector<float> left_sph_pos;

    const std::string hand_joint_name[HAND_DOF] = {
        "aa2" , "mcp2", "pip2", "dip2",
        "aa1" , "mcp1", "pip1", "dip1",
        "aa3" , "mcp3", "pip3", "dip3",
        "aa4" , "mcp4", "pip4", "dip4",
        "act1", "act2", "act3", "act4"};
    const std::string left_hand_joint_name[HAND_DOF] = {
        "left_aa2" , "left_mcp2", "left_pip2", "left_dip2",
        "left_aa1" , "left_mcp1", "left_pip1", "left_dip1",
        "left_aa3" , "left_mcp3", "left_pip3", "left_dip3",
        "left_aa4" , "left_mcp4", "left_pip4", "left_dip4",
        "left_act1", "left_act2", "left_act3", "left_act4"};
};

#endif
