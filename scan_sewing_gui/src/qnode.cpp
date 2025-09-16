/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date January 2022
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <iostream>
#include "../include/scan_sewing_gui/qnode.hpp"
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace scan_sewing_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    delete nh;
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"scan_sewing_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    init_nh();
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    start();
    return true;
}

void QNode::init_nh() {

    nh = new ros::NodeHandle("scan_sewing_gui");

    // Publisher
    pattern_former_state_pub_ = nh->advertise<std_msgs::Char>("pattern_former_state",10);
    vision_capture_num_pub_ = nh->advertise<std_msgs::Char>("vision_capture_num",10);
    vision_capture_pub_ = nh->advertise<std_msgs::Bool>("vision_capture",10);
    vision_exposure_time_pub_ = nh->advertise<std_msgs::Int32>("vision_exposure_time",10);
    vision_offset_pub_ = nh->advertise<std_msgs::Float64>("vision_offset",10);
    vision_image_processing_pub_ = nh->advertise<std_msgs::Empty>("vision_image_processing",10);
    vision_refresh_pub_ = nh->advertise<std_msgs::Empty>("vision_refresh",10);
    vision_calibration_pub_ = nh->advertise<std_msgs::Float64MultiArray>("vision_calibration",10);
    macro_mini_trigger_pub_ = nh->advertise<std_msgs::Bool>("scan_complete",10);
    macro_mini_topstitch_pub_ = nh->advertise<std_msgs::Bool>("stitch",10);
    led_state_pub_ = nh->advertise<std_msgs::Bool>("led_state",10);
    path_num_pub_ = nh->advertise<std_msgs::Int32>("path_num",10);
    ros_shutdown_pub_ = nh->advertise<std_msgs::Bool>("ros_shutdown",10);

    //Subscriber
    vision_image_sub_ = nh->subscribe("/vision_image",10, &QNode::visionImageCallback, this);
    vision_error_sub_ = nh->subscribe("/vision_error",10, &QNode::visionErrorCallback, this);
    pf_foot_sub_ = nh->subscribe("/scan_sewing_pf/pf_foot",10, &QNode::pfFootCallback, this);
    pf_frame_sub_ = nh->subscribe("/scan_sewing_pf/pf_frame",10, &QNode::pfFrameCallback, this);
    pf_loose_sub_ = nh->subscribe("/scan_sewing_pf/pf_loose",10, &QNode::pfLooseCallback, this);
    pf_reset_sub_ = nh->subscribe("/scan_sewing_pf/pf_reset",10, &QNode::pfResetCallback, this);
    pf_breakline_sub_ = nh->subscribe("/scan_sewing_pf/pf_breakline",10, &QNode::pfBreaklineCallback, this);
    mini_state_sub_ = nh->subscribe("/scan_sewing_mini/mini_state",10, &QNode::miniStateCallback, this);
    mini_needle_sub_ = nh->subscribe("/scan_sewing_mini/mini_needle",10, &QNode::miniNeedleCallback, this);
    mini_motor_pos_sub_ = nh->subscribe("/scan_sewing_mini/mini_motor_pos",10, &QNode::miniMotorPosCallback, this);
}

void QNode::run() {
    ros::Rate loop_rate(100);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }


    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::rosShutdownTrigger() {
    light_control(false);
    ros_shutdown_pub_.publish(true);
}

void QNode::send_transition(std::string state) {
    if(state=="stop_button") {
        std_msgs::Char msg;
        msg.data = 48;
        pattern_former_state_pub_.publish(msg);
    } else if(state=="press_button") {
        std_msgs::Char msg;
        msg.data = 49;
        pattern_former_state_pub_.publish(msg);
    } else if(state=="start_button") {
        std_msgs::Char msg;
        msg.data = 50;
        pattern_former_state_pub_.publish(msg);
    } else if(state=="light_button"){
        light_control(!light_status);
    } else if(state=="calibration_button") {
        std_msgs::Char msg;
        msg.data = 48;
        vision_capture_num_pub_.publish(msg);

        std_msgs::Bool msg2;
        msg2.data = true;
        vision_capture_pub_.publish(msg2);

        //light_control(true);
    } else if(state=="refresh_button") {
        light_control(false);
        image_num = '0';
        rt_image = true;

        std_msgs::Empty msg;
        vision_refresh_pub_.publish(msg);

        imageUpdated();
        // 초기화 해야할 변수 추가해야함
    } else if(state=="detection_button") {
        std_msgs::Char msg;
        msg.data = 49;
        vision_capture_num_pub_.publish(msg);

        light_control(true);

        std_msgs::Char msg2;
        msg2.data = 51;
        pattern_former_state_pub_.publish(msg2);

        std_msgs::Int32 msg3;
        msg3.data = template_shape_;
        path_num_pub_.publish(msg3);
    } else if (state=="topstitching_button") {
        // Start Pattern Former
        std_msgs::Char msg;
        msg.data = 52;
        pattern_former_state_pub_.publish(msg);
        Sleep(10);

        // Start Mini module, 다시 한번 체크필요
        std_msgs::Bool msg2;
        msg2.data = true;
        macro_mini_topstitch_pub_.publish(msg2);
    }
}

void QNode::exposure_time_send_transition(int exposure_time) {
    std_msgs::Int32 msg;
    msg.data = exposure_time;
    vision_exposure_time_pub_.publish(msg);
}

void QNode::offset_send_transition(double offset) {
    std_msgs::Float64 msg;
    msg.data = offset;
    vision_offset_pub_.publish(msg);
}

void QNode::template_shape_send_transition(int template_shape) {
    std_msgs::Int32 msg;
    msg.data = template_shape;
    path_num_pub_.publish(msg);
    template_shape_ = template_shape;
}

void QNode::calibration_send_transition(double pf_x, double pf_y, double temp_x, double temp_y) {
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(pf_x);
    msg.data.push_back(pf_y);
    msg.data.push_back(temp_x);
    msg.data.push_back(temp_y);
    vision_calibration_pub_.publish(msg);
}

void QNode::light_control(bool tr) {
    std_msgs::Bool msg;
    msg.data = tr;
    led_state_pub_.publish(msg);
    light_status = tr;
}

// Callback functions

// image_num 업데이트
void QNode::visionImageCallback(const std_msgs::CharConstPtr& msg) {
    if (msg->data!='0' || image_num=='0') {
        rt_image = false;
        image_num = msg->data;
        if(image_num=='1') {    // Calibration image 들어옴
            // light_control(false);
        } else if(image_num=='2') {     // 1st 캡처

        } else if(image_num=='3') {     // 2nd 캡처
            light_control(false);

            // Stop Pattern Former
            std_msgs::Char msg;
            msg.data = 48;
            pattern_former_state_pub_.publish(msg);

            // Tell mini node that scan finished
            std_msgs::Bool msg2;
            msg2.data = true;
            macro_mini_trigger_pub_.publish(msg2);

            // image processing
            std_msgs::Empty msg3;
            vision_image_processing_pub_.publish(msg3);

        } else if(image_num=='4') {

        }
    } else {
        rt_image = true;
    }
    imageUpdated();
}

void QNode::visionErrorCallback(const std_msgs::CharConstPtr& msg) {
    vision_error = msg->data;
    if(vision_error=='1') {
        light_control(false);
    }
    visionErrorUpdated();
}

void QNode::pfFootCallback(const std_msgs::BoolConstPtr& msg) {
    pf_foot = msg->data;
    pfStatusUpdated();
}

void QNode::pfFrameCallback(const std_msgs::BoolConstPtr& msg) {
    pf_frame = msg->data;
    pfStatusUpdated();
}

void QNode::pfLooseCallback(const std_msgs::BoolConstPtr& msg) {
    pf_loose = msg->data;
    pfStatusUpdated();
}

void QNode::pfResetCallback(const std_msgs::BoolConstPtr& msg) {
    pf_reset = msg->data;
    pfStatusUpdated();
}

void QNode::pfBreaklineCallback(const std_msgs::BoolConstPtr& msg) {
    pf_breakline = msg->data;
    pfStatusUpdated();
}
void QNode::miniStateCallback(const std_msgs::Int32ConstPtr& msg) {
    mini_state = msg->data;
    miniStateUpdated();
}
void QNode::miniNeedleCallback(const std_msgs::Int32ConstPtr& msg) {

    if (mini_path_length==0) {
        mini_path_length = msg->data;
    } else {
        mini_needle = msg->data;
        miniNeedleUpdated();
    }
}

void QNode::miniMotorPosCallback(const geometry_msgs::PointConstPtr& msg) {
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;
    if (z==0.0) {
        mini_motor_pos[0] = x;
        mini_motor_pos[1] = y;
    } else if (z==1.0) {
        mini_motor_pos[2] = x;
        mini_motor_pos[3] = y;
    } else if (z==2.0) {
        mini_motor_pos[4] = x;
        mini_motor_pos[5] = y;
    }
    miniMotorPosUpdated();
}

}  // namespace scan_sewing_gui
