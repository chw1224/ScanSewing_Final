#pragma once

#include "atlstr.h"
#include <fstream>
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <winsock2.h>
#include <thread>


class SewingMachine
{
private:
    int init_argc;
    char** init_argv;
    ros::NodeHandle* nh;

    ros::Publisher connect_pub_;
    ros::Publisher motor_pos_pub_;
    // ros::Publisher motor_state_pub_;
    ros::Publisher is_moving_pub_;
    ros::Publisher pf_io_pub_;

    ros::Subscriber pf_state_sub_;
    ros::Subscriber pf_led_sub_;
    ros::Subscriber pf_foot_sub_;
    ros::Subscriber pf_frame_sub_;
    ros::Subscriber pf_control_sub_;
    ros::Subscriber pf_jog_vel_sub_;
    ros::Subscriber pf_jog_rpm_sub_;
    ros::Subscriber pf_stitch_length_sub_;
    ros::Subscriber pf_stitch_rpm_sub_;
    ros::Subscriber pf_jog_sub_;
    ros::Subscriber ros_shutdown_sub_;

    bool connected = false;
    bool motor_moving = false;
    double target_pos[2];
    double jog_vel = 30.0;
    int jog_rpm = 100;
    double stitch_length = 3.0;
    int stitch_rpm = 500;
    bool motor_position_out = false;
    int stop_count = 0;

public:
    SewingMachine(int argc, char** argv);
    virtual ~SewingMachine();
public:
    DWORD m_dwDevice;
    BOOL m_bDeviceopen;
    HINSTANCE m_hPmaclib;

    double m_dTa;
    double m_dTs;
    double m_dZdwelltime;
    double m_dScale[3];

    double m_dPosition[3];
    double m_dVelocity[3];
    int m_nXStatus[5];
    int m_nYStatus[5];
    int m_nZStatus[5];
    int m_nInput[32];
    int m_nOutput[32];

    // Callback
    void pfCallback(const std_msgs::Char::ConstPtr &msg);
    void pfLedCallback(const std_msgs::Bool::ConstPtr &msg);
    void pfFootCallback(const std_msgs::Bool::ConstPtr &msg);
    void pfFrameCallback(const std_msgs::Bool::ConstPtr &msg);
    void pfControlCallback(const geometry_msgs::Point::ConstPtr &msg);
    void pfJogVelCallback(const std_msgs::Float64::ConstPtr &msg);
    void pfJogRPMCallback(const std_msgs::Int32::ConstPtr &msg);
    void pfStitchLengthCallback(const std_msgs::Float64::ConstPtr &msg);
    void pfStitchRPMCallback(const std_msgs::Int32::ConstPtr &msg);
    void pfJogCallback(const std_msgs::Char::ConstPtr &msg);
    void rosShutdownCallback(const std_msgs::Bool::ConstPtr &msg);


    // Functions
    bool init();
    void init_nh();
    void run();
    bool PMACConnect();
    void CommandInput(string str);
    void PatternFormerStart();
    CString GetResponse(CString msg);
    void JogPlus(int motorNum, double velocity);
    void JogMinus(int motorNum, double velocity);
    void JogStop(int motorNum);
    void ZHome();
    void EMS();
    void MoveOrigin();

    void TopStitch();
    void MacroMini();

    // Loop Function
    bool isMotorStopped();
    void ThreadFunction();

};
