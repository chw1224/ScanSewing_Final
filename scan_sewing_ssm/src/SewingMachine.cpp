using namespace std;

#include "../include/scan_sewing_ssm/SewingMachine.h"
#include "../include/scan_sewing_ssm/runtime.h"
#include "../include/scan_sewing_ssm/atlstr.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <Windows.h>
#include <thread>
#include <vector>

// Define
#define XYSpeed "P10"
#define GoOrigin "P100"
#define OriginOffsetX "P102"
#define OriginOffsetY "P101"



SewingMachine::SewingMachine(int argc, char** argv) : init_argc(argc), init_argv(argv) {

    target_pos[0] = 0.0;
    target_pos[1] = 0.0;

    CString filepath;
    filepath = "C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_ssm\\ini\\SystemParameter.ini";
    char* cBuf = NULL;
    cBuf = (char*)malloc(sizeof(char) * 256);
    memset(cBuf, 0x00, sizeof(cBuf));
    GetPrivateProfileString("SYSTEM", "TA", "0", cBuf, 256, filepath);
    m_dTa = atof(cBuf);
    GetPrivateProfileString("SYSTEM", "TS", "0", cBuf, 256, filepath);
    m_dTs = atof(cBuf);
    GetPrivateProfileString("SYSTEM", "ZDWELLTIME", "0", cBuf, 256, filepath);
    m_dZdwelltime = atof(cBuf);
    GetPrivateProfileString("SYSTEM", "SCALE0", "0", cBuf, 256, filepath);
    m_dScale[0] = atof(cBuf);
    GetPrivateProfileString("SYSTEM", "SCALE1", "0", cBuf, 256, filepath);
    m_dScale[1] = atof(cBuf);
    GetPrivateProfileString("SYSTEM", "SCALE2", "0", cBuf, 256, filepath);
    m_dScale[2] = atof(cBuf);
}

SewingMachine::~SewingMachine() {
    delete nh;
}

bool SewingMachine::init() {
    ros::init(init_argc,init_argv,"scan_sewing_ssm");
    if(!ros::master::check()) {
        return false;
    }
    init_nh();
    ros::start();
    return true;
}

void SewingMachine::init_nh() {
    nh = new ros::NodeHandle("scan_sewing_ssm");

    connect_pub_ = nh->advertise<std_msgs::Bool>("pattern_former_connect",10);
    motor_pos_pub_ = nh->advertise<std_msgs::Float64MultiArray>("pattern_former_motor_pos",5);
    // motor_state_pub_ = nh->advertise<std_msgs::Int32MultiArray>("pattern_former_motor_state",5);
    is_moving_pub_ = nh->advertise<std_msgs::Empty>("pattern_former_is_moving",5);
    pf_io_pub_ = nh->advertise<std_msgs::Int32MultiArray>("pattern_former_io",5);

    pf_state_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_state", 10, &SewingMachine::pfCallback, this);
    pf_led_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_led",10, &SewingMachine::pfLedCallback, this);
    pf_foot_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_foot",10, &SewingMachine::pfFootCallback, this);
    pf_frame_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_frame",10, &SewingMachine::pfFrameCallback, this);
    pf_control_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_control",10, &SewingMachine::pfControlCallback, this);
    pf_jog_vel_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_jog_vel", 10, &SewingMachine::pfJogVelCallback, this);
    pf_jog_rpm_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_jog_rpm", 10, &SewingMachine::pfJogRPMCallback, this);
    pf_stitch_length_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_stitch_length", 10, &SewingMachine::pfStitchLengthCallback, this);
    pf_stitch_rpm_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_stitch_rpm", 10, &SewingMachine::pfStitchRPMCallback, this);
    pf_jog_sub_ = nh->subscribe("/scan_sewing_gui/pattern_former_jog", 10, &SewingMachine::pfJogCallback, this);

    ros_shutdown_sub_ = nh->subscribe("/scan_sewing_gui/ros_shutdown", 10, &SewingMachine::rosShutdownCallback, this);

}

void SewingMachine::run() {
    while ( ros::ok() ) {
        if (connected) {
            ThreadFunction();
        }
        if(motor_moving) {
            if(isMotorStopped()) {
                stop_count = 0;
                std_msgs::Empty signal;
                is_moving_pub_.publish(signal);
                motor_moving = false;
            }
        }
        ros::spinOnce();
//        loop_rate.sleep();
    }
}

void SewingMachine::rosShutdownCallback(const std_msgs::Bool::ConstPtr &msg) {
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
}

void SewingMachine::pfCallback(const std_msgs::Char::ConstPtr &msg) {
    char c = msg->data;
    if(c=='0') {
        PMACConnect();
    } else if(c=='1') {
        MoveOrigin();
    } else if(c=='2') {
        EMS();
    } else if(c=='3') {
        ZHome();
    } else if(c=='4') {
        TopStitch();
    } else if(c=='5') {
        MacroMini();
    }
}

void SewingMachine::pfLedCallback(const std_msgs::Bool::ConstPtr &msg) {
    bool checked = msg->data;
    if(checked) {
        CString str;
        str = "m7033=1\n";
        GetResponse(str);
    } else {
        CString str;
        str = "m7033=0\n";
        GetResponse(str);
    }
}

void SewingMachine::pfFootCallback(const std_msgs::Bool::ConstPtr &msg) {
    bool checked = msg->data;
    if(checked) {
        CString str;
        str = "m7049=1\n";
        GetResponse(str);
    } else {
        CString str;
        str = "m7049=0\n";
        GetResponse(str);
    }
}

void SewingMachine::pfFrameCallback(const std_msgs::Bool::ConstPtr &msg) {
    bool checked = msg->data;
    if(checked) {
        CString str;
        str = "m7051=1\n";
        GetResponse(str);
    } else {
        CString str;
        str = "m7051=0\n";
        GetResponse(str);
    }
}

void SewingMachine::pfControlCallback(const geometry_msgs::Point::ConstPtr &msg) {
    double pf_x = msg->x;
    double pf_y = msg->y;
    target_pos[0] = pf_x;
    target_pos[1] = pf_y;
    motor_moving = true;
    string str;
    str = "X";
    stringstream stream;
    stream << fixed << setprecision(3) << pf_x;
    string round_str = stream.str();
    str.append(round_str);
    stream.clear();
    stream.str(string());
    str.append(" Y");
    stream << fixed << setprecision(3) << pf_y;
    round_str = stream.str();
    str.append(round_str);
    str.append("\n");
    str.append("dwell0\n");
    str.append("m3\n");
    CommandInput(str);

}

void SewingMachine::pfJogVelCallback(const std_msgs::Float64::ConstPtr &msg) {
    jog_vel = msg->data;
}

void SewingMachine::pfJogRPMCallback(const std_msgs::Int32::ConstPtr &msg) {
    jog_rpm = msg->data;
}

void SewingMachine::pfStitchLengthCallback(const std_msgs::Float64::ConstPtr &msg) {
    stitch_length = msg->data;
}

void SewingMachine::pfStitchRPMCallback(const std_msgs::Int32::ConstPtr &msg) {
    stitch_rpm = msg->data;
}

void SewingMachine::pfJogCallback(const std_msgs::Char::ConstPtr &msg) {
    char c = msg->data;
    if(c=='0') {
        if(m_nXStatus[0] == 1) {
            JogStop(2);
        }
        if (m_nYStatus[0] == 1) {
            JogStop(1);
        }
        if(m_nZStatus[0] == 1) {
            JogStop(3);
        }
    } else if(c=='1') {
        JogPlus(2, jog_vel);
    } else if(c=='2') {
        JogPlus(1, jog_vel);
    } else if(c=='3') {
        JogMinus(2, jog_vel);
    } else if(c=='4') {
        JogMinus(1, jog_vel);
    } else if(c=='5') {
        JogPlus(3,(double)jog_rpm);
    } else if(c=='6') {
        JogMinus(3,(double)jog_rpm);
    }
}

void SewingMachine::TopStitch(){

    motor_position_out = true;
    string str = "m7051=1\n";
    str.append("dwell0\n");
    str.append("dwell500\n");

    ifstream file("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_ssm\\cmdline\\SL_Final.txt");\
    if (file.is_open()==true) {
        string line;
        bool tr = false;
        while(file) {
            getline(file,line);
            if(tr==false) {
                tr = true;
                str.append(line);
                str.append("\n");
                str.append("dwell0\n");
                str.append("m7049=0\n");
                str.append("dwell0\n");
                str.append("dwell1000\n");
                str.append("m1\n");
                str.append(line);
                str.append("\n");
            }
            str.append(line);
            str.append("\n");
        }
        file.close();
    }
    str.append("m5\n");
//    str.append("m7051=0\n");
    str.append("m7049=1\n");
    str.append("m3\n");
    CommandInput(str);
}

void SewingMachine::MacroMini(){

    // TODO

}

bool SewingMachine::isMotorStopped() {
    if(abs(target_pos[0]-m_dPosition[1]) <0.003 && abs(target_pos[1]-m_dPosition[0]) <0.003) {
        if(stop_count==5) {
            return true;
        } else {
            stop_count += 1;
            return false;
        }

    } else {
        return false;
    }
}


bool SewingMachine::PMACConnect() {
    m_hPmaclib = OpenRuntimeLink();
    m_dwDevice = 0;
    // m_dwDevice = DeviceSelect(NULL);
    m_bDeviceopen = false;
    for (int i = 0; i < 3; i++) {
        if (!m_bDeviceopen) {
            m_bDeviceopen = DeviceOpen(m_dwDevice);
        } else {
            connected = true;
            std_msgs::Bool _msg;
            _msg.data = true;
            connect_pub_.publish(_msg);
            return true;
        }
    }
    std_msgs::Bool _msg;
    _msg.data = false;
    connect_pub_.publish(_msg);
    return false;
}

void SewingMachine::CommandInput(string command_str) {
    CString filepath;
    filepath = "C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_ssm\\cmdline\\textboxbuffer.txt";
    ofstream file(filepath);
    if (file.is_open()) {
        CString str;
        file << "open prog1 clear\n";
        file << "P99=1\n";
        file << "TA(";
        file << to_string((int)m_dTa);
        file << ")\n";
        file << "TS(";
        file << to_string((int)m_dTs);
        file << ")\n";
        file << "F(";
        file << XYSpeed;
        file << ")\n";
        file << "ABS\n";
        file << "LINEAR\n";
        file << "DWELL(";
        file << to_string((int)m_dZdwelltime);
        file << ")\n";

        file << command_str;

        file << "DWELL(";
        file << to_string((int)m_dZdwelltime);
        file << ")\n";
        file << "P99=0\n";
        file << "close";
        file.close();
    }
    while(true) {
        if (DeviceDownload(m_dwDevice, NULL, NULL, NULL,
                           filepath.GetBuffer(), TRUE, TRUE, TRUE, TRUE) == TRUE) {
            break;
        }
    }
    PatternFormerStart();
//    if (DeviceDownload(m_dwDevice, NULL, NULL, NULL,
//        filepath.GetBuffer(), TRUE, TRUE, TRUE, TRUE) == TRUE)
//    {
//        PatternFormerStart();
//    }
//    else
//    {
//        cout << "download error" << endl;
//    }
}

void SewingMachine::PatternFormerStart() {

//    CString strStitch, strRPM;
    double dStitch, dRPM;
//    GetDlgItemText(IDC_EDIT_STITCH, strStitch);
//    GetDlgItemText(IDC_EDIT_ZRPM, strRPM);
//    dStitch = atof(strStitch);
//    dRPM = atof(strRPM);

    dStitch = stitch_length;
    // dRPM = 500;
    dRPM = stitch_rpm;

    double xyVel = dStitch / (60 / dRPM / 2); //- 0.001);

    CString str;
    str = XYSpeed;
    str.Append("=");
    str.Append(to_string(xyVel).c_str());
    str.Append(" i322=");
    str.Append(to_string(dRPM / 60 * m_dScale[2] / 1000).c_str());
    // str.Format("%s=%f i322=%f", XYSpeed, xyVel, dRPM / 60 * m_dScale[2] / 1000);
    GetResponse(str);
    str = "&1b1r";
    GetResponse(str);
    // GetResponse("&1b1r");
    str = "%100";
    GetResponse(str);
    // GetResponse("%100");
}

CString SewingMachine::GetResponse(CString msg) {
    CString str;
    str.Empty();
    if (!m_bDeviceopen) return str;
    TCHAR chbuf[255];
    DeviceGetResponse(m_dwDevice, chbuf, 255, (LPSTR)(LPCSTR)msg);
    CString strbuf;
    strbuf = chbuf;
    strbuf.Remove('\r');
    return strbuf;
}

void SewingMachine::JogPlus(int motorNum, double velocity)
{
    float vel = 0.0;
    if(motorNum!=3) {
        vel = velocity * m_dScale[motorNum - 1] / 1000;
    } else {
        vel = velocity / 60 * m_dScale[motorNum - 1] / 1000;
    }
    stringstream stream;
    stream << fixed << setprecision(2) << vel;
    string round_str = stream.str();
    CString str;
    str = "i";
    str.Append(to_string(motorNum).c_str());
    str.Append("22=");
    str.Append(round_str.c_str());
    stream.clear();
    str.Append(" #");
    str.Append(to_string(motorNum).c_str());
    str.Append("J+");
    GetResponse(str);
}

void SewingMachine::JogMinus(int motorNum, double velocity)
{
    float vel = 0.0;
    if(motorNum!=3) {
        vel = velocity * m_dScale[motorNum - 1] / 1000;
    } else {
        vel = velocity / 60 * m_dScale[motorNum - 1] / 1000;
    }
    stringstream stream;
    stream << fixed << setprecision(2) << vel;
    string round_str = stream.str();
    CString str;
    str = "i";
    str.Append(to_string(motorNum).c_str());
    str.Append("22=");
    str.Append(round_str.c_str());
    stream.clear();
    str.Append(" #");
    str.Append(to_string(motorNum).c_str());
    str.Append("J-");
    GetResponse(str);
}

void SewingMachine::JogStop(int motorNum)
{
    CString str;
    str = "#";
    str.Append(to_string(motorNum).c_str());
    str.Append("J/");
    GetResponse(str);
}

void SewingMachine::ZHome() {
    CString str;
    str = "#3hm";
    GetResponse(str);
}

void SewingMachine::EMS() {
    CString str;
    str = "&1a";
    GetResponse(str);
    str = "p99=0 #3j/";
    GetResponse(str);
    str = "%100";
    GetResponse(str);
}

void SewingMachine::MoveOrigin() {

    CString str, strbuf;
    CString strbuf1, strbuf2;
    strbuf1 = "0";
    strbuf2 = "0";

    str = OriginOffsetX;
    str.Append("=");
    str.Append(strbuf1);
    str.Append(" ");
    str.Append(OriginOffsetY);
    str.Append("=");
    str.Append(strbuf2);
    str.Append(" ");
    str.Append(GoOrigin);
    str.Append("=1");
    GetResponse(str);
}

void SewingMachine::ThreadFunction() {
    TCHAR chbuf[255];
    DeviceGetResponse(m_dwDevice, chbuf, 255, "#1P #2P #3P #1V #2V #3V");

    string str = chbuf;
    istringstream ss(str);
    string stringBuffer;
    vector<string> x;
    x.clear();
    while (getline(ss, stringBuffer, '\r'))
    {
        x.push_back(stringBuffer);
    }
    m_dPosition[0] = atof(x[0].c_str());
    m_dPosition[1] = atof(x[1].c_str());
    m_dPosition[2] = atof(x[2].c_str());
    m_dVelocity[0] = atof(x[3].c_str());
    m_dVelocity[1] = atof(x[4].c_str());
    m_dVelocity[2] = atof(x[5].c_str());

    m_dPosition[0] = m_dPosition[0] / m_dScale[0];
    m_dPosition[1] = m_dPosition[1] / m_dScale[1];
    m_dPosition[2] = abs((int)m_dPosition[2] % (int)m_dScale[2]);
    m_dVelocity[0] = m_dVelocity[0] * 2258.6505 / m_dScale[0];
    m_dVelocity[1] = m_dVelocity[1] * 2258.6505 / m_dScale[1];
    m_dVelocity[2] = m_dVelocity[2] * 2258.6505 / m_dScale[2] * 60;

    std_msgs::Float64MultiArray _msg;
    _msg.data.push_back(m_dPosition[1]);
    _msg.data.push_back(m_dPosition[0]);
    _msg.data.push_back(m_dPosition[2]);
    _msg.data.push_back(m_dVelocity[1]);
    _msg.data.push_back(m_dVelocity[0]);
    _msg.data.push_back(m_dVelocity[2]);
    motor_pos_pub_ .publish(_msg);


    TCHAR chbuf2[255];
    DeviceGetResponse(m_dwDevice, chbuf2, 255, "m139 m133 m140 m131 m132 m142 m239 m233 m240 m231 m232 m242 m339 m333 m340 m331 m332 m342");

    string str2 = chbuf2;
    istringstream ss2(str2);
    string stringBuffer2;
    vector<string> x2;
    x2.clear();
    int nbuf;
    while (getline(ss2, stringBuffer2, '\r'))
    {
        x2.push_back(stringBuffer2);
    }
    m_nYStatus[0] = atoi(x2[0].c_str());
    if (atoi(x2[0].c_str()) * atoi(x2[1].c_str()) == 1)
        nbuf = 0;
    else
        nbuf = 1;
    m_nYStatus[1] = nbuf;
    m_nYStatus[2] = atoi(x2[3].c_str());
    m_nYStatus[3] = atoi(x2[4].c_str());
    m_nYStatus[4] = atoi(x2[5].c_str());

    m_nXStatus[0] = atoi(x2[6].c_str());
    if (atoi(x2[6].c_str()) * atoi(x2[7].c_str()) == 1)
        nbuf = 0;
    else
        nbuf = 1;
    m_nXStatus[1] = nbuf;
    m_nXStatus[2] = atoi(x2[9].c_str());
    m_nXStatus[3] = atoi(x2[10].c_str());
    m_nXStatus[4] = atoi(x2[11].c_str());

    m_nZStatus[0] = atoi(x2[12].c_str());
    if (atoi(x2[12].c_str()) * atoi(x2[13].c_str()) == 1)
        nbuf = 0;
    else
        nbuf = 1;
    m_nZStatus[1] = nbuf;
    m_nZStatus[2] = atoi(x2[15].c_str());
    m_nZStatus[3] = atoi(x2[16].c_str());
    m_nZStatus[4] = atoi(x2[17].c_str());

//    std_msgs::Int32MultiArray _msg2;
//    for(int i=0;i<5;i++) {
//        _msg2.data.push_back(m_nXStatus[i]);
//    }
//    for(int i=0;i<5;i++) {
//        _msg2.data.push_back(m_nYStatus[i]);
//    }
//    for(int i=0;i<5;i++) {
//        _msg2.data.push_back(m_nZStatus[i]);
//    }
//   motor_state_pub_ .publish(_msg2);


    TCHAR chbuf3[255];
    DeviceGetResponse(m_dwDevice, chbuf3, 255, "m7000,64");

    string str3 = chbuf3;
    istringstream ss3(str3);
    string stringBuffer3;
    vector<string> x3;
    x3.clear();

    while (getline(ss3, stringBuffer3, '\r'))
    {
        x3.push_back(stringBuffer3);
    }

    for (int i = 0; i < 32; i++)
    {
        m_nInput[i] = atoi(x3[i].c_str());
    }

    for (int i = 0; i < 32; i++)
    {
        m_nOutput[i] = atoi(x3[i+32].c_str());
    }

    std_msgs::Int32MultiArray _msg3;
    _msg3.data.push_back(m_nOutput[1]);
    _msg3.data.push_back(m_nOutput[17]);
    _msg3.data.push_back(m_nOutput[19]);
    pf_io_pub_ .publish(_msg3);

}



int main(int argc, char **argv) {

    SewingMachine ssm(argc,argv);

    ssm.init();
    ssm.run();

    return 0;
}
