#pragma once
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <random>
#include <algorithm>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include "MOTION_DEFINE.h"
#include "ReturnCodes_Define.h"
#include "FAS_EziMOTIONPlusE.h"


class StepperMotor

{
private:
	bool xMotioning = false;
	bool yMotioning = false;

	int preDirX = -1;
	int preDirY = -1;

	int dummyValue = 1;
	int dummyValue2 = 1;
//	double preDirY = -1;
	

	int dummy3 = 1;
	int dummy4 = 1;

	double del_x, del_y;
	double xIncre = 0;
	double yIncre = 0;

	int baudrate;
	int mm_to_pulse = 5000;
	int currentPosAbs = 0;
	int preDir[2] = { -1, -1 };

	static const int idX = 2;
	static const int idY1 = 3;
	static const int idY2 = 4;
	DWORD dwAxisStatus;
	EZISTEP_AXISSTATUS stAxisStatus;
	ros::Publisher mini_state_pub;
	ros::Publisher mini_motor_pos_pub;

	double prePosX;
	double prePosY;
	double preComX;
	double preComY;
	double increXP, increXN, increYP, increYN;


public:
	StepperMotor(ros::Publisher& int32_pub, ros::Publisher& point_pub);
	~StepperMotor();

	// Pre-processing
	void clearAlarm();
	void setMotorParameter(int slaveNo, int ppr, int maxSpeed, int startSpeed, int accTime, int decTime, int orgSpeed, int orgSearchSpeed, int orgMethod);
	void setValueCompensation(double xp, double xn, double yp, double yn);
	int servoEnable(int id, bool enable);
	int servoEnable(bool enable);
	void tmp();
	// Origin Search
	int limitPosSearchX();
	int limitPosSearchY();
	int limitPosSearch();	// For both motors.

	// Action
	int moveSingleAxis_mm(int id, double posAbs, double period);
	int moveAxis_mm(double posX, double posY, double period);
	void isMotioning(int id);
	int emergencyStop();
	
	void getPreCommand(double* arr);


	// Exception Handling
};

double randomComp(double pre, double cur);
double getSingleComp(int r, int axis, int side);