#pragma once
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "fastech/MOTION_DEFINE.h"
#include "fastech/ReturnCodes_Define.h"
#include "fastech/FAS_EziMOTIONPlusE.h"


class StepperMotor

{
private:
	BYTE comPort;
	bool xMotioning, yMotioning;
	int baudrate;
	int mm_to_pulse = 1000;
	int currentPosAbs = 0;
	DWORD dwAxisStatus;
	EZISTEP_AXISSTATUS stAxisStatus;

public:
	StepperMotor(BYTE lastIP, int baudrate);
	~StepperMotor();

	// Pre-processing
	int checkStatus(int slaveNo);
	void clearAlarm(int slaveNo);
	void setMotorParameter(int slaveNo, int ppr, int maxSpeed, int startSpeed, int accTime, int decTime, int orgSearchSpeed, int orgMethod);
	int findZeroPosition();

	// Origin Search
	int limitPosSearchX();
	int limitPosSearchY();
	int limitPosSearch();	// For both motors.
	int limitPosSearch_own(int slaveNo);
	// Action
	int moveSingleAxis_mm(int slaveNo, double posAbs, double period);
	int moveAxis_mm(double posAbsX, double posAbsY, double period);
	bool isMotioning(int slaveNo);
	bool getMotionState(int slaveNo);
	int emergencyStop();


	// Exception Handling
};