#include "motor.h"

StepperMotor::StepperMotor(BYTE comPort, int baudrate) {
	while (FAS_Connect(comPort, baudrate) == FALSE) {
		std::cout << "Connection Failed" << std::endl;
		Sleep(1000);
	}
	std::cout << "Successfully Connected" << std::endl;
	//FAS_StepAlarmReset
	this->comPort = comPort;
	this->baudrate = baudrate;
}

StepperMotor::~StepperMotor() {
	FAS_Close(this->comPort);
}

int StepperMotor::checkStatus(int slaveNo) {
	if (FAS_IsSlaveExist(this->comPort, slaveNo) == FALSE)	return 0;
	else return 1;
}

void StepperMotor::setMotorParameter(int slaveNo, int ppr, int maxSpeed, int startSpeed, int accTime, int decTime, int orgSearchSpeed, int orgMethod) {
	FAS_SetParameter(this->comPort, slaveNo, 0, ppr);
	FAS_SetParameter(this->comPort, slaveNo, 1, maxSpeed);
	FAS_SetParameter(this->comPort, slaveNo, 2, startSpeed);
	FAS_SetParameter(this->comPort, slaveNo, 3, accTime);
	FAS_SetParameter(this->comPort, slaveNo, 4, decTime);
	FAS_SetParameter(this->comPort, slaveNo, 15, orgSearchSpeed);
	FAS_SetParameter(this->comPort, slaveNo, 17, orgMethod);

	FAS_SetParameter(this->comPort, slaveNo, 22, 0);

	this->clearAlarm(2);
	this->clearAlarm(3);
	this->clearAlarm(4);
	return;
}

void StepperMotor::clearAlarm(int slaveNo) {
	FAS_StepAlarmReset(this->comPort, slaveNo, 1);
	FAS_StepAlarmReset(this->comPort, slaveNo, 0);
	return;
}

// posAbs [millimeter]
int StepperMotor::moveSingleAxis_mm(int slaveNo, double posAbs, double period) {
	long pulseAbs = posAbs * mm_to_pulse;	// 1rev : 1mm -> 1/1000 rev : 1 micron
	int velocity = abs(pulseAbs	/ period);

	if (isMotioning(slaveNo) == true) {
		std::cout << "StepperMotor - slave " << slaveNo << " - can't execute - still motioning" << std::endl;
		return 0;
	}
	if (FAS_MoveSingleAxisAbsPos(this->comPort, slaveNo, pulseAbs, 30000) != FMM_OK) {
#ifdef DEBUG
		std::cout << "DEBUG - Steppermotor - slave " << slaveNo << " - moveResult: " << "Failed to move" << '\n';
#endif
		return -1;
	}

#ifdef DEBUG
	std::cout << "DEBUG - Steppermotor - moveSuccess - pulseInc: " << pulseRel << '\n';
#endif

	return 1;
}

int StepperMotor::moveAxis_mm(double posX, double posY, double period) {
	if (posX > 0) {
		posX = 0;
	}
	if (posX < -5.0) {
		posX = -5.0;
	}
	if (posY > 0) {
		posY = 0.0;
	}
	if (posY < -5.0) {
		posY = -5.0;
	}
	int resX = this->moveSingleAxis_mm(0, posX, period);
	int resY1 = this->moveSingleAxis_mm(1, posY, period);
	int resY2 = this->moveSingleAxis_mm(2, posY, period);

	if (resX == 1 && resY1 == 1 && resY2 == 1) return 1;
	else return 0;
}

bool StepperMotor::isMotioning(int slaveNo) {
	//do {
	if (FAS_GetAxisStatus(comPort, slaveNo, &dwAxisStatus) != 0) {
		std::cout << "ERROR - StepperMotor - slaveNo " << slaveNo << " - isMotioning - failed to execute isMotioning" << "\n";
		return false;
	}
	stAxisStatus.dwValue = dwAxisStatus;
	
	if(slaveNo == 0) xMotioning = stAxisStatus.FFLAG_MOTIONING;
	else yMotioning = stAxisStatus.FFLAG_MOTIONING;

	return stAxisStatus.FFLAG_MOTIONING;
}

bool StepperMotor::getMotionState(int slaveNo){
	if(slaveNo == 0) return xMotioning;
	else return yMotioning;
}

int StepperMotor::limitPosSearchX() {
	if (FAS_MoveOriginSingleAxis(comPort, 0) != FMM_OK) {
		std::cout << "ERROR - MOTOR - motor " << 0 << " - limitPosSearch " << "\n";
		return -1;
	}
	while (this->isMotioning(0) == true) {
		continue;
	}

	std::cout << "Notification - motor " << 0 << " - limitPosSearch - Limit Search End" << "\n";


#ifdef DEBUG
	std::cout << "DEBUG - motor - limitPosSearch - Current Zero : ";
#endif

	return 1;
}

int StepperMotor::limitPosSearchY(){
	DWORD dwInput;
	int nRtn;
	int iDir = 1;
	long lVelocityFast, lVelocitySlow;
	lVelocityFast = 500;
	lVelocitySlow = 100;

	FAS_MoveVelocity(comPort, 1, lVelocityFast, iDir);
	FAS_MoveVelocity(comPort, 2, lVelocityFast, iDir);
	
	do{
		nRtn = FAS_GetIOInput(comPort, 1, &dwInput);
	}
	while(!(dwInput & STEP_IN_BITMASK_LIMITP));
	FAS_MoveStop(comPort, 1);
	FAS_MoveStop(comPort, 2);

	FAS_MoveVelocity(comPort, 1, lVelocitySlow, !iDir);
	FAS_MoveVelocity(comPort, 2, lVelocitySlow, !iDir);
	
	do{
		nRtn = FAS_GetIOInput(comPort, 1, &dwInput);
	}
	while(dwInput & STEP_IN_BITMASK_LIMITP);
	FAS_MoveStop(comPort, 1);
	FAS_MoveStop(comPort, 2);

	while(this->isMotioning(1) || this->isMotioning(2)){
		continue;
	}
	
	FAS_ClearPosition(comPort, 1);
	FAS_ClearPosition(comPort, 2);

	return 1;
}


int StepperMotor::limitPosSearch() {
	if (this->limitPosSearchX() == -1) {
		return -1;
	}
	if (this->limitPosSearchY() == -1) {
		return -1;
	}
}

int StepperMotor::emergencyStop(){
	FAS_AllMoveStop(comPort);
	return 1;
}