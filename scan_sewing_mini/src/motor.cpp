#include "motor.hpp"
using namespace PE;


StepperMotor::StepperMotor(ros::Publisher& int32_pub, ros::Publisher& point_pub)
	: mini_state_pub(int32_pub), mini_motor_pos_pub(point_pub) {
	std_msgs::Int32 msg;
    while(FAS_ConnectTCP(192,168,0,idX,idX) == FALSE){
		msg.data = 11;
		mini_state_pub.publish(msg);
        // std::cout << "X Motor Connection Failed" << std::endl;
        Sleep(1000);
    }
	
	while(FAS_ConnectTCP(192,168,0,idY1,idY1) == FALSE){
		msg.data = 12;
		mini_state_pub.publish(msg);
        // std::cout << "Y1 Motor Connection Failed" << std::endl;
        Sleep(1000);
    }

	while(FAS_ConnectTCP(192,168,0,idY2,idY2) == FALSE){
		msg.data = 13;
		mini_state_pub.publish(msg);
        // std::cout << "Y2 Motor Connection Failed" << std::endl;
        Sleep(1000);
    }
	//preDirX = -1;
	//preDirY = -1;
	del_x = -0.00;
	del_y = -0.00; 
	prePosX = -5.0;
	prePosY = -5.0;
	preComX = 0;
	preComY = 0;
	msg.data = 14;
	mini_state_pub.publish(msg);
	// std::cout << "Successfully Connected" << std::endl;
}

StepperMotor::~StepperMotor(){
	//this->servoEnable(false);
    FAS_Close(idX);
	FAS_Close(idY1);
	FAS_Close(idY2);
}

void StepperMotor::clearAlarm(){
    FAS_ServoAlarmReset(idX);
	FAS_ServoAlarmReset(idY1);
	FAS_ServoAlarmReset(idY2);

	return;
}

void StepperMotor::setMotorParameter(int id, int ppr, int maxSpeed, int startSpeed, int accTime, int decTime, int orgSpeed, int orgSearchSpeed, int orgMethod){
    FAS_SetParameter(id, 0, ppr);
	FAS_SetParameter(id, 1, maxSpeed);
	FAS_SetParameter(id, 2, startSpeed);
	FAS_SetParameter(id, 3, accTime);
	FAS_SetParameter(id, 4, decTime);
	FAS_SetParameter(id, 14, orgSpeed);
	FAS_SetParameter(id, 15, orgSearchSpeed);
	FAS_SetParameter(id, 17, orgMethod);
	FAS_SetParameter(id, 18, 0);
	FAS_SetParameter(id, 25, 0);

    clearAlarm();
    return;
}

void StepperMotor::setValueCompensation(double xp, double xn, double yp, double yn){
	this->increXP = xp/1000.0;
	this->increXN = xn/1000.0;
	this->increYP = yp/1000.0;
	this->increYN = yn/1000.0;

	return;
}

int StepperMotor::servoEnable(bool enable) {
	int nRtn = 1;
	nRtn *= FAS_ServoEnable(idX, enable);
	nRtn *= FAS_ServoEnable(idY1, enable);
	nRtn *= FAS_ServoEnable(idY2, enable);
	return nRtn;
}

int StepperMotor::servoEnable(int id, bool enable){
    return FAS_ServoEnable(id, enable);
}

int StepperMotor::moveSingleAxis_mm(int id, double posAbs, double period){
    long pulseAbs = posAbs * mm_to_pulse;	// 1rev : 1mm -> 1/1000 rev : 1 micron
	int velocity = abs(pulseAbs	/ period);
	int nRtn;
    nRtn = FAS_MoveSingleAxisAbsPos(id, pulseAbs, 250000);

	return (nRtn == FMM_OK);

}

void StepperMotor::isMotioning(int id){
    EZISERVO_AXISSTATUS stAxisStatus;
    int nRtn;
    do{
        Sleep(1);
        nRtn = FAS_GetAxisStatus(id, &dwAxisStatus);
        stAxisStatus.dwValue = dwAxisStatus;
    }
    while(stAxisStatus.FFLAG_MOTIONING);
}

int StepperMotor::moveAxis_mm(double posX, double posY, double period) {
	int dirX, dirY;
	dirX = 0;
	dirY = 0;
	
	// X compensation
	if (posX > prePosX){
		dirX = 1;
		if (preDir[0] == -1) {
			xIncre += increXP;
		}
//		xIncre += increXP * (dirX != preDirX);
	}
	else if (posX < prePosX){
		dirX = -1;
		if (preDir[0] == 1) {
			xIncre -= increXN;
		}
	//	xIncre -= increXN * (dirX != preDirX);
	}
	else {
		dirX = 0; 
	}

	preComX += randomComp(prePosX, posX);
	this->prePosX = posX + 0.0;
	posX += xIncre + preComX;

	// Y compensation
	if (posY > prePosY){
		dirY = 1;
		if (preDir[1] == -1) {
			yIncre += increYP;
		}
	}
	else if (posY < prePosY){
		dirY = -1;
		if (preDir[1] == 1) {
			yIncre -= increYN;
		}
	}
	else { dirY = 0; }
	preComY += randomComp(prePosY, posY);
	prePosY = posY + 0.0;
	posY += yIncre + preComY;

	// Apply HW Limit constraint
	if (posX > 0) {
		posX = 0;
	}
	if (posX < -10.0) {
		posX = -10.0;
	}
	if (posY > 0) {
		posY = 0.0;
	}
	if (posY < -10.0) {
		posY = -10.0;
	}

	// Update previous commands
	
	

	//std::cout << "X comp  " << posX << ", " << prePosX << ", " << preComX << ", " << dirX << ", " << preDir[0] << ", " << xIncre << std::endl;
	//std::cout << "Y comp  " << posY << ", " << prePosY << ", " << preComY << ", " << dirY << ", " << preDir[1] << ", " << yIncre << std::endl;

	if (dirX != 0) {
		preDir[0] = dirX + 0;
	}
	if (dirY != 0) { preDir[1] = dirY + 0; }

	// Actual actuation command
	int resX = this->moveSingleAxis_mm(idX, posX, period);
	int resY1 = this->moveSingleAxis_mm(idY1, posY, period);
	int resY2 = this->moveSingleAxis_mm(idY2, posY, period);
	/*geometry_msgs::Point msg;
	msg.x = posX;
	msg.y = posY;	
	msg.z = 0.0;
	mini_motor_pos_pub.publish(msg);
	msg.x = preComX;
	msg.y = preComY;
	msg.z = 1.0;
	mini_motor_pos_pub.publish(msg);*/
	//std::cout << "Target: " << posXtemp << ", " << posYtemp << "  Command: ";
	//std::cout << preComX << ", " << posY << std::endl;
	if (resX == 1 && resY1 == 1 && resY2 == 1) return 1;
	else return 0;
}

void StepperMotor::getPreCommand(double* arr) {
	arr[0] = prePosX + 0.0;
	arr[1] = prePosY + 0.0;
	arr[2] = preComX;
	arr[3] = preComY;
	return;
}

int StepperMotor::limitPosSearchX() {
	std_msgs::Int32 msg;
	if (FAS_MoveOriginSingleAxis(idX) != FMM_OK) {
		msg.data = 15;
		mini_state_pub.publish(msg);
		// std::cout << "ERROR - MOTOR - motor " << 0 << " - limitPosSearch " << "\n";
		return -1;
	}
	msg.data = 16;
	mini_state_pub.publish(msg);
	// std::cout << "Notification - motor " << 0 << " - limitPosSearch - Limit Search End" << "\n";

	return 1;
}

void StepperMotor::tmp() {
	DWORD dwInput;
	int nRtn;
	int iDir = 1;
	long lVelocityFast, lVelocitySlow;
	lVelocityFast = 1000;
	lVelocitySlow = 200;
	FAS_MoveVelocity(idY2, lVelocityFast, iDir);

	do {
		nRtn = FAS_GetIOInput(idY1, &dwInput);
	} while (!(dwInput & STEP_IN_BITMASK_LIMITP));

	FAS_MoveStop(idY2);

	Sleep(1000);

	FAS_MoveVelocity(idY2, lVelocitySlow, !iDir);

	do {
		nRtn = FAS_GetIOInput(idY1, &dwInput);
	} while (dwInput & STEP_IN_BITMASK_LIMITP);

	FAS_MoveStop(idY2);

}
int StepperMotor::limitPosSearchY(){
	DWORD dwInput;
	int nRtn;
	int iDir = 1;
	long lVelocityFast, lVelocitySlow;
	lVelocityFast = 4000;
	lVelocitySlow = 500;

	FAS_MoveVelocity(idY1, lVelocityFast, iDir);
	FAS_MoveVelocity(idY2, lVelocityFast, iDir);
	
	do{
		nRtn = FAS_GetIOInput(idY1, &dwInput);
	}
	while(!(dwInput & STEP_IN_BITMASK_LIMITP));
	FAS_MoveStop(idY1);
	FAS_MoveStop(idY2);

	Sleep(1000);

	FAS_MoveVelocity(idY1, lVelocitySlow, !iDir);
	// std::cout << "back 1" << std::endl;
	FAS_MoveVelocity(idY2, lVelocitySlow, !iDir);
	// std::cout << "back 2" << std::endl;
	
	do{
		nRtn = FAS_GetIOInput(idY1, &dwInput);
	}
	while(dwInput & STEP_IN_BITMASK_LIMITP);
	FAS_MoveStop(idY1);
	FAS_MoveStop(idY2);

	this->isMotioning(idY1);
	this->isMotioning(idY2);
	
	FAS_ClearPosition(idY1);
	FAS_ClearPosition(idY2);

	return 1;
}


int StepperMotor::limitPosSearch() {
	if (this->limitPosSearchX() == -1) {
		return -1;
	}
	if (this->limitPosSearchY() == -1) {
		return -1;
	}
	return 1;
}

int StepperMotor::emergencyStop(){
	FAS_EmergencyStop(idX);
	FAS_EmergencyStop(idY1);
	FAS_EmergencyStop(idY2);
	return 1;
}

double randomComp(double pre, double cur) {
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<> dist(1, 100);
	int r;
	double tmp;
	int dir = cur < pre;
	double compValue = 0.0;

	if (pre < cur) {
		while ((cur - pre >= 0.01)) {
			r = dist(mt);
			tmp = 0;// getSingleComp(r, 0, 0);
			compValue += tmp;
			pre += 0.01;
		}
	}
	else {
		while((pre - cur >= 0.01)) {
			r = dist(mt);
			tmp = 0;// getSingleComp(r, 0, 1);
			compValue += tmp;
			pre -= 0.01;
		}
	}

	return compValue;

}

double getSingleComp(int r, int axis, int dir) {
	// X axis = 0, Y axis = 1
	// increase = 0, decrease = 1
	if (axis == 0) {
		if (dir == 0) {
			if (r <= 1) {
				return -0.002;
			}
			else if (r <= 22) {
				return -0.001;
			}
			else if (r <= 75) {
				return 0.0;
			}
			else if (r <= 99) {
				return 0.001;
			}
			else {
				return 0.002;
			}
		}
		else {
			if (r <= 1) {
				return -0.002;
			}
			else if (r <= 26) {
				return -0.001;
			}
			else if (r <= 78) {
				return 0.0;
			}
			else if (r <= 98) {
				return 0.001;
			}
			else {
				return 0.002;
			}
		}
	}
		
	else {
		if (dir == 0) {
			if (r <= 1) {
				return -0.002;
			}
			else if (r <= 19) {
				return -0.001;
			}
			else if (r <= 83) {
				return 0.0;
			}
			else if (r <= 100) {
				return 0.001;
			}
			else {
				return 0.002;
			}
		}
		else {
			if (r <= 1) {
				return -0.002;
			}
			else if (r <= 18) {
				return -0.001;
			}
			else if (r <= 79) {
				return 0.0;
			}
			else if (r <= 99) {
				return 0.001;
			}
			else {
				return 0.002;
			}
		}
	}
	return 1;
}
