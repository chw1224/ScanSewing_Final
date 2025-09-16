#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "SerialPort.h"
#include "motor.h"
#include "config.h"
#include "Windows.h"

#define IDX 0
#define IDY 1

#define comPortSensingBoard 7
#define comPortIRSensor 14
struct PatternFormer{
	int dummy = 0;
	int irState;
	bool foot, frame, trim, loose, reset, breakLine, undefinedInput; // Foot, Frame, Trim, Loose, Reset, Break Line, Not Defined
	bool ready = false;
	bool scanComplete = false;
	bool topStitch = false;
	int needlePosition;
	unsigned int needleCnt = 0 ;
};

using namespace std;
using std::thread;

// Define Variables
static StepperMotor* mini;
static PatternFormer* patternFormer;
std::vector<float> xPath, yPath;
unsigned int nPath;
double refPosX = -2.5;
double refPosY = -0;
double tempPeriod = 0.1;
unsigned int needleCount;
bool needleSafety = true;
bool stitch = false;
bool stopsign = false;
bool nextFlag = true;


// Function define
void readPatternFormer();
void emergencyCheck();
void isMotioning();