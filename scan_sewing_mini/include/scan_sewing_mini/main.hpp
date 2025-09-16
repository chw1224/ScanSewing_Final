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
#include <random>
#include <math.h>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Char.h"
#include "geometry_msgs/Point.h"
#include "SerialPort.h"
#include "motor.hpp"
#include "config.h"
#include "Windows.h"

#define IDX 0
#define IDY 1

#define comPortSensingBoard 7
#define comPortIRSensor 13
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
double refPosX = -5.0;
double refPosY = -5.0;
double tempPeriod = 0.1;
unsigned int needleCount;
bool needleSafety = true;
bool stitch = false;
bool stopsign = false;
bool nextFlag = true;
int scanComplete = 0;
double posDiff = 0.05;
int mode = 1;
unsigned int pathNum = 3;
float incre[4] = {0.0, 0.0, 0.0, 0.0};

double coeff_travel_range0_neg = 0.009;
double coeff_travel_range1_neg = 0.021;
double coeff_travel_range2_neg = 0.012;
double coeff_travel_range3_neg = 0.002;

double coeff_travel_range0_pos = 0.003;
double coeff_travel_range1_pos = 0.003;
double coeff_travel_range2_pos = -0.005;
double coeff_travel_range3_pos = -0.005;

double coeff_travel_range0_negX = 0.002;
double coeff_travel_range1_negX = 0.000;
double coeff_travel_range2_negX = -0.00;
double coeff_travel_range3_negX = -0.001;

double coeff_travel_range0_posX = -0.002;
double coeff_travel_range1_posX = 0.007;
double coeff_travel_range2_posX = 0.008;
double coeff_travel_range3_posX = 0.006;
double coeff_dist_range0 = 0.00;
double coeff_dist_range1 = 0.009;
double coeff_dist_range2 = 0.021;
double coeff_dist_range3 = 0.012;
double coeff_dist_range4 = 0.002;

// Function define
void readPatternFormer();
void emergencyCheck();
void isMotioning();
double get_travel_error_compensation(double target);