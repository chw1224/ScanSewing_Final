#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <stdio.h>
#include <string.h>
#include <thread>
#include "SerialPort.h"
#include <Windows.h>
#include "ros/ros.h"
#include <std_msgs/Bool.h>

#define IDX 0
#define IDY 1

struct PatternFormer
{
	int foot, frame, trim, loose, breakLine, outputA; // Foot, Frame, Trim, Loose, Reset, Break Line, Not Defined
	int reset = 0;
};

using namespace std;
using std::thread;

// Define Variables
std::vector<double> xPath, yPath;
int nPath;
double refPosX = -5;
double refPosY = 0.0;
double tempPeriod = 0.1;
unsigned int needleCount;
bool isSafe = true;
bool stopsign = false;