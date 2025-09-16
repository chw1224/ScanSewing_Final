#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <stdio.h>
#include <string.h>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "SerialPort.h"
#include "motor.h"
#include "config.h"
#include "Windows.h"


void main() {
	ifstream file("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_ssm\\cmdline\\SL_Final.txt");
	string line;
	float x, y;
	int point_num = 0;
	while (file) {
		getline(line, line);
		vector<string> result = split(line, ' ');
		result[0].erase(0);
		result[1].erase(1);
		x = stof(result[0]);
		y = stof(result[1]);
		point_num++;
		cout << x << y << endl;
	}
	file.close();