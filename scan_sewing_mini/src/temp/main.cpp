#include "main.h"

#define MAX_DATA_LENGTH 255

ofstream motor_pos_out("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\file.csv");

// SerialPort patternFormerSensingBoard(comPortSensingBoard, 115200, 0.01);
SerialPort needleSensingBoard(14, 115200, 0.01);

vector<string> split(string str, char Delimiter) {
	istringstream iss(str);             
	string buffer;                      

	vector<string> result;

	while (getline(iss, buffer, Delimiter)) {
		result.push_back(buffer); 
	}

	return result;
}

void scanCallback(const std_msgs::BoolConstPtr& msg){
	patternFormer->scanComplete = msg->data;
	return;
}

void topStitchCallback(const std_msgs::BoolConstPtr& msg){
	patternFormer->topStitch = msg->data;
	return;
}

void nextCallback(const std_msgs::BoolConstPtr& msg){
	nextFlag = false;
}

void loadExagPath() {
	nPath = 107;
	for (int i = 0; i < nPath; i++) {
		xPath.push_back(-2.5);
		yPath.push_back(-1.5 - 2 * (i % 2));
	}
}
void loadPatternformerPath() {
	ofstream pathFile("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\pathFile.txt");
	ifstream file("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_ssm\\cmdline\\SL_Final.txt");
	string line;
	float rx, ry;
	float sx, sy;
	float ax0, ay0;
	float delx, dely;
	float offX, offY;

	offX = 0;
	offY = 0;

	sx = 710;
	sy = 50;

	ax0 = 155;
	ay0 = -255.5;

	nPath = 0;

	while (file) {
		getline(file, line);
		vector<string> result = split(line , ' ');
		if (result.size() == 2) {
			result[0].erase(0, 1);
			result[1].erase(0, 1);
			rx = stof(result[0]);
			ry = stof(result[1]);
			delx = -(ax0 + sx - rx) - 2.5 + 3 * nPath;
			dely = (ay0 + sy - ry);	

			pathFile << delx << " " << dely << std::endl;
			if (nPath == 0){
				offX = -delx - 0;
				offY = -dely - 0;
			}
			 if (delx < -5.0){
			 	delx = -5.0;
			 }
			 else if (delx > 0.0){
			 	delx = 0.0;
			 }
			 if (dely < -5.0){
			 	dely = -5.0;
			 }
			 else if (dely > 0.0){
			 	dely = 0.0;
			 }
			
			xPath.push_back(delx);
			yPath.push_back(dely);
			nPath++;
		}
	}
	file.close();
	return;
}

void rosRun(){
	ros::Rate lr(100);
	while(ros::ok()){
		ros::spinOnce();
		lr.sleep();
	}
	cout << "Ros Out" << endl;
	stopsign = true;
}

// void readSensingBoard(){
// 	int MAX_DATA_LENGTH = 255;
// 	bool foot, frame, trim, loose, reset, breakLine, undefinedInput;
// 	char buffer[100];

// 	while(!stopsign){
// 		int nread = patternFormerSensingBoard.readline((uint8_t*)buffer, MAX_DATA_LENGTH);
// 		if(nread>0){
// 			buffer[nread-2]=0;
// 			sscanf(buffer, "<%d %d %d %d %d %d %d>", 
// 			&foot, &frame, &trim, &loose, &reset, &breakLine);

// 			patternFormer->foot = foot;
// 			patternFormer->frame = frame;
// 			patternFormer->loose = loose;
// 			patternFormer->trim = trim;
// 			patternFormer->reset = reset;
// 			patternFormer->breakLine = breakLine;
// 		}	
// 	}
// 	return;
// }

void readIRSensor(){
int irState;
char buffer[MAX_DATA_LENGTH];
	while(!stopsign){
		Sleep(1);
		int nread = needleSensingBoard.readline((uint8_t*)buffer, MAX_DATA_LENGTH);
		if(nread > 0) buffer[nread-2] = 0;
			sscanf(buffer, "%d", &irState);
			
			// Needle safety decision logic
			if (nread == 3){
				needleSafety = false;
			}
			if (nread == 4){
				patternFormer->needleCnt++;
				stitch = false;
				needleSafety = true;
			}
			///

			patternFormer->irState = irState;
		
	}
	return;
}

void checkEmergency(){
	while(!stopsign){
		if (!needleSafety){
			if (mini->getMotionState(0) && mini->getMotionState(1)){
				mini->emergencyStop();
			}
		}
	}
		
}
// void readPatternFormer() {
// 	while (!stopsign){
// 		if (needleSafety && (patternFormer->encoderZ > 4000)) {
// 			if (mini->getMotionState(0) && mini->getMotionState(1)){
// 				mini->emergencyStop();
// 				break;
// 			}
// 			needleSafety = false;
// 			stitch = false;
// 			patternFormer->needleCnt++;
// 		}
// 		else if (!needleSafety && (patternFormer->encoderZ < 1000)) {
// 			needleSafety = true;
// 		}
// 	}
// 	return;
// }

void isMotioning() {
	while (mini->isMotioning(2)) continue;
	while (mini->isMotioning(3)) continue;
	while (mini->isMotioning(4)) continue;
	return;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "scan_sewing_mini");
	ros::NodeHandle n;
	ros::Publisher mini_state_pub = n.advertise<std_msgs::Int32>("/scan_sewing_mini/mini_state", 10);
	// ros::Subscriber pf_encoderZ_sub = n.subscribe("/scan_sewing_ssm/pattern_former_motor_pos", 10, motorPosCallback);
	ros::Subscriber scan_complete_sub = n.subscribe("/scan_sewing_gui/scan_complete", 1, scanCallback);
	ros::Subscriber topStitch_sub = n.subscribe("/scan_sewing_gui/stitch", 1, topStitchCallback);
	
	// Zeroing Test (XXX: NEED TO DELETE)
	ros::Subscriber next_sub = n.subscribe("/next", 1, nextCallback);

	// Initialize variables and objects.
	mini = new StepperMotor(comPort, baudrate);
	patternFormer = new PatternFormer;

	int res;
	// Motor Setup
	for (int i = 2; i < 5; i++) {
		// Motor Connection Check
		if (mini->checkStatus(i) == 0) {
			cout << "ERROR - main - motor_" << i << " - No connection" << endl;
			return 1;
		}
		// Set motor parameter
		mini->setMotorParameter(i, PPR, MaxSpeed, StartSpeed, AccTime, DecTime, OrgSearchSpeed, OrgSearchMethod);
	}

	while(nextFlag) continue;
	nextFlag = true;
	// Origin Adjustment
		// Origin Search
	if (mini->limitPosSearch() == -1) {
			cout << "ERROR - MAIN - Motor Limit Search" << endl;
			/*return 1;*/
		};
	cout << "MAIN - Success - Motor Limit Search" << endl;
		// Reference Pos. Search
	if (mini->moveAxis_mm(refPosX, refPosY, tempPeriod) != 1) {
		cout << "ERROR - MAIN - Move to Ref Position" << endl;
	}
	isMotioning();
	
	cout << "MAIN - Success - Motor Reference Search" << endl;
	cout << "*************Ready to Reset Pattern Former****************" << endl;
	while(nextFlag) continue;
	nextFlag = false;

	// Init thread
	thread t_ros(rosRun);
	thread t_readIRSensor(readIRSensor);
	bool loadPath = false;
	// ------------------------ Ready to Run --------------------------------------- //
	while (!stopsign) {
		if (!patternFormer->scanComplete){
			continue;
			Sleep(1);
			loadPath = false;
		}
		
		if(!loadPath){
			cout << "MAIN - Success - Trajectory loaded" << endl;
			loadPatternformerPath();
			// loadExagPath();
			loadPath = true;
		}

		// Get Mini Trajectory
		patternFormer->needleCnt = 0;
		if(!patternFormer->topStitch){
			Sleep(1);
			continue;
		}
		cout << "MAIN - Receive top stitch trigger" << endl;
		cout << "MAIN - Befor stitch - needleCnt = " << patternFormer->needleCnt << ", nPath = " << nPath << endl;
		patternFormer->topStitch = false;
		patternFormer->scanComplete = false;
		if (patternFormer->needleCnt == 0) {
			Sleep(1000);
		}
		// ---------------------------------- //
		// Check availablity of trajectory.
		// ---------------------------------- //

		// ---------------- Start to sew ------------------------------ //
		// Move to Start Pos.

		// res = mini->moveAxis_mm(xPath.at(0), yPath.at(0), tempPeriod);
		// if (res != 1) {
		// 	cout << "ERROR - MAIN - Move - initial position" << endl;
		// 	return 1;
		// }
		// isMotioning();
		
		while (!stopsign && (patternFormer->needleCnt < nPath - 2)) {
			patternFormer->dummy = 0;
			// Move to next position
			if (stitch) {
				patternFormer->dummy = 1;
				continue;
			}
			if (needleSafety) {
				stitch = true;
				res = mini->moveAxis_mm(xPath.at(patternFormer->needleCnt), yPath.at(patternFormer->needleCnt), tempPeriod);
				cout << "Path : " << xPath.at(patternFormer->needleCnt) << ", " 
					 << yPath.at(patternFormer->needleCnt)  
					 << "  needleCnt: " << patternFormer->needleCnt << " / " <<  nPath<<endl;

				if (res != 1) {
					cout << "ERROR - MAIN - Move - %dth position" << endl, patternFormer->needleCnt;
					return 1;
				}
				isMotioning();
			}

			// End Check
			if (patternFormer->needleCnt >= nPath - 2) {
				cout << "End Trajectory" << endl;
				patternFormer->ready = false;
				patternFormer->scanComplete = false;
				// Move to Reference Position
				if (mini->moveAxis_mm(refPosX, refPosY, tempPeriod) != 1) {
					cout << "ERROR - MAIN - Failed to return to Ref Position" << endl;
				}
				isMotioning();
				break;
			}
		}
	}
	// Move to Reference Position
	if (mini->moveAxis_mm(refPosX, refPosY, tempPeriod) != 1) {
		cout << "ERROR - MAIN - Failed to return to Ref Position" << endl;
	}
	isMotioning();
	cout << "MAIN - Success - Return to Ref Position" << endl;
	t_ros.join();
	t_readIRSensor.join();
	cout << patternFormer->needleCnt << endl;
	return 1;


}

