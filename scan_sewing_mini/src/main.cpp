#include "main.hpp"

#define MAX_DATA_LENGTH 255

ofstream motor_pos_out("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\file.csv");

// SerialPort patternFormerSensingBoard(comPortSensingBoard, 115200, 0.01);
SerialPort needleSensingBoard(3, 921600, 0.01);

vector<string> split(string str, char Delimiter) {
	istringstream iss(str);             
	string buffer;                      

	vector<string> result;

	while (getline(iss, buffer, Delimiter)) {
		result.push_back(buffer); 
	}

	return result;
}

void scanCallback(const std_msgs::CharConstPtr& msg){
	if (msg->data == 52)
		patternFormer->scanComplete = 1;
	return;
}

void topStitchCallback(const std_msgs::BoolConstPtr& msg){
	patternFormer->topStitch = msg->data;
	return;
}

void pfBreaklineCallback(const std_msgs::BoolConstPtr& msg) {
	bool pf_breakline = msg->data;
	if (pf_breakline) {
		scanComplete++;
		nextFlag = false;
		return;
	}
}


void nextCallback(const std_msgs::Int32ConstPtr& msg){
	int d = msg->data;
	//posDiff = (double)d / 1000;
	cout << "Next Msg Received" << endl;
	nextFlag = false;
	//stopsign = true;
}

void pathNumCallback(const std_msgs::Int32ConstPtr& msg) {
	pathNum = msg->data;
}


void xpCallback(const std_msgs::Float32ConstPtr& msg) {
	incre[0] = msg->data;
	mini->setValueCompensation(incre[0], incre[1], incre[2], incre[3]);
}
void xnCallback(const std_msgs::Float32ConstPtr& msg) {
	incre[1] = msg->data;
	mini->setValueCompensation(incre[0], incre[1], incre[2], incre[3]);
}
void ypCallback(const std_msgs::Float32ConstPtr& msg) {
	incre[2] = msg->data;
	mini->setValueCompensation(incre[0], incre[1], incre[2], incre[3]);
}
void ynCallback(const std_msgs::Float32ConstPtr& msg) {
	incre[3] = msg->data;
	mini->setValueCompensation(incre[0], incre[1], incre[2], incre[3]);
}



void loadExagPath() {
	nPath = 106;
	for (unsigned int i = 0; i < nPath; i++) {
		xPath.push_back(-5.0);
		//yPath.push_back(-5.0);
		yPath.push_back(-5.0 - 0.1 * (i % 2)); 
	}
}

void loadPatternformerPath() {
	ofstream pathFile("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\pathFile.txt");
	ifstream file("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_ssm\\cmdline\\SL_Final.txt");
	ifstream p0("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\preset_path_00.txt");
	ifstream p1("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\preset_path_01.txt");
	ifstream p2("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\preset_path_02.txt");
	string line;
	float rx, ry;
	float sx, sy;
	float ax0, ay0;
	float delx, dely;
	float offX, offY;
	vector<float> presetX, presetY;
	offX = 0;
	offY = 0;

	sx = 760;
	sy = 30;

	ax0 = 155;
	ay0 = -255.5;
	nPath = 0;
	// cout << "path Num " << pathNum << endl;
	if (pathNum == 0 || pathNum == 3) {
		while (p0) {
			getline(p0, line);
			vector<string> result0 = split(line, ' ');
			presetX.push_back(stof(result0[0]));
			presetY.push_back(stof(result0[1]));
		}
		p0.close();
	}
	else if (pathNum == 1) {
		while (p1) {
			getline(p1, line);
			vector<string> result1 = split(line, ' ');
			presetX.push_back(stof(result1[0]));
			presetY.push_back(stof(result1[1]));
		}
		p1.close();
	}
	else if (pathNum == 2) {
		while (p2) {
			getline(p2, line);
			vector<string> result = split(line, ' ');
			presetX.push_back(stof(result[0]));
			presetY.push_back(stof(result[1]));
		}
		p2.close();
	}
	else {
		cout << "Path num error" << endl;
		p0.close();
		p1.close();
		p2.close();
		return;
	}



	while (file) {
		getline(file, line);
		vector<string> result = split(line, ' ');
		if (result.size() == 2) {
			result[0].erase(0, 1);
			result[1].erase(0, 1);
			rx = stof(result[0]);
			ry = stof(result[1]);
			delx = -(sx - rx) + presetX.at(nPath) - 5;
			dely = (sy - ry) + presetY.at(nPath) - 5;

			pathFile << delx << " " << dely << std::endl;
			if (nPath == 0) {
				offX = -delx - 0;
				offY = -dely - 0;
			}
			if (delx < -9.5) {
				delx = -9.5;
			}
			else if (delx > -0.5) {
				delx = -0.5;
			}
			if (dely < -9.5) {
				dely = -9.5;
			}
			else if (dely > -0.5) {
				dely = -0.5;
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
// cout << "Start IR Sensor Reading" << endl;
	while(!stopsign){
		Sleep(1);
		int nread = needleSensingBoard.readline((uint8_t*)buffer, MAX_DATA_LENGTH);
		if(nread > 0) buffer[nread-2] = 0;
			sscanf(buffer, "%d", &irState);

			// Needle safety decision logic
			if (nread == 3){
				needleSafety = false;
				// cout << "IR State 1" << endl;
			}
			if (nread == 4 && !needleSafety){
				// cout << "IR State 2" << endl;
				patternFormer->needleCnt++;
				stitch = false;
				needleSafety = true;
			}
			///

			patternFormer->irState = irState;
		
	}
	return;
}

// void checkEmergency(){
// 	while(!stopsign){
// 		if (!needleSafety){
// 			if (mini->getMotionState(0) && mini->getMotionState(1)){
// 				mini->emergencyStop();
// 			}
// 		}
// 	}
		
// }
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
	mini->isMotioning(2);
	mini->isMotioning(3);
	mini->isMotioning(4);

	return;
}

double get_travel_error_compensation(double target) {
	double sig = target / abs(target);
	if (sig == 1.0) {
		if (abs(target) < 1.0) {
			target = (1 + coeff_travel_range0_pos) * target;
		}
		else if (abs(target) < 2.0) {
			target = (1 + coeff_travel_range1_pos) * target + sig * (coeff_travel_range0_pos - coeff_travel_range1_pos);
		}
		else if (abs(target) < 3.0) {
			target = (1 + coeff_travel_range2_pos) * target + sig * (coeff_travel_range0_pos + coeff_travel_range1_pos - 2 * coeff_travel_range2_pos);
		}
		else {
			target = (1 + coeff_travel_range3_pos) * target + sig * (coeff_travel_range0_pos + coeff_travel_range1_pos + coeff_travel_range2_pos - 3 * coeff_travel_range3_pos);
		}
	}
	else {
		if (abs(target) < 1.0) {
			target = (1 + coeff_travel_range0_neg) * target;
		}
		else if (abs(target) < 2.0) {
			target = (1 + coeff_travel_range1_neg) * target + sig * (coeff_travel_range0_neg - coeff_travel_range1_neg);
		}
		else if (abs(target) < 3.0) {
			target = (1 + coeff_travel_range2_neg) * target + sig * (coeff_travel_range0_neg + coeff_travel_range1_neg - 2 * coeff_travel_range2_neg);
		}
		else {
			target = (1 + coeff_travel_range3_neg) * target + sig * (coeff_travel_range0_neg + coeff_travel_range1_neg + coeff_travel_range2_neg - 3 * coeff_travel_range3_neg);
		}
	}
	
	return target;
}

double get_travel_error_compensation_X(double target) {
	double sig = target / abs(target);
	if (sig == 1.0) {
		if (abs(target) < 1.0) {
			target = (1 + coeff_travel_range0_posX) * target;
		}
		else if (abs(target) < 2.0) {
			target = (1 + coeff_travel_range1_posX) * target + sig * (coeff_travel_range0_posX - coeff_travel_range1_posX);
		}
		else if (abs(target) < 3.0) {
			target = (1 + coeff_travel_range2_posX) * target + sig * (coeff_travel_range0_posX + coeff_travel_range1_posX - 2 * coeff_travel_range2_posX);
		}
		else {
			target = (1 + coeff_travel_range3_posX) * target + sig * (coeff_travel_range0_posX + coeff_travel_range1_posX + coeff_travel_range2_posX - 3 * coeff_travel_range3_posX);
		}
	}
	else {
		if (abs(target) < 1.0) {
			target = (1 + coeff_travel_range0_negX) * target;
		}
		else if (abs(target) < 2.0) {
			target = (1 + coeff_travel_range1_negX) * target + sig * (coeff_travel_range0_negX - coeff_travel_range1_negX);
		}
		else if (abs(target) < 3.0) {
			target = (1 + coeff_travel_range2_negX) * target + sig * (coeff_travel_range0_negX + coeff_travel_range1_negX - 2 * coeff_travel_range2_negX);
		}
		else {
			target = (1 + coeff_travel_range3_negX) * target + sig * (coeff_travel_range0_negX + coeff_travel_range1_negX + coeff_travel_range2_negX - 3 * coeff_travel_range3_negX);
		}
	}

	return target;
}



double get_step_distance_error_compensation(double target) {
	double sig = target / abs(target);
	double del[5];
	del[0] = 0.0;
	del[1] = 0.05 * (coeff_dist_range0 - coeff_dist_range1);
	del[2] = del[1] + (coeff_dist_range1 - coeff_dist_range2);
	del[3] = del[2] + (coeff_dist_range2 - coeff_dist_range3) * 2.0;
	del[4] = del[3] + (coeff_dist_range3 - coeff_dist_range4) * 3.0;
	if (abs(target) < 0.05) {
		target = coeff_dist_range0 * target + del[0];
	}
	else if (abs(target) < 1.0) {
		target = coeff_dist_range1 * target + sig * del[1];
	}
	else if (abs(target) < 2.0) {
		target = coeff_dist_range2 * target + sig * del[2];
	}
	else if (abs(target) < 3.0) {
		target = coeff_dist_range3 * target + sig * del[3];
	}
	else {
		target = coeff_dist_range4 * target + sig * del[4];
	}

	return target;
}


int main(int argc, char **argv) {
	ofstream randomPathFile("C:\\catkin_ws\\src\\ScanSewing\\scan_sewing_mini\\pathFile_241028_random_X_nph.txt");
	ros::init(argc, argv, "scan_sewing_mini");
	ros::NodeHandle n;
	ros::Publisher mini_state_pub = n.advertise<std_msgs::Int32>("/scan_sewing_mini/mini_state", 10);
	ros::Publisher mini_needle_pub = n.advertise<std_msgs::Int32>("/scan_sewing_mini/mini_needle", 10);
	ros::Publisher mini_motor_pos_pub = n.advertise<geometry_msgs::Point>("/scan_sewing_mini/mini_motor_pos", 10);
	



	// ros::Subscriber pf_encoderZ_sub = n.subscribe("/scan_sewing_ssm/pattern_former_motor_pos", 10, motorPosCallback);
	ros::Subscriber scan_complete_sub = n.subscribe("vision_image", 1, scanCallback);
	ros::Subscriber topStitch_sub = n.subscribe("/scan_sewing_gui/stitch", 1, topStitchCallback);
	ros::Subscriber incre_xp = n.subscribe("incre_xp", 1, xpCallback);
	ros::Subscriber incre_xn = n.subscribe("incre_xn", 1, xnCallback);
	ros::Subscriber incre_yp = n.subscribe("incre_yp", 1, ypCallback);
	ros::Subscriber incre_yn = n.subscribe("incre_yn", 1, ynCallback);
	ros::Subscriber pf_loc_sub = n.subscribe("/scan_sewing_pf/pf_breakline", 1, pfBreaklineCallback);
	ros::Subscriber path_num_sub = n.subscribe("/scan_sewing_gui/path_num", 1, pathNumCallback);
	// Zeroing Test (XXX: NEED TO DELETE)
	ros::Subscriber next_sub = n.subscribe("next", 1, nextCallback);

	// Initialize variables and objects.
	mini = new StepperMotor(mini_state_pub, mini_motor_pos_pub);
	patternFormer = new PatternFormer;
	double* p;

	// 25 25 29 29
	incre[0] = 20;
	incre[1] = 20;
	incre[2] = 20;
	incre[3] = 20;

	mini->setValueCompensation(20,20, 20, 20);

	thread t_ros(rosRun);
	int res;
	int cPath = 0;
	// Motor Setup
	for (int i = 2; i < 5; i++) {
		// Set motor parameter
		mini->setMotorParameter(i, PPR, MaxSpeed, StartSpeed, AccTime, DecTime, OrgSpeed, OrgSearchSpeed, OrgSearchMethod);

	}

	mini->servoEnable(true);

	nextFlag = true;
	Sleep(1000);
	// Origin Adjustment
		// Origin Search
	if (mini->limitPosSearchX() == -1) {
		std_msgs::Int32 msg;
		msg.data = 0;
		mini_state_pub.publish(msg);
			// cout << "ERROR - MAIN - Motor Limit Search" << endl;
			/*return 1;*/
		};

	
	if (mini->limitPosSearchY() == -1) {
	}
	std_msgs::Int32 msg;
	msg.data = 1;
	mini_state_pub.publish(msg);
	// cout << "MAIN - Success - Motor Limit Search" << endl;
	isMotioning();

	Sleep(1000);

	mini->moveAxis_mm(refPosX, 0, tempPeriod);
	isMotioning();
	msg.data = 2;
	mini_state_pub.publish(msg);
	//mini->moveAxis_mm(refPosX, refPosY, tempPeriod);
	//isMotioning();

	// cout << "MAIN - Success - Motor Reference Search" << endl;
	// cout << "*************Ready to Reset Pattern Former****************" << endl;

	/*while (nextFlag) {
		patternFormer->dummy = 1;
		continue;
	}

	nextFlag = true;*/


	double xp, yp;
	mode = 99;
	posDiff = 0.05;
	double del = -0.00;
	double del_x = 0.00;
	const int maxIncre = 91;
	////// mode 1 = X one-side
	//if (mode == 1){
	//	for (int i = 1; i < maxIncre; i++) {
	//		xp = refPosX - posDiff * i;
	//		yp = refPosY;
	//		mini->moveAxis_mm(xp, yp, tempPeriod);
	//		isMotioning();
	//		Sleep(1000);
	//	}
	//	for (int i = 1; i < maxIncre; i++) {
	//		xp = refPosX - posDiff * (maxIncre - i);
	//		yp = refPosY;
	//		mini->moveAxis_mm(xp, yp, tempPeriod);
	//		isMotioning();
	//		Sleep(1000);
	//	}
	//}
	//
	//// mode 2 = Y one-side
	//else if (mode == 2){
	//	for (int i = 1; i < maxIncre; i++) {
	//		xp = refPosX;
	//		yp = refPosY + posDiff * i;
	//		mini->moveAxis_mm(xp, yp, tempPeriod);
	//		isMotioning();
	//		Sleep(1000);
	//	}
	//	for (int i = 1; i < maxIncre; i++) {
	//		xp = refPosX;
	//		yp = refPosY + posDiff * (maxIncre - i);
	//		mini->moveAxis_mm(xp, yp, tempPeriod);
	//		isMotioning();
	//		Sleep(1000);
	//	}
	//}

	//// mode 3 = X both-side
	//else if (mode == 3){
	//	for (int i = 1; i < maxIncre; i++) {
	//		for (int j = 0; j < 3; j++){
	//			xp = refPosX + posDiff * i;
	//			yp = refPosY;
	//			mini->moveAxis_mm(xp, yp, tempPeriod);
	//			isMotioning();
	//			Sleep(1000);

	//			xp = refPosX - posDiff * i;
	//			yp = refPosY;
	//			mini->moveAxis_mm(xp, yp, tempPeriod);
	//			isMotioning();
	//			Sleep(1000);
	//		}
	//	}
	//}
	//else if (mode == 4){
	//	for (int i = 1; i < maxIncre; i++) {
	//		for (int j = 0; j < 1; j++){
	//			xp = refPosX + posDiff * i;
	//			yp = refPosY;
	//			mini->moveAxis_mm(xp, yp, tempPeriod);
	//			isMotioning();
	//			Sleep(1000);
	//			/*
	//			xp = refPosX;
	//			yp = refPosY;
	//			mini->moveAxis_mm(xp, yp, tempPeriod);
	//			isMotioning();
	//			Sleep(1000);
	//			*/
	//		}
	//	}
	//	for (int i = 1; i < maxIncre; i++) {
	//		for (int j = 0; j < 1; j++) {
	//			xp = refPosX + posDiff * (maxIncre - i);
	//			yp = refPosY;
	//			mini->moveAxis_mm(xp, yp, tempPeriod);
	//			isMotioning();
	//			Sleep(1000);
	//			/*
	//			xp = refPosX;
	//			yp = refPosY;
	//			mini->moveAxis_mm(xp, yp, tempPeriod);
	//			isMotioning();
	//			Sleep(1000);
	//			*/
	//		}
	//	}
	
	//}
	const double PI = 3.1415926;
	if (mode == 1) {
		// X reps
		for (int i = 0; i < 5; i++) {
			mini->moveAxis_mm(refPosX - 2.5, refPosY, tempPeriod);
			isMotioning();
			mini->moveAxis_mm(refPosX + 2.5, refPosY, tempPeriod);
			isMotioning();
		}
		mini->moveAxis_mm(refPosX, refPosY, tempPeriod);
		isMotioning();
		// Y reps
		for (int i = 0; i < 5; i++) {
			mini->moveAxis_mm(refPosX, refPosY - 2.5, tempPeriod);
			isMotioning();
			mini->moveAxis_mm(refPosX, refPosY + 2.5, tempPeriod);
			isMotioning();
		}
		//Box Reps
		for (int i = 0; i < 3; i++){
			mini->moveAxis_mm(refPosX - 2.5, refPosY + 2.5, tempPeriod);
			isMotioning();
			mini->moveAxis_mm(refPosX - 2.5, refPosY - 2.5, tempPeriod);
			isMotioning();
			mini->moveAxis_mm(refPosX + 2.5, refPosY - 2.5, tempPeriod);
			isMotioning();
			mini->moveAxis_mm(refPosX + 2.5, refPosY + 2.5, tempPeriod);
			isMotioning();
		}
		mini->moveAxis_mm(refPosX, refPosY, tempPeriod);
		isMotioning();
		// Circular Reps
		const int cycleNum = 100;
		for (int i = 0; i < 1; i++) {
			for (int j = 0; j < cycleNum; j++) {
				mini->moveAxis_mm(refPosX + 2.5 * cos(2 * PI * (double)j / cycleNum), refPosY + 2.5 * sin(2 * PI * (double)j / cycleNum), tempPeriod);
				isMotioning();
			}
		}
		mini->moveAxis_mm(refPosX, refPosY, tempPeriod);
		isMotioning();
	}
	else if (mode == 5) {
		for (int i = 0; i < 480; i++) {
			xp = refPosX;
			yp = refPosY + 0.01 * i;
			std::cout << i << "th target   " << xp << "   " << yp << std::endl;
			mini->moveAxis_mm(xp, yp, tempPeriod);
			isMotioning();
			Sleep(1000);
		}
		for (int i = 480; i >= 0; i--) {
			xp = refPosX;
			yp = refPosY + 0.01 * i;
			std::cout << "Reverse " << i << "th target   " << xp << "   " << yp << std::endl;
			mini->moveAxis_mm(xp, yp, tempPeriod);
			isMotioning();
			Sleep(1000);
		}
	}
	//else if (mode == 6) {
	//	for (int i = 0; i < 480; i++) {
	//		xp = refPosX;
	//		yp = refPosY + 0.01 * i;
	//		std::cout << i << "th target   " << xp << "   " << yp << std::endl;
	//		mini->moveAxis_mm(xp, yp, tempPeriod);
	//		isMotioning();
	//		Sleep(1000);
	//	}
	//}
	else if (mode == 6) {
		double xp_cor = refPosX;
		double yp_cor = refPosY;
		std::random_device rd;
		std::mt19937 mt(rd());
		std::uniform_int_distribution<> dist(0, 900);
		del = -0.005;

		Sleep(1000);
	
		for (int i = 0; i < 600; i++) {
			xp = (double)(dist(mt)) * 0.01 - 9.5;
			yp = refPosY;
			//xp_cor = get_travel_error_compensation_x(xp - refposx) + refposx;
			//yp_cor = get_travel_error_compensation(yp - refposy) + refposy;
			mini->moveAxis_mm(xp, yp, tempPeriod);
			mini->getPreCommand(p);
			for (int i = 0; i < 4; i++) {
				randomPathFile << p[i] << ",";
			}
			randomPathFile << std::endl;
			//randomPathFile << xp << ", " << yp << std::endl; // ", " << xp_cor << ", " << yp_cor << std::endl;

			std::cout << "current number : " << i << "  " <<p[0] << ", " << p[3] << std::endl;
			isMotioning();
			Sleep(1000);
		}
	}
	//else if (mode == 7) {
	//	double xp_cor = refPosX;
	//	double yp_cor = refPosY;

	//	for (int i = 0; i < 10; i++) {
	//		xp = refPosX - 4.0;
	//		yp = refPosY;
	//		xp_cor = get_travel_error_compensation_X(xp - refPosX) + refPosX;
	//		//yp_cor = get_travel_error_compensation(yp - refPosY) + refPosY;
	//		mini->moveAxis_mm(xp_cor, yp, tempPeriod);
	//		isMotioning();
	//		Sleep(2000);

	//		xp = refPosX;
	//		yp = refPosY;
	//		xp_cor = get_travel_error_compensation_X(xp - refPosX) + refPosX;
	//		//yp_cor = get_travel_error_compensation(yp - refPosY) + refPosY;
	//		mini->moveAxis_mm(xp_cor, yp, tempPeriod);
	//		isMotioning();
	//		Sleep(2000);
	//	}
	//}
	// 
	// Init thread
	thread t_readIRSensor(readIRSensor);
	bool loadPath = false;
	// ------------------------ Ready to Run --------------------------------------- //
	while (!stopsign) {
		if (!patternFormer->scanComplete){
			Sleep(1);
			loadPath = false;
			continue;
			
		}
		Sleep(1000);
		if(!loadPath){
			loadPatternformerPath();
			std_msgs::Int32 msg;
			msg.data = 3;
			mini_state_pub.publish(msg);
			// cout << "MAIN - Success - Trajectory loaded" << endl;
			loadPath = true;
		}

		// Get Mini Trajectory
		patternFormer->needleCnt = 0;
		if(!patternFormer->topStitch){
			Sleep(1);
			continue;
		}
		std_msgs::Int32 msg;
		msg.data = 4;
		mini_state_pub.publish(msg);
		int int_nPath = static_cast<int>(nPath);
		msg.data = int_nPath;
		mini_needle_pub.publish(msg);
		// cout << "MAIN - Receive top stitch trigger" << endl;
		// cout << "MAIN - Befor stitch - needleCnt = " << patternFormer->needleCnt << ", nPath = " << nPath << endl;
		patternFormer->topStitch = false;
		patternFormer->scanComplete = false;
		if (patternFormer->needleCnt == 0) {
			Sleep(1000);
		}
		xp = xPath.at(0);
		yp = yPath.at(0);

		res = mini->moveAxis_mm(get_travel_error_compensation_X(xp - refPosX) + refPosX, get_travel_error_compensation(yp - refPosY) + refPosY, tempPeriod);
		msg.data = patternFormer->needleCnt;
		mini_needle_pub.publish(msg);
		geometry_msgs::Point msg2;
		msg2.x = xPath.at(0);
		msg2.y = yPath.at(0);
		msg2.z = 2.0;
		mini_motor_pos_pub.publish(msg);
		/*
		cout << "path : " << xPath.at(0) << ", "
			<< yPath.at(0)
			<< "  needleCnt: " << patternFormer->needleCnt << " / " << nPath << endl;*/

		if (res != 1) {
			msg.data = 5;
			mini_state_pub.publish(msg);
			// cout << "ERROR - MAIN - Move - %dth position" << endl, patternFormer->needleCnt;
			return 1;
		}
		isMotioning();

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

		while (!stopsign) {
			patternFormer->dummy = 0;

			if (scanComplete == 0) {
				patternFormer->dummy = 2;
				continue;
			}
			if (scanComplete >= 3) {
				Sleep(500);
				msg.data = 6;
				mini_state_pub.publish(msg);
				// cout << "End Trajectory" << endl;
				patternFormer->ready = false;
				patternFormer->scanComplete = false;
				// Move to Reference Position
				if (mini->moveAxis_mm(refPosX, 0, tempPeriod) != 1) {
					msg.data = 7;
					mini_state_pub.publish(msg);
					// cout << "ERROR - MAIN - Failed to return to Ref Position" << endl;
				}
				isMotioning();
				scanComplete = 0;
				break;
			}
			// Move to next position
			if (stitch) {
				patternFormer->dummy = 1;
				continue;
			}

			// End Check
			

			if (needleSafety) {
				stitch = true;
				if (patternFormer->needleCnt < nPath) cPath = patternFormer->needleCnt;
				else cPath = nPath-1;
				xp = xPath.at(cPath);
				yp = yPath.at(cPath);
				cout << cPath << ", " << xp << ", " << yp;
				res = mini->moveAxis_mm(get_travel_error_compensation_X(xp - refPosX) + refPosX, get_travel_error_compensation(yp - refPosY) + refPosY, tempPeriod);
				msg.data = patternFormer->needleCnt;
				mini_needle_pub.publish(msg);
				geometry_msgs::Point msg2;
				msg2.x = xPath.at(0);
				msg2.y = yPath.at(0);
				msg2.z = 2.0;
				mini_motor_pos_pub.publish(msg2);
				
				if (res != 1) {
					msg.data = 8;
					mini_state_pub.publish(msg);
					// cout << "ERROR - MAIN - Move - %dth position" << endl, patternFormer->needleCnt;
					return 1;
				}
				isMotioning();
				cout << endl;
			}
		}
	}
	// Move to Reference Position
	if (mini->moveAxis_mm(refPosX, 0, tempPeriod) != 1) {
		msg.data = 9;
		mini_state_pub.publish(msg);
	}
	isMotioning();
	msg.data = 10;
	mini_state_pub.publish(msg);
	t_ros.join();
	t_readIRSensor.join();
	return 1;


}

