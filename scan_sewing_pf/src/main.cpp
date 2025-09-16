#include "main.h"

#define MAX_DATA_LENGTH 255

SerialPort patternFormerSensingBoard(7, 115200, 0.01);


int main(int argc, char **argv) {
	PatternFormer* patternFormer = new PatternFormer;
	char buffer[MAX_DATA_LENGTH];
	int foot, frame, trim, loose, reset, breakLine, output;
	breakLine = 0;
	// bool pf_loc;
	// std_msgs::Bool pf_loc_msg;

	ros::init(argc, argv, "scan_sewing_pf");
	ros::NodeHandle n;
	// ros::Publisher pf_loc_pub = n.advertise<std_msgs::Bool>("/scan_sewing_pf/pf_loc", 10);

	ros::Publisher pf_foot_pub = n.advertise<std_msgs::Bool>("/scan_sewing_pf/pf_foot", 10);
	ros::Publisher pf_frame_pub = n.advertise<std_msgs::Bool>("/scan_sewing_pf/pf_frame", 10);
	ros::Publisher pf_loose_pub = n.advertise<std_msgs::Bool>("/scan_sewing_pf/pf_loose", 10);
	ros::Publisher pf_reset_pub = n.advertise<std_msgs::Bool>("/scan_sewing_pf/pf_reset", 10);
	ros::Publisher pf_breakline_pub = n.advertise<std_msgs::Bool>("/scan_sewing_pf/pf_breakline", 10);





	while(ros::ok()){
		int nread = patternFormerSensingBoard.readline((uint8_t*)buffer, MAX_DATA_LENGTH);
		if(nread>0)	buffer[nread-2]=0;
		
		sscanf(buffer, "<%d %d %d %d %d %d %d>", 
		&foot, &frame, &trim, &loose, &reset, &output, &breakLine);

		if (breakLine > 1) {
			breakLine = 1;
		}

		if (foot != patternFormer->foot) {
			std_msgs::Bool msg;
			msg.data = (bool)foot;
			pf_foot_pub.publish(msg);
		}
		if (frame != patternFormer->frame) {
			std_msgs::Bool msg;
			msg.data = (bool)frame;
			pf_frame_pub.publish(msg);
		}
		if (loose != patternFormer->loose) {
			std_msgs::Bool msg;
			msg.data = (bool)loose;
			pf_loose_pub.publish(msg);
		}
		if (reset != patternFormer->reset) {
			std_msgs::Bool msg;
			msg.data = (bool)reset;
			pf_reset_pub.publish(msg);
		}

		if (breakLine != patternFormer->breakLine) {
			std_msgs::Bool msg;
			msg.data = (bool)breakLine;
			pf_breakline_pub.publish(msg);
			// std::cout << breakLine << std::endl;
		}

		/*
		if (breakLine && !patternFormer->breakLine){
			// cout << breakLine << endl;
			pf_loc = (bool)breakLine;
			pf_loc_msg.data = pf_loc;

			pf_loc_pub.publish(pf_loc_msg);
		}
		*/

		patternFormer->foot = foot;
		patternFormer->frame = frame;
		patternFormer->loose = loose;
		patternFormer->reset = reset;
		patternFormer->breakLine = breakLine;


		ros::spinOnce();

	}
	return 0;

}



