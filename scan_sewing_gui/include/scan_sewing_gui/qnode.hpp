/**
 * @file /include/scan_sewing_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date January 2022
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef scan_sewing_gui_QNODE_HPP_
#define scan_sewing_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <QThread>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace scan_sewing_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  void init_nh();
  void run();

  void send_transition(std::string state);
  void exposure_time_send_transition(int exposure_time);
  void offset_send_transition(double offset);
  void template_shape_send_transition(int template_shape);
  void calibration_send_transition(double pf_x, double pf_y, double temp_x, double temp_y);
  void light_control(bool tr);

  // Set functions

  // Get functions
  bool getRtImage() const {return rt_image;}
  char getImageNum() const {return image_num;}
  char getVisionError() const {return vision_error;}
  bool getPfFoot() const {return pf_foot;}
  bool getPfFrame() const {return pf_frame;}
  bool getPfLoose() const {return pf_loose;}
  bool getPfReset() const {return pf_reset;}
  bool getPfBreakline() const {return pf_breakline;}
  int getMiniState() const {return mini_state;}
  int getMiniPathLength() const {return mini_path_length;}
  int getMiniNeedle() const {return mini_needle;}
  double* getMiniMotorPos() const {return const_cast<double*>(mini_motor_pos);}


  // Other functions
  void rosShutdownTrigger();

  // Callback functions
  void visionImageCallback(const std_msgs::CharConstPtr& msg);
  void visionErrorCallback(const std_msgs::CharConstPtr& msg);
  void pfFootCallback(const std_msgs::BoolConstPtr& msg);
  void pfFrameCallback(const std_msgs::BoolConstPtr& msg);
  void pfLooseCallback(const std_msgs::BoolConstPtr& msg);
  void pfResetCallback(const std_msgs::BoolConstPtr& msg);
  void pfBreaklineCallback(const std_msgs::BoolConstPtr& msg);
  void miniStateCallback(const std_msgs::Int32ConstPtr& msg);
  void miniNeedleCallback(const std_msgs::Int32ConstPtr& msg);
  void miniMotorPosCallback(const geometry_msgs::PointConstPtr& msg);


Q_SIGNALS:
  void rosShutdown();
  void imageUpdated();
  void visionErrorUpdated();
  void pfStatusUpdated();
  void miniStateUpdated();
  void miniNeedleUpdated();
  void miniMotorPosUpdated();

private:
  int init_argc;
  char** init_argv;

  ros::NodeHandle* nh;

  // Publisher,
  ros::Publisher pattern_former_state_pub_;

  ros::Publisher vision_capture_num_pub_;
  ros::Publisher vision_capture_pub_;
  ros::Publisher vision_exposure_time_pub_;
  ros::Publisher vision_offset_pub_;
  ros::Publisher vision_image_processing_pub_;
  ros::Publisher vision_refresh_pub_;


  ros::Publisher vision_calibration_pub_;

  ros::Publisher macro_mini_trigger_pub_;
  ros::Publisher macro_mini_topstitch_pub_;

  ros::Publisher led_state_pub_;
  ros::Publisher path_num_pub_;

  ros::Publisher ros_shutdown_pub_;



  // Subscriber
  ros::Subscriber vision_image_sub_;
  ros::Subscriber vision_error_sub_;
  ros::Subscriber pf_foot_sub_;
  ros::Subscriber pf_frame_sub_;
  ros::Subscriber pf_loose_sub_;
  ros::Subscriber pf_reset_sub_;
  ros::Subscriber pf_breakline_sub_;
  ros::Subscriber mini_state_sub_;
  ros::Subscriber mini_needle_sub_;
  ros::Subscriber mini_motor_pos_sub_;


  // variables
  bool rt_image = true;
  char image_num='0'; // UI vision image status number
  char vision_error = '0';
  bool light_status = false;
  int template_shape_ = 0;
  bool pf_foot = false;
  bool pf_frame = false;
  bool pf_loose = false;
  bool pf_reset = false;
  bool pf_breakline = false;
  int mini_state = -1;
  int mini_path_length = 0;
  int mini_needle = -1;
  double mini_motor_pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


};

}  // namespace scan_sewing_gui

#endif /* scan_sewing_gui_QNODE_HPP_ */
