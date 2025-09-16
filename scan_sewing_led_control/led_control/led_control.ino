#include <ros.h>
#include <std_msgs/Bool.h>

// ROS
ros::NodeHandle nh;

void ledCallback(const std_msgs::Bool& msg) {
  bool led = msg.data;
  if (led == true) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, LOW);
  }
}

ros::Subscriber<std_msgs::Bool> led_sub("/scan_sewing_gui/led_state", &ledCallback);

void setup() {
  nh.initNode();
  nh.subscribe(led_sub);

  pinMode(8,OUTPUT);
  delay(10);

}

void loop() {
  nh.spinOnce();
  delay(1);
}
