#include <ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Bool.h>

const int LED = 8;

const int stopTouch = 11;
const int pressTouch = 12;
const int startTouch = 13;

const int touchTime = 100;



// ROS
ros::NodeHandle nh;

void ledCallback(const std_msgs::Bool& msg) {
  bool led = msg.data;
  if (led == true) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}

void patternFormerStateCallback(const std_msgs::Char& msg) {
  char pattern_former_state = msg.data;
  if (pattern_former_state=='0') {
    TouchStopButton();
  } else if (pattern_former_state=='1'){
    TouchPressButton();
  } else if (pattern_former_state=='2') {
    TouchStartButton();
  } else if (pattern_former_state=='3') {
    TouchPressButton();
    delay(1000);
    TouchStartButton();
  } else if (pattern_former_state=='4') {
    TouchStopButton();
    delay(1000);
    TouchStartButton();
  }
}

ros::Subscriber<std_msgs::Char> pattern_former_state_sub("/scan_sewing_gui/pattern_former_state", &patternFormerStateCallback);
ros::Subscriber<std_msgs::Bool> led_sub("/scan_sewing_gui/led_state", &ledCallback);

void setup() {

  nh.initNode();
  nh.subscribe(led_sub);
  nh.subscribe(pattern_former_state_sub);
  pinMode(LED, OUTPUT);

  pinMode(stopTouch, OUTPUT);
  pinMode(pressTouch, OUTPUT);
  pinMode(startTouch, OUTPUT);
delay(10);
  digitalWrite(stopTouch, LOW);
  digitalWrite(pressTouch, LOW);
  digitalWrite(startTouch, LOW);
  delay(10);

}


void TouchStopButton() {
  digitalWrite(stopTouch, HIGH);
  delay(touchTime);
  digitalWrite(stopTouch, LOW);
}

void TouchPressButton() {
  digitalWrite(pressTouch, HIGH);
  delay(touchTime);
  digitalWrite(pressTouch, LOW);
}

void TouchStartButton() {
  digitalWrite(startTouch, HIGH);
  delay(touchTime);
  digitalWrite(startTouch, LOW);
}

void TurnOnLED() {
  digitalWrite(LED, HIGH);
}

void TurnOffLED() {
  digitalWrite(LED, LOW);
}


void loop() {
  nh.spinOnce();
  delay(1);
}
