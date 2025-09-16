# ScanSewing

------------

## Description

ROS packages for Scan&Sewing SNU-Hojeon project.

## List

+ scan_sewing_gui
+ scan_sewing_launch
+ scan_sewing_led_control
+ scan_sewing_pf
+ scan_sewing_ssm
+ scan_sewing_vision

## What you need

+ qt_build package (only for Windows User)
+ rosserial-noetic-devel (Arduino-ROS)
+ scikit-learn-main (Vision Package)
+ PySpin package (Spinnaker Camera Python Package)

## How to run

### Scan and Sewing Machine Launch

    devel\setup.bat && roslaunch scan_sewing_launch scan_sewing_machine.launch

### Vision

    rosrun scan_sewing_vision scripts\vision_node.py

### LED CONTROL

    rosrun rosserial_python nodes\serial_node.py COM8

### GUI
    rosrun scan_sewing_gui scan_sewing_gui

### Scan and Sewing Machine

    rosrun scan_sewing_ssm scan_sewing_ssm

### Macro-mini Pattern Former

    rosrun scan_sewing_pf scan_sewing_pf

### GUI & Pattern Former

    roslaunch scan_sewing_launch scan_sewing_test.launch