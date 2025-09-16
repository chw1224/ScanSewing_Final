/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date January 2022
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <QMessageBox>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <QPushButton>
#include <QPixmap>
#include <QTransform>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <QString>
#include "../include/scan_sewing_gui/main_window.hpp"
#endif
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace scan_sewing_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  qnode.init();
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  // Update Status
  QObject::connect(&qnode, SIGNAL(imageUpdated()), this, SLOT(updateImage()));
  QObject::connect(&qnode, SIGNAL(visionErrorUpdated()), this, SLOT(updateVisionError()));
  QObject::connect(&qnode, SIGNAL(pfStatusUpdated()), this, SLOT(updatePfStatus()));
  QObject::connect(&qnode, SIGNAL(miniStateUpdated()), this, SLOT(updateMiniState()));
  QObject::connect(&qnode, SIGNAL(miniNeedleUpdated()), this, SLOT(updateMiniNeedle()));
  QObject::connect(&qnode, SIGNAL(miniMotorPosUpdated()), this, SLOT(updateMiniMotorPos()));

  // Button Click
  QObject::connect(ui.stop_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.press_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.start_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.light_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.calibration_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.refresh_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.detection_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.topstitching_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  QObject::connect(ui.param_set_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  ui.template_shape_cbox->setCurrentIndex(0);


}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright ? Robot</p><p>This package needs an about description.</p>"));
}

void MainWindow::buttonClicked() {
    QPushButton* button = (QPushButton*)sender();
    std::string state;
    state = button->objectName().toStdString();
    if(state=="stop_button") {

    } else if(state=="press_button") {

    } else if(state=="start_button") {

    } else if(state=="light_button") {

    } else if(state=="calibration_button") {
        ui.state_text->setText("Start Calibration...");
        qnode.calibration_send_transition(ui.pf_x_dsbox->value(),ui.pf_y_dsbox->value(),
                                            ui.temp_x_dsbox->value(),ui.temp_y_dsbox->value());
    } else if(state=="refresh_button") {
        ui.topstitching_button->setEnabled(false);
        ui.state_text->setText("Restarting...\n\
* New Calibration -> Place the chessboard in the center of the screen and Press Calibration button\n\
* SeamLine Detection-> Press SeamLine Detection button");
    } else if(state=="detection_button") {
        ui.state_text->setText("Start SeamLine Detection...");
    } else if(state=="topstitching_button") {

    } else if(state=="param_set_button") {
        if(exposure_time != ui.exposure_time_sbox->value()) {
            exposure_time = ui.exposure_time_sbox->value();
            qnode.exposure_time_send_transition(exposure_time);
        }
        if(offset != ui.offset_dsbox->value()) {
            offset = ui.offset_dsbox->value();
            qnode.offset_send_transition(offset);
        }
        if(template_shape != ui.template_shape_cbox->currentIndex()){
            template_shape = ui.template_shape_cbox->currentIndex();
            qnode.template_shape_send_transition(template_shape);
        }
        ui.detection_button->setEnabled(true);
        ui.topstitching_button->setEnabled(true);
        ui.state_text->setText("Setting Finished...");
    }
    qnode.send_transition(state);
}

void MainWindow::updateImage() {

    int small_frame_width = ui.img1_frame->width();
    int small_frame_height = ui.img1_frame->height();
    int large_frame_width = ui.camera_frame->width();
    int large_frame_height = ui.camera_frame->height();

    if (!qnode.getRtImage()){
        char result = qnode.getImageNum();
        QImage* qimg_ = new QImage();
        if (result=='0') {
            qimg_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_rt.bmp");
        } else if (result=='1') {    // calibration 캡처
            ui.state_text->setText("Calibration Finished...\n\
* SeamLine Detection-> Press SeamLine Detection button\n\
* Refresh -> Press Refresh button");
            qimg_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_calibration.bmp");
        } else if (result=='2') {   // ui_image1 캡처
            qimg_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_image1.bmp");
            QImage* qimg_stitch_ = new QImage();
            qimg_stitch_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_image1.bmp");
            QPixmap qpix_stitch_ = QPixmap::fromImage(qimg_stitch_->scaled(small_frame_width,small_frame_height,Qt::IgnoreAspectRatio));
            qpix_stitch_ = qpix_stitch_.transformed(QTransform().scale(-1, 1));
            ui.img1_frame->setPixmap(qpix_stitch_);
            delete qimg_stitch_;
        } else if (result=='3') {    // ui_image2 캡처
            qimg_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_image2.bmp");
            QImage* qimg_stitch_ = new QImage();
            qimg_stitch_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_image2.bmp");
            QPixmap qpix_stitch_ = QPixmap::fromImage(qimg_stitch_->scaled(small_frame_width,small_frame_height,Qt::IgnoreAspectRatio));
            qpix_stitch_ = qpix_stitch_.transformed(QTransform().scale(-1, 1));
            ui.img2_frame->setPixmap(qpix_stitch_);
            delete qimg_stitch_;
        } else if (result=='4') {    // result
            ui.topstitching_button->setEnabled(true);
            ui.state_text->setText("Seam Line Detection Finished...\n\
* trying -> Press TopStitching button\n\
* Restart -> Press Restart button");
            qimg_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_image_tot.bmp");
            QImage* qimg_result_ = new QImage();
            qimg_result_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_image_tot.bmp");
            QPixmap qpix_result_ = QPixmap::fromImage(qimg_result_->scaled(small_frame_width,small_frame_height,Qt::IgnoreAspectRatio));
            qpix_result_ = qpix_result_.transformed(QTransform().scale(-1, 1));
            ui.zoom_frame->setPixmap(qpix_result_);
        }
        QPixmap qpix_ = QPixmap::fromImage(qimg_->scaled(large_frame_width,large_frame_height,Qt::IgnoreAspectRatio));
        qpix_ = qpix_.transformed(QTransform().scale(-1, 1));
        ui.camera_frame->setPixmap(qpix_);
        delete qimg_;
    } else {
        QImage* qimg_rt_ = new QImage();
        qimg_rt_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_rt.bmp");
        QPixmap qpix_rt_ = QPixmap::fromImage(qimg_rt_->scaled(large_frame_width,large_frame_height,Qt::IgnoreAspectRatio));
        qpix_rt_ = qpix_rt_.transformed(QTransform().scale(-1, 1));
        ui.camera_frame->setPixmap(qpix_rt_);
        delete qimg_rt_;

        QImage* qimg_empty_ = new QImage();
        qimg_empty_->load("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/ui_no_image.png");
        QPixmap qpix_empty_ = QPixmap::fromImage(qimg_empty_->scaled(small_frame_width,small_frame_height,Qt::IgnoreAspectRatio));
        ui.img1_frame->setPixmap(qpix_empty_);
        ui.img2_frame->setPixmap(qpix_empty_);
        ui.zoom_frame->setPixmap(qpix_empty_);
        delete qimg_empty_;
    }
}

void MainWindow::updateVisionError() {
    char vision_error = qnode.getVisionError();
    if(vision_error=='0') {
        // No Calibration Data
        ui.detection_button->setEnabled(false);
        ui.state_text->setText("* No Calibration Data -> Place the chessboard and Press Calibration button");
        ui.detection_button->setEnabled(false);
    } else if(vision_error=='1') {
        // Calibration Failed
        ui.state_text->setText("Error : Calibraiton Failed. Replace the chessboard and Press Calibration button Again");
        ui.detection_button->setEnabled(false);
    } else if(vision_error=='2') {
        // No Camera Connection
        ui.state_text->setText("Error : Camera is not Connected!");
    } else if(vision_error=='3') {
        //Seam Line Detection Failed
        ui.state_text->setText("Error : SeamLine Detection Failed. Press SeamLine Detection Button Again");
    }
}

void MainWindow::updatePfStatus() {
    bool pf_foot = qnode.getPfFoot();
    bool pf_frame = qnode.getPfFrame();
    bool pf_loose = qnode.getPfLoose();
    bool pf_reset = qnode.getPfReset();
    bool pf_breakline = qnode.getPfBreakline();
    ui.foot_lcd->display(int(pf_foot));
    ui.frame_lcd->display(int(pf_frame));
    ui.loose_lcd->display(int(pf_loose));
    ui.reset_lcd->display(int(pf_reset));
    ui.breakline_lcd->display(int(pf_breakline));
}

void MainWindow::updateMiniState() {
    int mini_state = qnode.getMiniState();
    int mini_needle = qnode.getMiniNeedle();

    if (mini_state==0) {
        ui.mini_status->setText("ERROR - Motor Limit Search");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: red;");
    } else if (mini_state==1) {
        ui.mini_status->setText("Success - Motor Limit Search");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    } else if (mini_state==2) {
        ui.mini_status->setText("Success - Motor Reference Search");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    } else if (mini_state==3) {
        ui.mini_status->setText("Success - Trajectory loaded");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    } else if (mini_state==4) {
        ui.mini_status->setText("Success - Receive top stitch trigger");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    } else if (mini_state==5 || mini_state==8) {
        ui.mini_status->setText(QString("ERROR - Move - %1th position").arg(mini_needle));
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: red;");
    } else if (mini_state==6) {
        ui.mini_status->setText("Success - End Trajectory");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    } else if (mini_state==7 || mini_state==9) {
        ui.mini_status->setText("ERROR - Failed to return to Ref Position");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: red;");
    } else if (mini_state==10) {
        ui.mini_status->setText("Success - Return to Ref Position");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    } else if (mini_state==11) {
        ui.mini_x_status->setText("FAILED");
        ui.mini_x_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: red;");
    } else if (mini_state==12) {
        ui.mini_y_status->setText("Y1 FAILED");
        ui.mini_y_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: red;");
    } else if (mini_state==13) {
        ui.mini_y_status->setText("Y2 FAILED");
        ui.mini_y_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: red;");
    } else if (mini_state==14) {
        ui.mini_x_status->setText("READY!");
        ui.mini_x_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
        ui.mini_y_status->setText("READY!");
        ui.mini_y_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    } else if (mini_state==15) {
        ui.mini_status->setText("ERROR - MOTOR - Limit Pos Search");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: red;");
    } else if (mini_state==16) {
        ui.mini_status->setText("Success - Limit Search End");
        ui.mini_status->setStyleSheet("border-color: white;\
                                      border-width : 1.2px;\
                                      border-style: outset;\
                                      color: green;");
    }



}

void MainWindow::updateMiniNeedle() {
    int mini_path_length = qnode.getMiniPathLength();
    int mini_needle = qnode.getMiniNeedle();
    ui.mini_progress->setMaximum(mini_path_length);
    ui.mini_progress->setValue(mini_needle);
    QString progressText = QString("%1/%2").arg(mini_needle).arg(mini_path_length);
    ui.mini_progress->setFormat(progressText);

}

void MainWindow::updateMiniMotorPos() {
    double* mini_motor_pos = qnode.getMiniMotorPos();
    ui.target_x_lcd->display(mini_motor_pos[0]);
    ui.target_y_lcd->display(mini_motor_pos[1]);
    ui.command_x_lcd->display(mini_motor_pos[2]);
    ui.command_y_lcd->display(mini_motor_pos[3]);
    ui.path_x_lcd->display(mini_motor_pos[4]);
    ui.path_y_lcd->display(mini_motor_pos[5]);
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "scan_sewing_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "scan_sewing_gui");
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  qnode.rosShutdownTrigger();
  QMainWindow::closeEvent(event);
}

}  // namespace scan_sewing_gui

