/**
 * @file /include/scan_sewing_gui/main_window.hpp
 *
 * @brief Qt based gui for scan_sewing_gui.
 *
 * @date January 2022
 **/
#ifndef scan_sewing_gui_MAIN_WINDOW_H
#define scan_sewing_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtWidgets/QMainWindow>
//#include <cv_bridge/cv_bridge.h>
#include "ui_main_window.h"
#include "qnode.hpp"
#endif
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace scan_sewing_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
        *******************************************/
        void on_actionAbout_triggered();


        void updateImage();
        void updateVisionError();
        void updatePfStatus();
        void updateMiniState();
        void updateMiniNeedle();
        void updateMiniMotorPos();

        void buttonClicked();

    /******************************************
    ** Manual connections
    *******************************************/

private:
	Ui::MainWindowDesign ui;
    QNode qnode;

    int exposure_time = 30000;
    double offset = 0.0;
    int template_shape = 0;
};

}  // namespace scan_sewing_gui

#endif // scan_sewing_gui_MAIN_WINDOW_H
