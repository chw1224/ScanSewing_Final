/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui. 
 *
 * @date January 2022
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets>
#include <QApplication>
#include "../include/scan_sewing_gui/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    scan_sewing_gui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
