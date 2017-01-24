/* Description: launchwindow.cpp
 * This program creates GUI which is displayed first when the
 * simulator is launched.
 * Note: The GUI is created in design mode and hence this .cpp
 * doesn't contain much code. Edit in design mode to apply
 * changes.
*/

#include "launchwindow.h"
#include "ui_launchwindow.h"

LaunchWindow::LaunchWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LaunchWindow)
{
    ui->setupUi(this);
}

LaunchWindow::~LaunchWindow()
{
    delete ui;
}
