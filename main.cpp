/*******************************************************************************
 * Main.cpp - Entry point for the quadrotot simulator
 * Main.cpp description:
 * This program first creates a GUI containing information about the project
 * developers. The GUI contains a button which launches the simulator.
 *
 ******************************************************************************/

#include <QApplication>
#include <QGuiApplication>
#include <QMediaPlayer>
#include "launchwindow.h"
#include "simulatorWindow.h"
#include "MotorSoftware.h"

int main(int argc, char *argv[])
{
    // Initialize widget to display information and launch simulator
    QApplication launch_widget(argc, argv);

    // Play audio instructions for the pilot
    QMediaPlayer * ptrSound = new QMediaPlayer;
    ptrSound->setMedia(QUrl("qrc:/Music/StartUpMusic.mp3"));
    ptrSound->setVolume(10);
    ptrSound->play();

    // Create an instance of GUI window and display information
    LaunchWindow *ptrLaunchWindow = new LaunchWindow();
    ptrLaunchWindow->setWindowTitle(QString::fromUtf8("Quadrotor Flight Simulator"));
    ptrLaunchWindow->showMaximized();

    launch_widget.exec();

    // Stop playing audio instructions if still playing
    if (ptrSound->state() == QMediaPlayer::PlayingState)
        ptrSound->stop();

    // Initialize simulator to display quadrotor in 3D
    QGuiApplication simulator(argc, argv);

    // Set OpenGL Version information. Note: This format must be set before show() is called.
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setVersion(4,5);

    //initialize quadrotor parameters
    double dMass = 0.5, dInertia = 1.0, dArmLength = 1.0;
    double initialize_d[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    state_type in_state(initialize_d, initialize_d+12);

    // Setup the simulator window
    simulatorWindow window(dMass, dInertia, dArmLength, in_state);
    window.setFormat(format);
    window.setTitle(QString::fromUtf8("Quadrotor Flight Simulator"));
    window.show();
    window.showMaximized();

    return simulator.exec();
}
