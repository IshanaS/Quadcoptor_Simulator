QT += core gui \
      multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = Final_Project
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    vertex.cpp \
    Transform3D.cpp \
    input.cpp \
    camera3d.cpp \
    launchwindow.cpp \
    simulatorWindow.cpp \
    Battery.cpp \
    Motor.cpp \
    MotorSoftware.cpp \
    quadcoptor.cpp

HEADERS += \
    vertex.h \
    Transform3D.h \
    input.h \
    camera3d.h \
    launchwindow.h \
    simulatorWindow.h \
    Battery.h \
    Motor.h \
    MotorSoftware.h \
    quadcoptor.h

RESOURCES += \
    resources.qrc

FORMS += \
    launchwindow.ui

QMAKE_CFLAGS += -std=c++11 -stdlib=libc++ -mmacosx-version-min=10.8

QMAKE_CXXFLAGS += -std=c++11 -stdlib=libc++ -mmacosx-version-min=10.8

LIBS += -L"/usr/local/Cellar/boost/1.62.0/lib"

INCLUDEPATH += "/usr/local/Cellar/boost/1.62.0/include"
