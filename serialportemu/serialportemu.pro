#-------------------------------------------------
#
# Project created by QtCreator 2014-12-17T23:40:36
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SerialPortEmu
TEMPLATE = app

INCLUDEPATH += ../../lib/serialportdialog

SOURCES += main.cpp\
        mainwindow.cpp \
    ../../lib/serialportdialog/dialog_serialport.cpp

HEADERS  += mainwindow.h \
    dialog_serialport.h \
    ../../lib/serialportdialog/dialog_serialport.h

FORMS    += mainwindow.ui \
    ../../lib/serialportdialog/dialog_serialport.ui
