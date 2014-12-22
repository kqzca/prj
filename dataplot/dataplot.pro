#-------------------------------------------------
#
# Project created by QtCreator 2014-12-21T20:24:28
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = dataplot
TEMPLATE = app

INCLUDEPATH += ../../lib/qcustomplot

SOURCES += main.cpp\
        mainwindow.cpp \
    ../../lib/qcustomplot/qcustomplot.cpp

HEADERS  += mainwindow.h \
    ../../lib/qcustomplot/qcustomplot.h

FORMS    += mainwindow.ui
