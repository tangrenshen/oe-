#-------------------------------------------------#
# Project created by QtCreator 2022-12-23T20:39:43 #
#-------------------------------------------------#
QT       += core gui sql concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = xjp
TEMPLATE = app
CONFIG += console

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
    csvreader.cpp \
    flow_field.cpp \
    huiyu.cpp \
    main.cpp \
    mainwindow.cpp \
    qcustomplot.cpp \
    ship.cpp \
    trajectory_predictor.cpp \
    baseline_controller.cpp \
    mmg_model.cpp \
    mmg_kvlcc2.cpp \
    batch_runner.cpp

HEADERS += \
    csvreader.h \
    flow_field.h \
    huiyu.h \
    mainwindow.h \
    qcustomplot.h \
    ship.h \
    trajectory_predictor.h \
    baseline_controller.h \
    mmg_model.h \
    mmg_kvlcc2.h

FORMS += \
    mainwindow.ui

# Additional includes and configurations
INCLUDEPATH += ./

# Add resources if any
# RESOURCES += \
#     resources.qrc

# Enable C++11
CONFIG += c++11

# Disable warnings for unused variables and parameters
QMAKE_CXXFLAGS += -Wno-unused-variable -Wno-unused-parameter

# Debug and release configurations
CONFIG(debug, debug|release): {
    TARGET = xjp_debug
}

CONFIG(release, debug|release): {
    TARGET = xjp_release
}
