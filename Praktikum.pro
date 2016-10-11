#-------------------------------------------------
#
# Project created by QtCreator 2016-10-11T16:23:02
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Praktikum
TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_highgui

LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_photo
LIBS += -lopencv_shape
LIBS += -lopencv_stitching
LIBS += -lopencv_superres
LIBS += -lopencv_video
LIBS += -lopencv_videoio
LIBS += -lopencv_videostab
LIBS += -lopencv_viz
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
#LIBS += -lopencv_contrib
#LIBS += -lopencv_legacy
LIBS += -lopencv_flann
#LIBS += -lopencv_nonfree


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui
