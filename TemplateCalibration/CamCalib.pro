#-------------------------------------------------
#
# Project created by QtCreator 2014-11-07T14:36:50
# Calibration by Kathrin Goffart
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CamCalib
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
        camcalib.cpp

HEADERS  += camcalib.h

FORMS    += camcalib.ui

OTHER_FILES +=

#-lopencv_core249.dll -lopencv_imgproc249 -lopencv_highgui249
unix: LIBS += -lboost_system
#win32: LIBS += -L"C:/opencv/opencv2490_mingw32/lib/" -lopencv_core249.dll -lopencv_imgproc249.dll -lopencv_highgui249.dll

#win32: INCLUDEPATH += "C:/opencv/opencv2490_mingw32/include"
#win32: DEPENDPATH += "C:/opencv/opencv2490_mingw32/include"

# perhaps some unix stuff is missing
