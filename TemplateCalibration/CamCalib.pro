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

CONFIG += link_pkgconfig
PKGCONFIG += opencv


SOURCES += main.cpp\
        camcalib.cpp \
    camera_calibration.cpp \
    stereo_calib.cpp \
    video_stream.cpp \
    visualfeatures.cpp

HEADERS  += camcalib.h \
    camera_calibration.h \
    video_stream.h \
    stereo_calib.h \
    visualfeatures.h

FORMS    += camcalib.ui

OTHER_FILES +=

#-lopencv_core249.dll -lopencv_imgproc249 -lopencv_highgui249
unix: LIBS += -lboost_system
#win32: LIBS += -L"C:/opencv/opencv2490_mingw32/lib/" -lopencv_core249.dll -lopencv_imgproc249.dll -lopencv_highgui249.dll

#win32: INCLUDEPATH += "C:/opencv/opencv2490_mingw32/include"
#win32: DEPENDPATH += "C:/opencv/opencv2490_mingw32/include"

# perhaps some unix stuff is missing
