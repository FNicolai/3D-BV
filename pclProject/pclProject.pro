#-------------------------------------------------
#
# Project created by QtCreator 2016-11-22T16:32:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pclProject
TEMPLATE = app

CONFIG += link_pkgconfig\
    c++11
PKGCONFIG += opencv\
    eigen3\
    pcl_2d-1.8\
    pcl_common-1.8\
    pcl_features-1.8\
    pcl_filters-1.8\
    pcl_geometry-1.8\
    pcl_io-1.8\
    pcl_kdtree-1.8\
    pcl_keypoints-1.8\
    pcl_ml-1.8\
    pcl_octree-1.8\
    pcl_outofcore-1.8\
    pcl_people-1.8\
    pcl_recognition-1.8\
    pcl_registration-1.8\
    pcl_sample_consensus-1.8\
    pcl_search-1.8\
    pcl_segmentation-1.8\
    pcl_stereo-1.8\
    pcl_surface-1.8\
    pcl_tracking-1.8\
    pcl_visualization-1.8\


SOURCES += main.cpp\
        pclproject.cpp \
    import_and_clean.cpp

HEADERS  += pclproject.h \
    import_and_clean.h

FORMS    += pclproject.ui

OTHER_FILES +=

unix: LIBS += -lboost_system
