#-------------------------------------------------
#
# Project created by QtCreator 2016-11-22T16:32:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pclProject
TEMPLATE = app

INCLUDEPATH += "/usr/include/vtk-5.8"
LIBS += -lvtkCharts\
    -lvtkexoIIc\
    -lvtkFiltering\
    -lvtkDICOMParser\
    -lvtkVolumeRendering\
    -lvtkCommon\
    -lvtkInfovis\
    -lvtkGenericFiltering\
    -lvtksys\
    -lvtkParallel\
    -lvtkGraphics\
    -lvtkIO\
    -lvtkalglib\
    -lvtkproj4\
    -lvtkverdict\
    -lvtkHybrid\
    -lvtkViews\
    -lvtkImaging\
    -lvtkWidgets\
    -lvtkmetaio\
    -lVPIC\
    -lvtkRendering\
    -lvtkGeovis\
    -lCosmo\
    -lvtkftgl\

#INCLUDEPATH += "/usr/include/pcl-1.7"
#LIBS += "-L/usr/lib"\
    #lpcl_apps\
    #lpcl_common\
    #lpcl_features\
    #lpcl_filters\
    #lpcl_io\
    #lpcl_io_ply\
    #lpcl_kdtree\
    #lpcl_keypoints\
    #lpcl_octree\
    #lpcl_outofcore\
    #lpcl_people\
    #lpcl_recognition\
    #lpcl_registration\
    #lpcl_sample_consensus\
    #lpcl_search\
    #lpcl_segmentation\
    #lpcl_surface\
    #lpcl_tracking\
    #lpcl_visualization\




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

unix: LIBS += -lboost_system\

