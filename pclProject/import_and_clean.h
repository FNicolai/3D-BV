#ifndef IMPORT_AND_CLEAN_H
#define IMPORT_AND_CLEAN_H

#include <iostream>
#include <string>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/keyboard_event.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#include <Eigen/Geometry>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

class Import_And_Clean
{
public:
    Import_And_Clean();
    void start();

    Eigen::Affine3f transform_to_origin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                             boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_
                             ,const std::string &cloud_id_);

    void outlier_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_in_,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_out_);

    void voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_in_,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_out_);

    Eigen::Affine3f transform_to_positivXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                             boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_
                             ,const std::string &cloud_id_);

    void planar_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planar_comp_cloud_ptr_,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &negativ_cloud_ptr_);

    void improved_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planar_comp_cloud_ptr_,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &negativ_cloud_ptr_);

    void show_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planar_comp_cloud_ptr_,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &negativ_cloud_ptr_);

private:

};

#endif // IMPORT_AND_CLEAN_H
