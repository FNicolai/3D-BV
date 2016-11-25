#include "import_and_clean.h"

#include <iostream>
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

using namespace std;

//The rotation axis and the norm of the plane in which translations take place (1=X | 2=Y | 3=Z)
int axis = 3;
//Translation of pointcloud along plane
float deltaX(0.0), deltaY(0.0);
//Torque angle around Axis
float deltaTorque(0.0);
float stepSize=5;

bool orthogonalMode(true), initialPositioningDone(false);

const int key_0 = 48;
const int key_1 = 49;
const int key_2 = 50;
const int key_3 = 51;
const int key_4 = 52;
const int key_5 = 53;
const int key_6 = 54;
const int key_7 = 55;
const int key_8 = 56;
const int key_9 = 57;

Import_And_Clean::Import_And_Clean()
{

}

void keyBoardEventOccoured(const pcl::visualization::KeyboardEvent& event, void* viewer_){

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_);

    if(event.getKeyCode() == '.' && event.keyUp()){
        stepSize /= 5;
    }
    if(event.getKeyCode() == ',' && event.keyUp()){
        stepSize *= 5;
    }
    viewer->updateText("Stepsize: " + boost::lexical_cast<std::string>(stepSize),10,30,"Stepsize");

    //default steps
    float moveSize = 0.10;
    float torqueSize = 0.010;
    moveSize *= stepSize;
    torqueSize *= stepSize;

    // Toogle Colour mode
    if(event.getKeyCode() == key_0){
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "moveableCloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "fixedCloud");
    }

    // X Axis
    if(event.getKeyCode() == key_1){
        axis = 1;
        if(orthogonalMode)
            viewer->setCameraPosition(10,0,0,0,0,0,0,0,1);
    }
    // Y Axis
    if(event.getKeyCode() == key_2){
        axis = 2;
        if(orthogonalMode)
            viewer->setCameraPosition(0,10,0,0,0,0,1,0,0);
    }
    // Z Axis
    if(event.getKeyCode() == key_3){
        axis = 3;
        if(orthogonalMode)
            viewer->setCameraPosition(0,0,10,0,0,0,0,1,0);
    }

    // Translations
    if(event.getKeyCode() == key_4){
        deltaX = moveSize;
    }
    if(event.getKeyCode() == key_5){
        deltaX = -moveSize;
    }
    if(event.getKeyCode() == key_6){
        deltaY = moveSize;
    }
    if(event.getKeyCode() == key_7){
        deltaY = -moveSize;
    }
    // Rotations
    if(event.getKeyCode() == key_8){
        deltaTorque = torqueSize;
    }
    if(event.getKeyCode() == key_9){
        deltaTorque = -torqueSize;
    }
    if(event.getKeyCode() == 'o' && event.keyUp()){
        orthogonalMode = !orthogonalMode;
        viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(orthogonalMode);
    }
    if(event.getKeyCode() == 'i'){
        initialPositioningDone = true;
    }
}

void Import_And_Clean::start()
{
    //string path = "../pclProject/pointclouds_fabian/cloud_10.pcd";
    string path = "../pclProject/Scan1.pcd";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualizerCloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *visualizerCloud1) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file!\n");
        return;
    }

    std::cout << "Loaded "
              << visualizerCloud1->width * visualizerCloud1->height
              << " data points from the pcd with the following fields: "
              << std::endl; 

    //    for (size_t i = 0; i < cloud->points.size (); ++i)
    //        std::cout << "    " << cloud->points[i].x
    //                  << " "    << cloud->points[i].y
    //                  << " "    << cloud->points[i].z << std::endl;


    //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //    viewer.showCloud (cloud);
    //    while (!viewer.wasStopped ())
    //    {
    //    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualizerCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*visualizerCloud1, *visualizerCloud2);

    //--------------------------
    // -----Vizualizer Init-----
    //--------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> fixedCloud(visualizerCloud1);
    viewer->addPointCloud<pcl::PointXYZRGB> (visualizerCloud1, fixedCloud, "fixedCloud");
    //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "fixedCloud");
    //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fixedCloud");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> moveableCloud(visualizerCloud2);
    viewer->addPointCloud<pcl::PointXYZRGB> (visualizerCloud2, moveableCloud, "moveableCloud");


    //    viewer->addCoordinateSystem(2.0,"moveableCloud",0);
    //    viewer->setCameraPosition();

    viewer->addCoordinateSystem(1.0, "fixedCloud", 0);

    viewer->initCameraParameters ();
    int Axis_txt(0);
    viewer->addText("Axis ?",10,15,"Axis",Axis_txt);
    int Stepsize_txt(0);
    viewer->addText("Stepsize: ?",10,30,"Stepsize",Stepsize_txt);

    viewer->registerKeyboardCallback(keyBoardEventOccoured, (void*) &viewer);


    /*
     * Get center of pointcloud
     */
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*visualizerCloud1, centroid);

    /*
     * Get covariance matrix
     */
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrix (*visualizerCloud1, centroid, covariance_matrix);

    /*
     * Move pointcloud to origin 0,0,0
     */
    Eigen::Vector3f init_translationsVector(-centroid[0],-centroid[1],-centroid[2]);
    Eigen::Vector3f init_rotationsVector(0,0,0);
    Eigen::Affine3f init_transform;

    init_transform = Eigen::Affine3f::Identity();
    init_transform.pretranslate(init_translationsVector);
    viewer->updatePointCloudPose("fixedCloud",init_transform);

    /*
     * Segmentation
     */
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;

//    // Optional
//    seg.setOptimizeCoefficients (true);

//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);

//    seg.setInputCloud (visualizerCloud1);
//    seg.segment (*inliers, *coefficients);

    viewer->setCameraPosition(0,0,10,0,0,0,0,1,0);
    viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);

    Eigen::Vector3f translationsVector(0,0,0);
    Eigen::Vector3f rotationsVector(0,0,0);
    Eigen::Affine3f transform2;
    while (!viewer->wasStopped ()){
        viewer->updateText("Stepsize: " + boost::lexical_cast<std::string>(stepSize),10,30,"Stepsize");
        viewer->spinOnce (100);
        transform2 = Eigen::Affine3f::Identity();
        transform2.pretranslate(translationsVector);
        switch(axis){
        case 1:
            viewer->updateText("Axis X",10,15,"Axis");
            rotationsVector[0] +=deltaTorque;
            translationsVector[1] += deltaX;
            translationsVector[2] += deltaY;
            break;
        case 2:
            viewer->updateText("Axis Y",10,15,"Axis");
            rotationsVector[1] +=deltaTorque;
            translationsVector[0] += deltaY;
            translationsVector[2] += deltaX;
            break;
        case 3:
            viewer->updateText("Axis Z",10,15,"Axis");
            rotationsVector[2] +=deltaTorque;
            translationsVector[0] += deltaX;
            translationsVector[1] += deltaY;
            break;
        }
        transform2.rotate(Eigen::AngleAxisf(rotationsVector[0] , Eigen::Vector3f::UnitX()));
        transform2.rotate(Eigen::AngleAxisf(rotationsVector[1] , Eigen::Vector3f::UnitY()));
        transform2.rotate(Eigen::AngleAxisf(rotationsVector[2] , Eigen::Vector3f::UnitZ()));
        viewer->updatePointCloudPose("moveableCloud",transform2);


        deltaTorque = 0.0;
        deltaX = 0.0;
        deltaY = 0.0;
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        if(initialPositioningDone)
            break;
    }



}
