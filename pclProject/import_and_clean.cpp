#include "import_and_clean.h"

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

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
    int viewport0(0);
    int viewport1(1);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    /*
     * xmax = 0.5 for second viewport (voxel_filter)
     * xmax = 1.0 for normal screen
     */
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewport0);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, viewport0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> fixedCloud(visualizerCloud1);
    viewer->addPointCloud<pcl::PointXYZRGB> (visualizerCloud1, fixedCloud, "fixedCloud", viewport0);
    //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "fixedCloud");
    //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fixedCloud");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> moveableCloud(visualizerCloud2);
    viewer->addPointCloud<pcl::PointXYZRGB> (visualizerCloud2, moveableCloud,"moveableCloud", viewport0);


    //    viewer->addCoordinateSystem(2.0,"moveableCloud",0);
    //    viewer->setCameraPosition();

    viewer->addCoordinateSystem(1.0, "fixedCloud", viewport0);

    viewer->initCameraParameters ();
    viewer->addText("Axis ?",10,15,"Axis", viewport0);
    viewer->addText("Stepsize: ?",10,30,"Stepsize", viewport0);

    viewer->registerKeyboardCallback(keyBoardEventOccoured, (void*) &viewer);

    /*
     * Transform pcl to it's origin
     * based on it's centroid
     */
    transform_to_origin(visualizerCloud1, viewer);

    /*
     * Get the max segmentation with planar segmentation
     */
    //    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //    planar_segmentation(visualizerCloud1, coefficients, inliers);


    /*
     * Do a voxel filterung to reduce points -> faster
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_visualizerCloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_filter(visualizerCloud1,filtered_visualizerCloud1,viewer,viewport1);

    /*
     * Do segmentation and get the largest segment
     * More intelligent than planar_segmentation
     *
     * Needs viewport1 to be 0.5 in Xmax (see above)
     */
    extract_indices(filtered_visualizerCloud1);

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

void Import_And_Clean::transform_to_origin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                                           boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_)
{
    /*
     * Get center of pointcloud
     */
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud_ptr_, centroid);

    /*
     * Move pointcloud to origin 0,0,0
     */
    Eigen::Vector3f init_translationsVector(-centroid[0],-centroid[1],-centroid[2]);
    Eigen::Vector3f init_rotationsVector(0,0,0);
    Eigen::Affine3f init_transform;

    init_transform = Eigen::Affine3f::Identity();
    init_transform.pretranslate(init_translationsVector);
    viewer_->updatePointCloudPose("fixedCloud",init_transform);
}

void Import_And_Clean::planar_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                                           pcl::ModelCoefficients::Ptr &coefficients_,
                                           pcl::PointIndices::Ptr &inliers_)
{
    /*
     * Planar Segmentation
     */

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.10);

    seg.setInputCloud (cloud_ptr_);
    seg.segment (*inliers_, *coefficients_);

    if (inliers_->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    for (size_t i = 0; i < inliers_->indices.size (); ++i)
        std::cerr << inliers_->indices[i] << "    " << cloud_ptr_->points[inliers_->indices[i]].x << " "
                  << cloud_ptr_->points[inliers_->indices[i]].y << " "
                  << cloud_ptr_->points[inliers_->indices[i]].z << std::endl;

    std::cerr << "Model coefficients: " << coefficients_->values[0] << " "
              << coefficients_->values[1] << " "
              << coefficients_->values[2] << " "
              << coefficients_->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers_->indices.size () << std::endl;

    return;
}

void Import_And_Clean::voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_in_,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_out_,
                                    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_,
                                    int viewport_){

    std::cerr << "PointCloud before filtering: " << cloud_ptr_in_->width * cloud_ptr_in_->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid< pcl::PointXYZRGB > sor;
    sor.setInputCloud (cloud_ptr_in_);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_ptr_out_);

    std::cerr << "PointCloud after filtering: " << cloud_ptr_out_->width * cloud_ptr_out_->height << " data points." << std::endl;

    viewer_->createViewPort (0.5, 0.0, 1.0, 1.0, viewport_);
    viewer_->setBackgroundColor (0.3, 0.3, 0.3, viewport_);
    std::stringstream ss;
    ss << "Downsampled cloud: " << cloud_ptr_out_->width * cloud_ptr_out_->height << " data points";
    viewer_->addText (ss.str() , 10, 10, "viewport1 text", viewport_);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> downsampledCloud(cloud_ptr_out_);
    viewer_->addPointCloud<pcl::PointXYZRGB> (cloud_ptr_out_, downsampledCloud,"downsampledCloud", viewport_);
}

void Import_And_Clean::extract_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &filtered_cloud_ptr_)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Write the downsampled version to disk
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZRGB> ("Scan1_downsampled.pcd", *filtered_cloud_ptr_, false);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.10);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int) filtered_cloud_ptr_->points.size ();
    // While 30% of the original cloud is still there
    while (filtered_cloud_ptr_->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (filtered_cloud_ptr_);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (filtered_cloud_ptr_);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

//        std::stringstream ss;
//        ss << "segmented_plane_" << i << ".pcd";
//        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        filtered_cloud_ptr_.swap (cloud_f);
        i++;
    }

    return;
}
