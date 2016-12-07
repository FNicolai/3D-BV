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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>


using namespace std;

//The rotation axis and the norm of the plane in which translations take place (1=X | 2=Y | 3=Z)
int axis = 3;
//Translation of pointcloud along plane
float deltaX(0.0), deltaY(0.0);
//Torque angle around Axis
float deltaTorque(0.0);
float stepSize=5;

bool orthogonalMode(false), initialPositioningDone(false), saveRequest(false);

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
    viewer->updateText("Stepsize: " + boost::lexical_cast<std::string>(stepSize),10,45,"Stepsize");

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
        //if(orthogonalMode)
            viewer->setCameraPosition(10,0,0,0,0,0,0,0,1);
    }
    // Y Axis
    if(event.getKeyCode() == key_2){
        axis = 2;
        //if(orthogonalMode)
            viewer->setCameraPosition(0,10,0,0,0,0,1,0,0);
    }
    // Z Axis
    if(event.getKeyCode() == key_3){
        axis = 3;
        //if(orthogonalMode)
            viewer->setCameraPosition(0,0,10,0,0,0,0,1,0);
    }

    if(pcl::visualization::KeyboardEvent::isCtrlPressed()){ // Manual alignment operations
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
    }else{ // Cube operations

    }


    if(event.getKeyCode() == 'i'){
        initialPositioningDone = true;
    }
    if(event.getKeyCode() == 's'){
        saveRequest = true;
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

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualizerCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::copyPointCloud(*visualizerCloud1, *visualizerCloud2);

    //--------------------------
    // -----Vizualizer Init-----
    //--------------------------
    int viewport0(0);
    int viewport1(1);
    int viewport2(2);
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

//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> moveableCloud(visualizerCloud2);
//    viewer->addPointCloud<pcl::PointXYZRGB> (visualizerCloud2, moveableCloud,"moveableCloud", viewport0);


    //    viewer->addCoordinateSystem(2.0,"moveableCloud",0);
    //    viewer->setCameraPosition();

    viewer->addCoordinateSystem(1.0, viewport0);

    viewer->initCameraParameters ();
    viewer->addText("Axis ?",10,15,"Axis", viewport0);
    viewer->addText("Stepsize: ?",10,45,"Stepsize", viewport0);

    viewer->registerKeyboardCallback(keyBoardEventOccoured, (void*) &viewer);

    viewer->setCameraPosition(0,0,10,0,0,0,0,1,0);
    //viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);


    /*
     * Transform pcl to it's origin
     * based on it's centroid
     */
    transform_to_origin(visualizerCloud1, viewer, "fixedCloud");

    /*
     * Do a voxel filterung to reduce points -> faster
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_filter(visualizerCloud1, downsampledCloud_ptr);

    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, viewport1);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, viewport1);
    std::stringstream ss;
    ss << "Downsampled cloud: " << downsampledCloud_ptr->width * downsampledCloud_ptr->height << " data points";
    viewer->addText (ss.str() , 10, 10, "viewport1 text", viewport1);
    std::stringstream ss2;
    ss2 << "Original cloud: " << visualizerCloud1->width * visualizerCloud1->height << " data points";
    viewer->addText (ss2.str() , 10, 10, "viewport0 text", viewport0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> downsampledCloud(downsampledCloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (downsampledCloud_ptr, downsampledCloud, "downsampledCloud", viewport1);
    viewer->addCoordinateSystem(1.0, viewport1);
    Eigen::Affine3f better_transform = transform_to_origin(downsampledCloud_ptr, viewer, "downsampledCloud");

    /*
     * Transform the original pointcloud to the coordinates of the
     * downsampled cloud.
     */
    pcl::transformPointCloud(*visualizerCloud1,*visualizerCloud1,better_transform);
    viewer->updatePointCloud(visualizerCloud1,"fixedCloud");

    /*
     * Removing outliers using
     * a StatisticalOutlierRemoval filter
     */
    outlier_removal(downsampledCloud_ptr, downsampledCloud_ptr);

    /*
     * Transform pointcloud into positve space.
     */
    transform_to_positivXYZ(downsampledCloud_ptr, viewer, "downsampledCloud");

    pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;
    cropBoxFilter.setInputCloud(downsampledCloud_ptr);

    viewer->addCube (16.2, 18.2, -7.9, -5.9, 0.21, 2.21, 1.0, 0, 0, "cube", 0);

    // ############################## Segmentation ##################################
    /*
     * Instanciate clouds for segmentation
     * Use planar or improved_segmentation
     */
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_comp_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr negativ_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    /*
     * Get the max segmentation with planar segmentation
     */
    //planar_segmentation(downsampledCloud_ptr, planar_comp_cloud_ptr,negativ_cloud_ptr);


    /*
     * Do segmentation and get the largest segment
     * More intelligent than planar_segmentation
     *
     * Needs viewport1 to be 0.5 in Xmax (see above)
     */
    //improved_segmentation(downsampledCloud_ptr,planar_comp_cloud_ptr,negativ_cloud_ptr);

    /*
     * Show the results of segmentation
     */
    //show_segmentation(planar_comp_cloud_ptr,negativ_cloud_ptr);
    // ############################## Segmentation END ##############################


    /*
     * ToDo description
     */
    Eigen::Vector3f translationsVector(0,0,0);
    Eigen::Vector3f rotationsVector(0,0,0);
    Eigen::Affine3f transform2;
    while (!viewer->wasStopped ()){
        viewer->updateText("Stepsize: " + boost::lexical_cast<std::string>(stepSize),10,45,"Stepsize");
        viewer->spinOnce (100);
        transform2 = Eigen::Affine3f::Identity();
        transform2.pretranslate(translationsVector);
        switch(axis){
        case 1:
            viewer->updateText("Axis X",10,30,"Axis");
            rotationsVector[0] +=deltaTorque;
            translationsVector[1] += deltaX;
            translationsVector[2] += deltaY;
            break;
        case 2:
            viewer->updateText("Axis Y",10,30,"Axis");
            rotationsVector[1] +=deltaTorque;
            translationsVector[0] += deltaY;
            translationsVector[2] += deltaX;
            break;
        case 3:
            viewer->updateText("Axis Z",10,30,"Axis");
            rotationsVector[2] +=deltaTorque;
            translationsVector[0] += deltaX;
            translationsVector[1] += deltaY;
            break;
        }
        transform2.rotate(Eigen::AngleAxisf(rotationsVector[0] , Eigen::Vector3f::UnitX()));
        transform2.rotate(Eigen::AngleAxisf(rotationsVector[1] , Eigen::Vector3f::UnitY()));
        transform2.rotate(Eigen::AngleAxisf(rotationsVector[2] , Eigen::Vector3f::UnitZ()));
        //viewer->updatePointCloudPose("moveableCloud",transform2);

        pcl::transformPointCloud(*downsampledCloud_ptr,*downsampledCloud_ptr,transform2);
        viewer->updatePointCloud(downsampledCloud_ptr,"downsampledCloud");

        if (saveRequest){
            pcl::PCDWriter writer;
            string savePath = path;
            savePath.insert(savePath.length()-4, "_saved");

            writer.write<pcl::PointXYZRGB> (savePath, *downsampledCloud_ptr, false);
        }


        deltaTorque = 0.0;
        deltaX = 0.0;
        deltaY = 0.0;
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        if(initialPositioningDone)
            break;
    }
}

Eigen::Affine3f Import_And_Clean::transform_to_origin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                                           boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_,
                                           const std::string &cloud_id_)
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
    pcl::transformPointCloud(*cloud_ptr_,*cloud_ptr_,init_transform);
    viewer_->updatePointCloud(cloud_ptr_,cloud_id_);

    return init_transform;
}

void Import_And_Clean::outlier_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_in_,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_out_)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_ptr_in_);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_ptr_out_);

//    sor.setNegative (true);
//    sor.filter (*cloud_filtered);
    return;
}

void Import_And_Clean::voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_in_,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_out_){

    std::cerr << "PointCloud before filtering: " << cloud_ptr_in_->width * cloud_ptr_in_->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid< pcl::PointXYZRGB > sor;
    sor.setInputCloud (cloud_ptr_in_);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_ptr_out_);
    std::cerr << "PointCloud after filtering: " << cloud_ptr_out_->width * cloud_ptr_out_->height << " data points." << std::endl;
    return;
}

Eigen::Affine3f Import_And_Clean::transform_to_positivXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                                                          boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_
                                                          ,const std::string &cloud_id_)
{
    /*
     * Init
     */
    Eigen::Vector3f translationsVector(0,0,0);
    Eigen::Vector3f rotationsVector(0,0,0);
    Eigen::Affine3f transform;
    double maxNegX = 0, maxNegY = 0, maxNegZ = 0;

    /*
     * Find max negativ X,Y & Z
     */
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud_ptr_->begin(); it != cloud_ptr_->end(); it++){
        if (maxNegX > it->x){
            maxNegX = it->x;
        }
        if (maxNegY > it->y){
            maxNegY = it->y;
        }
        if (maxNegZ > it->z){
            maxNegZ = it->z;
        }
        cout << it->x << ", " << it->y << ", " << it->z << endl;
    }

    cout << "Max negative XYZ: " << maxNegX << ", " << maxNegY << ", " << maxNegZ << endl;

    translationsVector[0] = -maxNegX;
    translationsVector[1] = -maxNegY;
    translationsVector[2] = -maxNegZ;

    /*
     * Transform
     */
    transform = Eigen::Affine3f::Identity();
    transform.pretranslate(translationsVector);
    transform.rotate(Eigen::AngleAxisf(rotationsVector[0] , Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(rotationsVector[1] , Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(rotationsVector[2] , Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*cloud_ptr_, *cloud_ptr_, transform);
    viewer_->updatePointCloud(cloud_ptr_, cloud_id_);

    return transform;
}

void Import_And_Clean::planar_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planar_comp_cloud_ptr_,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &negativ_cloud_ptr_)
{
    /*
     * Planar Segmentation
     */

    // Create the segmentation object
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (3);


    seg.setInputCloud (cloud_ptr_);
    seg.segment (*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Extract the planar component
    extract.setInputCloud (cloud_ptr_);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planar_comp_cloud_ptr_);

    //Extract the negative
    extract.setNegative (true);
    extract.filter (*negativ_cloud_ptr_);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    for (size_t i = 0; i < inliers->indices.size (); ++i)
        std::cerr << inliers->indices[i] << "    " << cloud_ptr_->points[inliers->indices[i]].x << " "
                  << cloud_ptr_->points[inliers->indices[i]].y << " "
                  << cloud_ptr_->points[inliers->indices[i]].z << std::endl;

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    return;
}

void Import_And_Clean::improved_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr_,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planar_comp_cloud_ptr_,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &negativ_cloud_ptr_)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr max_planar_comp_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr max_inliers (new pcl::PointIndices ());

    pcl::copyPointCloud(*cloud_ptr_, *max_planar_comp_cloud_ptr);

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

    int i = 0, nr_points = (int) cloud_ptr_->points.size ();
    int max_data_points = 0;
    // While 30% of the original cloud is still there
    while (cloud_ptr_->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_ptr_);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_ptr_);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*planar_comp_cloud_ptr_);
        int curr_data_points = planar_comp_cloud_ptr_->width * planar_comp_cloud_ptr_->height;
        std::cerr << "PointCloud representing the planar component: " << curr_data_points << " data points." << std::endl;

        /*
         * Save the biggest planar component
         */
        if(curr_data_points > max_data_points){
            max_data_points = curr_data_points;
            max_inliers.swap(inliers);
            //pcl::copyPointCloud(*planar_comp_cloud_ptr_, *max_planar_comp_cloud_ptr);
        }

//        std::stringstream ss;
//        ss << "segmented_plane_" << i << ".pcd";
//        writer.write<pcl::PointXYZRGB> (ss.str (), *planar_comp_cloud_ptr_, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*negativ_cloud_ptr_);
        cloud_ptr_.swap (negativ_cloud_ptr_);
        i++;
    }

    // Extract the final inliers
    extract.setInputCloud (max_planar_comp_cloud_ptr);
    extract.setIndices (max_inliers);
    extract.setNegative (false);
    extract.filter (*planar_comp_cloud_ptr_);

    // Create the final filtering object negativ
    extract.setNegative (true);
    extract.filter (*negativ_cloud_ptr_);

    //planar_comp_cloud_ptr_ = max_planar_comp_cloud_ptr;
    std::cerr << "PointCloud representing the biggest planar component: "
              << planar_comp_cloud_ptr_->width * planar_comp_cloud_ptr_->height
              << " data points." << std::endl;

    return;
}

void Import_And_Clean::show_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planar_comp_cloud_ptr_,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &negativ_cloud_ptr_)
{
    int viewport0(0);
    int viewport1(1);
    int viewport2(2);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_seg (new pcl::visualization::PCLVisualizer ("3D Viewer seg"));

    viewer_seg->createViewPort(0.0, 0.0, 0.33, 1.0, viewport0);
    viewer_seg->setBackgroundColor(0.0, 0.0, 0.0, viewport0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> negativ_cloud(negativ_cloud_ptr_);
    viewer_seg->addPointCloud<pcl::PointXYZRGB> (negativ_cloud_ptr_, negativ_cloud, "negativ_cloud", viewport0);
    viewer_seg->addCoordinateSystem(1.0, viewport0);
    viewer_seg->addText ("Complement" , 10, 10, "negativ_cloud_txt", viewport0);

    viewer_seg->createViewPort(0.33, 0.0, 0.66, 1.0, viewport1);
    viewer_seg->setBackgroundColor(0.0, 0.0, 0.0, viewport1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> planar_comp_cloud(planar_comp_cloud_ptr_);
    viewer_seg->addPointCloud<pcl::PointXYZRGB> (planar_comp_cloud_ptr_, planar_comp_cloud, "planar_comp_cloud", viewport1);
    viewer_seg->addCoordinateSystem(1.0, viewport1);
    viewer_seg->addText ("Biggest planar component" , 10, 10, "planar_comp_cloud_txt", viewport1);

    viewer_seg->initCameraParameters ();
    viewer_seg->setCameraPosition(0,0,10,0,0,0,0,1,0);
    //viewer_seg->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);

    /*
     * Rotate segment and insert it back again
     */
    Eigen::Vector3f seg_translationsVector(0,0,0);
    Eigen::Vector3f seg_rotationsVector(90,0,0);
    Eigen::Affine3f seg_transform;

    seg_transform = Eigen::Affine3f::Identity();
    seg_transform.pretranslate(seg_translationsVector);
    seg_transform.rotate(Eigen::AngleAxisf(seg_rotationsVector[0] , Eigen::Vector3f::UnitX()));
    seg_transform.rotate(Eigen::AngleAxisf(seg_rotationsVector[1] , Eigen::Vector3f::UnitY()));
    seg_transform.rotate(Eigen::AngleAxisf(seg_rotationsVector[2] , Eigen::Vector3f::UnitZ()));


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*planar_comp_cloud_ptr_, *combined_cloud_ptr);

    pcl::transformPointCloud(*combined_cloud_ptr, *combined_cloud_ptr, seg_transform);
    *combined_cloud_ptr += *negativ_cloud_ptr_;

    viewer_seg->createViewPort(0.66, 0.0, 1.0, 1.0, viewport2);
    viewer_seg->setBackgroundColor(0.0, 0.0, 0.0, viewport2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> combined_cloud(combined_cloud_ptr);
    viewer_seg->addPointCloud<pcl::PointXYZRGB> (combined_cloud_ptr, combined_cloud, "combined_cloud", viewport2);
    viewer_seg->addCoordinateSystem(1.0, viewport2);
    viewer_seg->addText ("Combined cloud" , 10, 10, "combined_cloud_txt", viewport2);
}
