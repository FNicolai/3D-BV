#include "import_and_clean.h"

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

Import_And_Clean::Import_And_Clean()
{

}

void Import_And_Clean::start()
{
    string path = "../Scan1.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file!\n");
        return;
    }

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

//    for (size_t i = 0; i < cloud->points.size (); ++i)
//        std::cout << "    " << cloud->points[i].x
//                  << " "    << cloud->points[i].y
//                  << " "    << cloud->points[i].z << std::endl;


    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    }

}
