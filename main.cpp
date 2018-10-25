// c++ standard libraries
#include <iostream>

// project helpers
#include "DataStructures.h"

// Point Cloud Library (PCL) supports
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

using namespace std;

int main( int argc, char** argv ) {

    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZRGBA>::Ptr cloud_colored (new PointCloud<PointXYZRGBA>);
    visualization::PCLVisualizer visualizer ("Backend visualizer");
    visualization::CloudViewer viewer ("Simple Cloud Viewer");

    if (io::loadPCDFile<PointXYZ> ("test.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd "
              << std::endl;

    visualizer.addPointCloud( cloud_colored, "cloud" );
    cloud_colored->points.resize( cloud->points.size() );
    for( size_t i = 0; i < cloud->points.size(); i++ )
    {
        cloud_colored->points[i].x = cloud->points[i].x;
        cloud_colored->points[i].y = cloud->points[i].y;
        cloud_colored->points[i].z = cloud->points[i].z;
        cloud_colored->points[i].r = 255;
        cloud_colored->points[i].g = 0;
        cloud_colored->points[i].b = 0;
        cloud_colored->points[i].a = 255;
    }
    visualizer.setPointCloudRenderingProperties ( visualization::PCL_VISUALIZER_POINT_SIZE, 5 );

    viewer.showCloud ( cloud_colored );
    while( !viewer.wasStopped() )
    {

    }

//    for (const auto & point : cloud->points )
//    {
//        std::cout << "    " << point.x
//                  << " "    << point.y
//                  << " "    << point.z << std::endl;
//    }

    return 0;
}