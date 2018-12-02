// c++ standard libraries
#include <iostream>
#include <string>

// project helpers
#include "DataStructures.h"
#include "LidarPlaneExtractor.h"

// Point Cloud Library (PCL) supports
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// OpenCV supports
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace pcl;
using namespace cv;
using namespace std;

bool load_point_cloud( const string & filename, const PointCloud<PointXYZ>::Ptr &point_cloud_pointer )
{
    if (io::loadPCDFile<PointXYZ> (filename, *point_cloud_pointer) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return false;
    }

    std::cout << "Loaded "
              << point_cloud_pointer->width * point_cloud_pointer->height
              << " data points from test_pcd.pcd "
              << std::endl;

    return true;
}

void copy_point_cloud_xyz_to_rgbd( const PointCloud<PointXYZ>::Ptr &point_cloud_ptr, const PointCloud<PointXYZRGB>::Ptr &point_cloud_rgb_ptr )
{
    point_cloud_rgb_ptr->points.resize( point_cloud_ptr->points.size() );
    for( size_t i = 0; i < point_cloud_ptr->points.size(); i++ )
    {
        point_cloud_rgb_ptr->points[i].x = point_cloud_ptr->points[i].x;
        point_cloud_rgb_ptr->points[i].y = point_cloud_ptr->points[i].y;
        point_cloud_rgb_ptr->points[i].z = point_cloud_ptr->points[i].z;
        point_cloud_rgb_ptr->points[i].r = 255;
        point_cloud_rgb_ptr->points[i].g = 0;
        point_cloud_rgb_ptr->points[i].b = 0;
    }
}

void init_visualizer( visualization::PCLVisualizer &visualizer, const PointCloud<PointXYZRGB>::Ptr &rgb_cloud_ptr )
{
    visualizer.setBackgroundColor( 0, 0, 0 );
    visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb( rgb_cloud_ptr );
    visualizer.addPointCloud<PointXYZRGB> ( rgb_cloud_ptr, rgb, "cloud");
    visualizer.addCoordinateSystem( 1.0 );
    visualizer.setPointCloudRenderingProperties ( visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" );
    visualizer.initCameraParameters();
}

void plane_extraction( const PointCloud<PointXYZRGB>::Ptr &rgb_cloud_ptr, LidarPlaneExtractor &plane_extractor )
{
    plane_extractor.putPointsInCells();
    plane_extractor.find_ground_plane_in_each_cell();
    plane_extractor.labelGroundAndNonGroundPoints();
    plane_extractor.computePlaneParameters();
    plane_extractor.findPlaneInImage();
}

void print_cloud_rgb( const PointCloud<PointXYZRGB>::Ptr &rgb_cloud_ptr )
{
    for( const auto & point : rgb_cloud_ptr->points )
    {
        cout << "x = " << point.x << ", y = " << point.y << ", z = " << point.z << endl;
    }
}

void labelGround(Mat &image, const MatrixXd &ground_pixel_indices) {
    Size s = image.size();
    for (size_t row = 0; row < ground_pixel_indices.rows(); row++) {
        if (ground_pixel_indices(row, 0) < 0.f || ground_pixel_indices(row, 1) < 0.f) {
            continue;
        } else {
            auto j = static_cast<size_t>(floor(ground_pixel_indices(row, 0)));
            auto i = static_cast<size_t>(floor(ground_pixel_indices(row, 1)));
            if (i > s.width || j > s.height) {
                continue;
            }
            image.at<Vec3b>(i, j)[0] = 0;
            image.at<Vec3b>(i, j)[1] = 0;
            image.at<Vec3b>(i, j)[2] = 255;
            cout << "ground labled." << endl;
        }
    }
}

int main( int argc, char** argv ) {

    std::clock_t start;
    double duration;

    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

    PointCloud<PointXYZRGB>::Ptr cloud_colored (new PointCloud<PointXYZRGB>);

    start = std::clock();

    if( !load_point_cloud( "../data/scans/1528404295151414871.pcd", cloud ) )
    {
        return -1;
    };

    copy_point_cloud_xyz_to_rgbd( cloud, cloud_colored );

    LidarPlaneExtractor plane_extractor( cloud_colored );

    plane_extraction( cloud_colored, plane_extractor );

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    cout << "Loading and processing one scan consumes: " << duration << " secs." << endl;
//    print_cloud_rgb( cloud_colored );


//    visualization::PCLVisualizer visualizer( "Visualizer" );
//    init_visualizer( visualizer, cloud_colored );
//
//    while( !visualizer.wasStopped() )
//    {
//        visualizer.spinOnce( 100 );
//        boost::this_thread::sleep( boost::posix_time::microseconds (100) );
//    }

    Mat image;
    image = imread("../data/images/left_67_1528404295187234419.jpg", CV_LOAD_IMAGE_COLOR);
    if (!image.data) {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    labelGround(image, plane_extractor.getGroundPointsPixelLocation());
    namedWindow( "Image", WINDOW_AUTOSIZE );
    imshow( "Image", image );

    waitKey(0);
    return 0;
}
