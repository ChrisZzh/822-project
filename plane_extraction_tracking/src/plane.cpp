// ros
#include <ros/ros.h>

// opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// lidar plane extractor  
#include "../include/LidarPlaneExtractor.h"
#include "../include/DataStructures.h"

// Point Cloud Library (PCL) 
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

// c++ lib  
#include <iostream>
#include <string>


using namespace pcl;
using namespace cv;
using namespace std;

// typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

// global variables 
bool received_image = false; 
bool received_lidar = false; 
ros::Time camera_time = {};
ros::Time lidar_time = {}; 
PointCloud<PointXYZ> cloud;   
cv::Mat image; 
Matrix4d T_I_W = {}; 


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

void copy_point_cloud_xyz_to_rgbd( const PointCloud<PointXYZ> &point_cloud, const PointCloud<PointXYZRGB>::Ptr &point_cloud_rgb_ptr )
{
    point_cloud_rgb_ptr->points.resize( point_cloud.points.size() );
    for( size_t i = 0; i < point_cloud.points.size(); i++ )
    {
        point_cloud_rgb_ptr->points[i].x = point_cloud.points[i].x;
        point_cloud_rgb_ptr->points[i].y = point_cloud.points[i].y;
        point_cloud_rgb_ptr->points[i].z = point_cloud.points[i].z;
        point_cloud_rgb_ptr->points[i].r = 0;
        point_cloud_rgb_ptr->points[i].g = 255;
        point_cloud_rgb_ptr->points[i].b = 0;
    }
}

void init_visualizer( visualization::PCLVisualizer &visualizer, const PointCloud<PointXYZRGB>::Ptr &rgb_cloud_ptr )
{
    visualizer.setBackgroundColor( 0, 0, 0 );
    visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb( rgb_cloud_ptr );
    visualizer.addPointCloud<PointXYZRGB> ( rgb_cloud_ptr, rgb, "cloud");
    visualizer.addCoordinateSystem( 3.0 );
    visualizer.setPointCloudRenderingProperties ( visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" );
    visualizer.initCameraParameters();
}

void plane_extraction( const PointCloud<PointXYZRGB>::Ptr &rgb_cloud_ptr, LidarPlaneExtractor &plane_extractor )
{
    plane_extractor.putPointsInCells();
    plane_extractor.find_ground_plane_in_each_cell();
    plane_extractor.labelGroundAndNonGroundPoints();
    plane_extractor.computePlaneParameters();
    plane_extractor.findPlaneInImage(T_I_W);
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
    cout << s.height << " " << s.width << endl;
    for (size_t row = 0; row < ground_pixel_indices.rows(); row++) {
        if (ground_pixel_indices(row, 0) < 0.f || ground_pixel_indices(row, 1) < 0.f) {
            continue;
        } else {
//            cout << "j = " << ground_pixel_indices(row, 0) << " i = " << ground_pixel_indices(row, 1) << " w = " << ground_pixel_indices(row, 2) << endl;
            auto j = static_cast<size_t>(floor(ground_pixel_indices(row, 0)));
            auto i = static_cast<size_t>(floor(ground_pixel_indices(row, 1)));
//            cout << "i = " << i << " j = " << j << endl;
            if (i > s.height || j > s.width) {
                continue;
            }
            image.at<Vec3b>(i, j)[0] = 0;
            image.at<Vec3b>(i, j)[1] = 255;
            image.at<Vec3b>(i, j)[2] = 0;
//            cout << "ground labled." << endl;
        }
    }
}

void camBased_callback(const sensor_msgs::Image::ConstPtr &img_msg_L, const nav_msgs::Odometry::ConstPtr &odom_msg) { 
    camera_time = odom_msg->header.stamp; 
    received_image = true; 
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg_L, sensor_msgs::image_encodings::BGR8);
    // image = cv_ptr->image; 
    cv::resize(cv_ptr->image, image, cv::Size(), 0.5, 0.5); 
    // get pose in Eigen, rotation matrix and translation
    Eigen::Quaterniond orient = Eigen::Quaternion<double>(odom_msg->pose.pose.orientation.w,
                                                             odom_msg->pose.pose.orientation.x,
                                                             odom_msg->pose.pose.orientation.y,
                                                             odom_msg->pose.pose.orientation.z);
    // get Eigen transformation matrix
    T_I_W = Eigen::Matrix4d::Zero();
    T_I_W(3,3) = 1;
    T_I_W.block<3,3>(0,0) = orient.normalized().toRotationMatrix();
    T_I_W.block<3,1>(0,3) = Eigen::Matrix<double,3,1>(odom_msg->pose.pose.position.x,
                                                      odom_msg->pose.pose.position.y,
                                                      odom_msg->pose.pose.position.z);
}

// void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud) {
void lidar_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud) { 
    received_lidar = true; 
    // lidar_time = point_cloud->header.stamp;
    // pcl::PCLPointCloud2 pcl_pc2; 
    // pcl_conversions::toPCL(*point_cloud, pcl_pc2);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>); 
    // pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud); 
    // cloud = *temp_cloud; 
    cloud = *point_cloud;  
}

// main
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh("~");

    ros::Subscriber lidar_sub; 
    message_filters::Subscriber<sensor_msgs::Image> img_sub_L;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    img_sub_L.subscribe(nh, "/mapping/left/image_rect_color", 10000);
    odom_sub.subscribe(nh, "/smart_smoother/odom_imu", 10000);
    lidar_sub = nh.subscribe("/feature_points", 10, &lidar_callback); 

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), img_sub_L, odom_sub);
    sync.registerCallback(boost::bind(&camBased_callback, _1, _2));

    while (true) {
        // check condition 
        if (received_lidar && received_image) { 
            std::clock_t start;
            double duration = 0.f;

            PointCloud<PointXYZRGB>::Ptr cloud_colored (new PointCloud<PointXYZRGB>);

            start = std::clock();

            copy_point_cloud_xyz_to_rgbd( cloud, cloud_colored );

            LidarPlaneExtractor plane_extractor( cloud_colored );

            plane_extraction(cloud_colored, plane_extractor);

            // visualization::PCLVisualizer visualizer( "Visualizer" );
            // init_visualizer( visualizer, cloud_colored );

            duration += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

            // visualizer.spinOnce( 100 );
            // boost::this_thread::sleep( boost::posix_time::microseconds (100) );

            start = std::clock();

            if (!image.data) {
                cout <<  "Could not open or find the image" << std::endl ;
                return -1;
            }
            labelGround(image, plane_extractor.getGroundPointsPixelLocation());
            imshow( "Image", image );

            duration += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

            cout << "Loading and processing one scan consumes: " << duration << " secs." << endl;

            waitKey(1); 

            // cout << (double(lidar_time.toNSec())/1.0e9 - double(camera_time.toNSec())/1.0e9) << endl; 
            received_image = false; 
            received_lidar = false;  
        }
        ros::spinOnce(); 
    }
    return 0;
}
