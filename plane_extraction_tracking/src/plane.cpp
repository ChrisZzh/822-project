// ros
#include <ros/ros.h>

// opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// lidar plane extractor  
#include "LidarPlaneExtractor.h"
#include "DataStructures.h"

// Point Cloud Library (PCL) 
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// c++ lib  
#include <iostream>
#include <string>

// cv::Scalar colorTab[] =
// {
//     cv::Scalar(0, 0, 255),
//     cv::Scalar(0,255,0),
//     cv::Scalar(255,100,100),
//     cv::Scalar(255,0,255),
//     cv::Scalar(0,255,255),
//     cv::Scalar(255,0,0),
//     // cv::Scalar(100,100,255),
//     // cv::Scalar(100,255,100),
// };

// // row first 
// std::vector<uint> linearTo2D(uint row, uint col, uint l)
// {
//     std::vector<uint> TwD_index;
//     uint row_ind = floor(float(l)/float(col));
//     uint col_ind = l - row_ind*col;
//     TwD_index.push_back(row_ind);
//     TwD_index.push_back(col_ind);
//     return TwD_index;
// }

// // row first 
// uint TwDTolinear(uint row, uint col, std::vector<uint> TwD_index)
// {
//     return TwD_index[0] * col + TwD_index[1];
// }

// cv::Mat colorSeg(const cv::Mat& img)
// {
// 	// convert to HSV
// 	cv::Mat img_hsv;
// 	cv::cvtColor( img, img_hsv, CV_BGR2HSV );
//     // show BGR image
//     cv::imshow("orignal", img);
//     // show HSV image
//     cv::imshow( "hsv", img_hsv );
//     //
//     std::cout << "image rows: " << img_hsv.rows << "\n";
//     std::cout << "image cols: " << img_hsv.cols << "\n";

//     uint row_sz = img_hsv.rows, col_sz = img_hsv.cols, point_sz = row_sz*col_sz; 

//     // create a mat, filled with color data
//     cv::Mat points = cv::Mat(point_sz,3,CV_32F);
//     for ( uint i=0; i<point_sz; i++)
//     {
//         std::vector<uint> res = linearTo2D(row_sz,col_sz,i);
//         cv::Vec3b temp = img_hsv.at<cv::Vec3b>(res[0],res[1]);
// 	    points.at<float>(i,0) = float(temp.val[0]);
// 	    points.at<float>(i,1) = float(temp.val[1]);
// 	    points.at<float>(i,2) = float(temp.val[2]);
//     }
//     // k-means
//     cv::Mat labels, centers; // labels of size (point_sz,1)
//     cv::kmeans(points, 6
//     	, labels,
//         cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
//            3, cv::KMEANS_PP_CENTERS, centers);

//     cv::Mat img_seg(img_hsv.rows, img_hsv.cols, CV_8UC3);
//     img_seg = cv::Scalar::all(0); 
//     for( uint i = 0; i < img_seg.rows; i++ )
//     {
//         for ( uint j = 0; j < img_seg.cols; j++ )
//         {
//         	// retrive cluster ID
//         	uint ind_linear = TwDTolinear(row_sz, col_sz, std::vector<uint>{i,j});
//             int clusterID = labels.at<int>(ind_linear);
//             // plot with cluster color
//             cv::Point ipt = cv::Point(j,i);
//             cv::circle( img_seg, ipt, 2, colorTab[clusterID], cv::FILLED, cv::LINE_AA );
//         }
//     }
//     imshow("clusters", img_seg);
//     cv::waitKey(0);

//     return labels;	
// }

// // estimate homography
// void plane_extraction(cv::Mat& image_1, cv::Mat image_2) 
// {
// 	cv::Mat labels_1 = colorSeg(image_1);
// 	cv::Mat labels_2 = colorSeg(image_2);

// 	// extract features in the two images, return descriptors as well 
// 	// filter out points on the major plane (ground plane)

// 	// find homography use this set
// 	cv::Mat_<double> homo(3,3);
// 	cv::Mat_<double> K(3,3);
// 	homo << 0.003192301358473, 0.001081524315727, -0.999774880519790,
//             0.000025666856680, 0.002724831087490, -0.020709546919980,
//             0.000002269426529, 0.000000632177580,  0.001586535977295;
// 	K << 498.1357145, 0.0, 351.726944, 0.0, 498.1357145, 255.9642885, 0.0, 0.0, 1;

// 	cv::Mat homography = K * homo * K.inv();
// 	homography /= homography.at<double>(2,2);

//     std::vector<cv::Mat> Rs_decomp, ts_decomp, normals_decomp;
//     int solutions = cv::decomposeHomographyMat(homography, K, Rs_decomp, ts_decomp, normals_decomp);
//     std::cout << "Decompose homography matrix computed from the camera displacement:\n\n";
//     for (int i = 0; i < solutions; i++)
//     {
//       //double factor_d1 = 1.0 / d_inv1;
//       cv::Mat rvec_decomp;
//       cv::Rodrigues(Rs_decomp[i], rvec_decomp);
//       std::cout << "Solution " << i << ":" << std::endl;
//       std::cout << "rvec from homography decomposition: " << rvec_decomp.t() << std::endl;
//       std::cout << "tvec from homography decomposition: " << ts_decomp[i].t() << std::endl;;//<< " and scaled by d: " << factor_d1 * ts_decomp[i].t() << std::endl;
//       std::cout << "plane normal from homography decomposition: " << normals_decomp[i].t() << std::endl;
//       //cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
//     }
	
// 	// eigen solver n d

// 	return;
// }

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

void camBased_callback(const sensor_msgs::Image::ConstPtr &img_msg_L,
                                        const sensor_msgs::Image::ConstPtr &img_msg_R,
                                        const nav_msgs::Odometry::ConstPtr &odom_msg) {}
// main
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh("~");

    std::clock_t start;
    double duration = 0.f;

    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

    PointCloud<PointXYZRGB>::Ptr cloud_colored (new PointCloud<PointXYZRGB>);

    start = std::clock();

    String point_cloud_name = "../catkin_ws/scene_1/scans/1528404298784590960.pcd";
//    String point_cloud_name = "../catkin_ws/data/scans/1528404308068900824.pcd";
//    String point_cloud_name = "../catkin_ws/scene_2/test/1528543256210322857.pcd";
//    String point_cloud_name = "../catkin_ws/scene_2/test/1528543304469905853.pcd";
    if( !load_point_cloud( point_cloud_name, cloud ) )
    {
        return -1;
    };

    copy_point_cloud_xyz_to_rgbd( cloud, cloud_colored );

    LidarPlaneExtractor plane_extractor( cloud_colored );

    plane_extraction( cloud_colored, plane_extractor );



    visualization::PCLVisualizer visualizer( "Visualizer" );
    init_visualizer( visualizer, cloud_colored );

    duration += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    while( !visualizer.wasStopped() )
    {
        visualizer.spinOnce( 100 );
        boost::this_thread::sleep( boost::posix_time::microseconds (100) );
    }

    start = std::clock();

    Mat image;
//    image = imread("../catkin_ws/scene_1/images/left_67_1528404295187234419.jpg", CV_LOAD_IMAGE_COLOR);
    image = imread("../catkin_ws/scene_1/images/left_138_1528404298739645059.jpg", CV_LOAD_IMAGE_COLOR);
//    image = imread("../catkin_ws/scene_1/images/left_30_1528404293335930522.jpg", CV_LOAD_IMAGE_COLOR);
//    image = imread("../catkin_ws/scene_2/myOutput/left_137_1528543256280683879.jpg", CV_LOAD_IMAGE_COLOR);
//    image = imread("../catkin_ws/scene_2/myOutput/left_618_1528543304427714415.jpg", CV_LOAD_IMAGE_COLOR);
    if (!image.data) {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    labelGround(image, plane_extractor.getGroundPointsPixelLocation());
    namedWindow( "Image", WINDOW_AUTOSIZE );
    imshow( "Image", image );

    duration += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    cout << "Loading and processing one scan consumes: " << duration << " secs." << endl;

    waitKey(0);
    return 0;
}
