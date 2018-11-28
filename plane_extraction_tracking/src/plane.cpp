#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


#include <iostream>

//void project_lidar_point();
 
cv::Mat colorSeg(const cv::Mat& img)
{
	// convert to HSV
	cv::Mat img_hsv;
	cv::cvtColor( img, img_hsv, CV_BGR2HSV );

    cv::imshow( "window", img_hsv );
    cv::waitKey(0);

    return img_hsv;	
}


// main
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh("~");

    // test
    std::string image_path("/home/wz/Desktop/822/822_test/images/right_0_1528404291835066602.jpg");
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);   // assume BGR now. Make sure RGB or BGR

    std::cout << "test size: " << image.size << std::endl;

	cv::Mat imgSegged = colorSeg(image);
    // do the projection and show results
    //project_lidar_point();

    return 0;
}
