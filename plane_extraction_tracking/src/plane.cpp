#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <iostream>

cv::Scalar colorTab[] =
{
    cv::Scalar(0, 0, 255),
    cv::Scalar(0,255,0),
    cv::Scalar(255,100,100),
    cv::Scalar(255,0,255),
    cv::Scalar(0,255,255),
    cv::Scalar(255,0,0),
    // cv::Scalar(100,100,255),
    // cv::Scalar(100,255,100),
};

// row first 
std::vector<uint> linearTo2D(uint row, uint col, uint l)
{
    std::vector<uint> TwD_index;
    uint row_ind = floor(float(l)/float(col));
    uint col_ind = l - row_ind*col;
    TwD_index.push_back(row_ind);
    TwD_index.push_back(col_ind);
    return TwD_index;
}

// row first 
uint TwDTolinear(uint row, uint col, std::vector<uint> TwD_index)
{
    return TwD_index[0] * col + TwD_index[1];
}

cv::Mat colorSeg(const cv::Mat& img)
{
	// convert to HSV
	cv::Mat img_hsv;
	cv::cvtColor( img, img_hsv, CV_BGR2HSV );
    // show BGR image
    cv::imshow("orignal", img);
    // show HSV image
    cv::imshow( "hsv", img_hsv );
    //
    std::cout << "image rows: " << img_hsv.rows << "\n";
    std::cout << "image cols: " << img_hsv.cols << "\n";

    uint row_sz = img_hsv.rows, col_sz = img_hsv.cols, point_sz = row_sz*col_sz; 

    // create a mat, filled with color data
    cv::Mat points = cv::Mat(point_sz,3,CV_32F);
    for ( uint i=0; i<point_sz; i++)
    {
        std::vector<uint> res = linearTo2D(row_sz,col_sz,i);
        cv::Vec3b temp = img_hsv.at<cv::Vec3b>(res[0],res[1]);
	    points.at<float>(i,0) = float(temp.val[0]);
	    points.at<float>(i,1) = float(temp.val[1]);
	    points.at<float>(i,2) = float(temp.val[2]);
    }
    // k-means
    cv::Mat labels, centers; // labels of size (point_sz,1)
    cv::kmeans(points, 6
    	, labels,
        cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
           3, cv::KMEANS_PP_CENTERS, centers);

    cv::Mat img_seg(img_hsv.rows, img_hsv.cols, CV_8UC3);
    img_seg = cv::Scalar::all(0); 
    for( uint i = 0; i < img_seg.rows; i++ )
    {
        for ( uint j = 0; j < img_seg.cols; j++ )
        {
        	// retrive cluster ID
        	uint ind_linear = TwDTolinear(row_sz, col_sz, std::vector<uint>{i,j});
            int clusterID = labels.at<int>(ind_linear);
            // plot with cluster color
            cv::Point ipt = cv::Point(j,i);
            cv::circle( img_seg, ipt, 2, colorTab[clusterID], cv::FILLED, cv::LINE_AA );
        }
    }
    imshow("clusters", img_seg);
    cv::waitKey(0);

    return labels;	
}

// estimate homography
void plane_extraction(cv::Mat& image_1, cv::Mat image_2) 
{
	cv::Mat labels_1 = colorSeg(image_1);
	cv::Mat labels_2 = colorSeg(image_2);

	// extract features in the two images, return descriptors as well 
	// filter out points on the major plane (ground plane)

	// find homography use this set

	// eigen solver n d

	return;
}

// main
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh("~");

    std::string image_path_1("/home/wz/Desktop/822/822_test/images/right_0_1528404291835066602.jpg");
    cv::Mat image_1 = cv::imread(image_path_1, CV_LOAD_IMAGE_COLOR);   // assume BGR now. Make sure RGB or BGR

    std::string image_path_2("/home/wz/Desktop/822/822_test/images/right_1_1528404291885616260.jpg");
    cv::Mat image_2 = cv::imread(image_path_2, CV_LOAD_IMAGE_COLOR);   // assume BGR now. Make sure RGB or BGR

    plane_extraction(image_1, image_2);
    return 0;
}
