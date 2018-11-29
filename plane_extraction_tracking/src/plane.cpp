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
    cv::Scalar(0,255,255)
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
    uint row_sz = img_hsv.rows, col_sz = img_hsv.cols, point_sz = row_sz*col_sz; 

    // create a mat, filled with color data
    cv::Mat points = cv::Mat(point_sz,3,CV_32FC2);
    for ( uint i=969; i<point_sz; i++)
    {
        std::vector<uint> res = linearTo2D(row_sz,col_sz,i);
        uint linear = TwDTolinear(row_sz,col_sz,res);
        // TODO

        // ...
    }

    cv::Vec3b test = img_hsv.at<cv::Vec3b>(0,0);

    std::cout << "hsv data H: " << int(test.val[0]) << std::endl;
    std::cout << "hsv data S: " << int(test.val[1]) << std::endl;
    std::cout << "hsv data V: " << int(test.val[2]) << std::endl;

    img_hsv.convertTo(img_hsv, CV_32FC2);
    // k-means
    cv::Mat labels, centers;
    cv::kmeans(img_hsv, 5, labels,
        cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
           3, cv::KMEANS_PP_CENTERS, centers);

    cv::Mat img_seg(img_hsv.rows, img_hsv.cols, CV_8UC3);
    img_seg = cv::Scalar::all(0);
    std::cout << "labels rows: " << labels.rows << std::endl;
    std::cout << "labels cols: " << labels.cols << std::endl; 
    for( int i = 0; i < labels.rows; i++ )
    {
        for ( int j = 0; j < labels.cols; j++ )
        {
            int clusterIdx = labels.at<int>(i,j);
            img_seg.at<cv::Scalar>(i,j) = colorTab[clusterIdx];
            //cv::Point ipt = img_hsv.at<cv::Point2f>(i,j);
            // cv::circle( img_seg, cv::Point(j,i), 2, colorTab[clusterIdx], cv::FILLED, cv::LINE_AA );
        }
    }
    imshow("clusters", img_seg);
    cv::waitKey(0);

    return img_seg;	
}


// main
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh("~");

    // test
    std::string image_path("/home/wz/Desktop/822/822_test/images/right_0_1528404291835066602.jpg");
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);   // assume BGR now. Make sure RGB or BGR
    // std::cout << "size: " << image.size << std::endl;
    // std::cout << "dims: " << image.channels() << std::endl;
	cv::Mat imgSegged = colorSeg(image);

    return 0;
}
