#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>

typedef pcl::PointCloud< pcl::PointXYZI > PointCloudI;

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
	cv::Mat_<double> homo(3,3);
	cv::Mat_<double> K(3,3);
	homo << 0.003192301358473, 0.001081524315727, -0.999774880519790,
            0.000025666856680, 0.002724831087490, -0.020709546919980,
            0.000002269426529, 0.000000632177580,  0.001586535977295;
	K << 498.1357145, 0.0, 351.726944, 0.0, 498.1357145, 255.9642885, 0.0, 0.0, 1;

	cv::Mat homography = K * homo * K.inv();
	homography /= homography.at<double>(2,2);

    std::vector<cv::Mat> Rs_decomp, ts_decomp, normals_decomp;
    int solutions = cv::decomposeHomographyMat(homography, K, Rs_decomp, ts_decomp, normals_decomp);
    std::cout << "Decompose homography matrix computed from the camera displacement:\n\n";
    for (int i = 0; i < solutions; i++)
    {
      //double factor_d1 = 1.0 / d_inv1;
      cv::Mat rvec_decomp;
      cv::Rodrigues(Rs_decomp[i], rvec_decomp);
      std::cout << "Solution " << i << ":" << std::endl;
      std::cout << "rvec from homography decomposition: " << rvec_decomp.t() << std::endl;
      std::cout << "tvec from homography decomposition: " << ts_decomp[i].t() << std::endl;;//<< " and scaled by d: " << factor_d1 * ts_decomp[i].t() << std::endl;
      std::cout << "plane normal from homography decomposition: " << normals_decomp[i].t() << std::endl;
      //cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
    }
	
	// eigen solver n d

	return;
}

void camBased_callback(const sensor_msgs::Image::ConstPtr &img_msg_L,
                                        const sensor_msgs::Image::ConstPtr &img_msg_R,
                                        const nav_msgs::Odometry::ConstPtr &odom_msg) {}
// main
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh("~");

    img_sub.subscribe(nh, "/mapping/left/image_rect_color", 100);
    odom_sub.subscribe(nh, "/smart_smoother/odom_camera", 100);
    point_cloud_sub.subscribe(nh,"/feature_points", 10);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry, PointCloudI> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), img_sub_L, img_sub_R, odom_sub);
    sync.registerCallback(boost::bind(&camBased_callback, this, _1, _2, _3));


    // std::string image_path_1("/home/wz/Desktop/822_test/images/right_0_1528404291835066602.jpg");
    // cv::Mat image_1 = cv::imread(image_path_1, CV_LOAD_IMAGE_COLOR);   // assume BGR now. Make sure RGB or BGR

    // std::string image_path_2("/home/wz/Desktop/822_test/images/right_1_1528404291885616260.jpg");
    // cv::Mat image_2 = cv::imread(image_path_2, CV_LOAD_IMAGE_COLOR);   // assume BGR now. Make sure RGB or BGR

    // plane_extraction(image_1, image_2);
    return 0;
}
