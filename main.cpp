// c++ standard libraries
#include <iostream>

// project helpers
#include "DataStructures.h"

// Point Cloud Library (PCL) supports
#include <pcl/io/pcd_io.h>

using namespace pcl;

using namespace std;

int main( int argc, char** argv ) {
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

    if (io::loadPCDFile<PointXYZ> ("test.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (const auto & point : cloud->points )
    {
        std::cout << "    " << point.x
                  << " "    << point.y
                  << " "    << point.z << std::endl;
    }

    return 0;
}