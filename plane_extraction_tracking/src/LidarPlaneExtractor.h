//
// Created by chris on 10/17/18.
//

#ifndef INC_16822_PROJECT_LIDARPLANEEXTRACTOR_H
#define INC_16822_PROJECT_LIDARPLANEEXTRACTOR_H

#include "DataStructures.h"


using namespace Eigen;

using namespace std;

using namespace pcl;

class LidarPlaneExtractor {

protected:

    /* pointer to the accumulated point cloud contained in one scan */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;

    vector<vector<size_t>> grid_cell = {};

    vector<float> ground_height_of_each_cell = {};

    size_t num_of_cells_per_row_and_column = 0;

    size_t num_of_grid_cells = 0;

    size_t num_points_per_scan = 0;

    /* labels of the points: ground, plane1, plane2, etc. */
    vector<PointTypes> labels = {};

    PlaneInfo ground = {};

    MatrixXd ground_points_lidar = {};

    MatrixXd ground_points_camera = {};

    MatrixXd ground_points_pixel_location = {};

    MatrixXd mean = {};

public:

    LidarPlaneExtractor() = default;

    ~LidarPlaneExtractor() = default;

    explicit LidarPlaneExtractor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

    void putPointsInCells();

    void find_ground_plane_in_each_cell();

    void labelGroundAndNonGroundPoints();

    void computePlaneParameters();

    void findPlaneInImage();

    const MatrixXd &getGroundPointsPixelLocation() const { return ground_points_pixel_location; };

// helpers
protected:
    size_t findIndex( double coordinate ) const;

    float findGroundHeight( const vector<size_t> & cell ) const;

    void updatePlaneInfo(const JacobiSVD<MatrixXd> &svd);

    void transformGroundPointsLtoC();

    MatrixXd assembleTransformation(const Quaterniond &q, const Translation3d &t) const;

    void cameraProjection();

    MatrixXd assembleCameraIntrinsic() const;
};


#endif //INC_16822_PROJECT_LIDARPLANEEXTRACTOR_H