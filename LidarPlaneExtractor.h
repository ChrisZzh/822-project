//
// Created by chris on 10/17/18.
//

#ifndef INC_16822_PROJECT_LIDARPLANEEXTRACTOR_H
#define INC_16822_PROJECT_LIDARPLANEEXTRACTOR_H

#include "DataStructures.h"

using namespace Eigen;
using namespace std;

class LidarPlaneExtractor {

protected:

    /* pointer to the accumulated point cloud contained in one scan */
    Vector3d* point_cloud = nullptr;

    vector<vector<Vector3d>> point_cloud_in_cells = {};

    /* labels of the points: ground, plane1, plane2, etc. */
    vector<int> labels = {};

    LidarPlaneExtractor() = default;

    ~LidarPlaneExtractor() = default;

    LidarPlaneExtractor( Vector3d* point_cloud );

    void put_points_in_cells();

    void find_ground_plane_in_each_cell();

};


#endif //INC_16822_PROJECT_LIDARPLANEEXTRACTOR_H
