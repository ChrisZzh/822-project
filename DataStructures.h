//
// Created by chris on 10/17/18.
//

#ifndef INC_16822_PROJECT_DATASTRUCTURES_H
#define INC_16822_PROJECT_DATASTRUCTURES_H

// Eigen
#include "Eigen/Dense"

// Point Cloud Library
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using namespace std;

// grid plane properties

static constexpr size_t NUM_OF_CELL_PER_ROW = 50;

static constexpr size_t NUM_OF_CELL_PER_COL = 50;

// ground separation properties

static constexpr float MAX_GROUND_HEIGHT_METERS = 0.1f;

static constexpr float GROUND_REGION_HEIGHT = 0.1f;

// Lidar properties

//static constexpr size_t NUM_POINTS_PER_SCAN = 64 * 3480;

static constexpr float MAX_X_Y_RANGE_METERS = 5.0f;

static constexpr float CELL_RESOLUTION_METERS = 1.0f;

enum class PointTypes
{
    GROUND, NON_GROUND
};

// for floating point values comparison
static constexpr float EPSILON = 0.000001f;

#endif //INC_16822_PROJECT_DATASTRUCTURES_H
