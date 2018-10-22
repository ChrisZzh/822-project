//
// Created by chris on 10/17/18.
//

#ifndef INC_16822_PROJECT_DATASTRUCTURES_H
#define INC_16822_PROJECT_DATASTRUCTURES_H

#include "Eigen/Dense"

using namespace std;

// grid plane properties

static constexpr size_t NUM_OF_CELL_PER_ROW = 50;

static constexpr size_t NUM_OF_CELL_PER_COL = 50;

// ground separation properties

static constexpr float MAX_GROUND_HEIGHT = 0.1f;

// Lidar properties

static constexpr size_t NUM_POINTS_PER_SCAN = 64 * 3480;

static constexpr float MAX_X_RANGE = 120.f;

static constexpr float MAX_Y_RANGE = 120.f;


#endif //INC_16822_PROJECT_DATASTRUCTURES_H
