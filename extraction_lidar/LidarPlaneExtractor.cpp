//
// Created by chris on 10/17/18.
//

#include "LidarPlaneExtractor.h"
#include <iostream>
#include <cmath>

using namespace std;

LidarPlaneExtractor::LidarPlaneExtractor(pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_point_cloud)
{
    cout << "Constructor of LidarPlaneExtractor called." << endl;

    assert( nullptr != accumulated_point_cloud );
    point_cloud = accumulated_point_cloud;

    num_of_cells_per_row_and_column = static_cast<unsigned long>( MAX_X_Y_RANGE_METERS * 2.0f / CELL_RESOLUTION_METERS );
    num_of_grid_cells = num_of_cells_per_row_and_column * num_of_cells_per_row_and_column;

    grid_cell.resize( num_of_grid_cells );

    ground_height_of_each_cell.resize( num_of_grid_cells );

    labels.resize( NUM_POINTS_PER_SCAN );

}

void LidarPlaneExtractor::putPointsInCells() {

    assert( point_cloud != nullptr );
    assert( grid_cell.empty() );

    for( size_t i = 0; i < NUM_POINTS_PER_SCAN; i++ )
    {
        auto x = point_cloud->points[i].x;
        auto y = point_cloud->points[i].y;
        if( ( x < -MAX_X_Y_RANGE_METERS && x > MAX_X_Y_RANGE_METERS )
            ||
            ( y < -MAX_X_Y_RANGE_METERS && y > MAX_X_Y_RANGE_METERS ) )
        {
            continue;
        }
        auto row = findIndex( y );
        auto col = findIndex( x );
        grid_cell[col + num_of_cells_per_row_and_column * row].push_back( i );
    }

}

size_t LidarPlaneExtractor::findIndex( double coordinate ) const {
    coordinate += MAX_X_Y_RANGE_METERS;
    return static_cast<size_t>( floor(coordinate / CELL_RESOLUTION_METERS ) );
}

void LidarPlaneExtractor::find_ground_plane_in_each_cell() {
    size_t cell_id = 0;
    for( const auto & cell : grid_cell )
    {
        ground_height_of_each_cell[cell_id] = findGroundHeight( cell );

        cell_id++;
    }
}

float LidarPlaneExtractor::findGroundHeight(const vector<size_t> &cell) {
    float min_ground_height = 100.f;
    for( const auto point_index : cell )
    {
        auto z = point_cloud->points[point_index].z;
        if( min_ground_height > z )
        {
            min_ground_height = z;
        }
    }
    if( fabs( min_ground_height - 100.f ) <= EPSILON )
    {
        cerr << "minumum ground height = 100 meters" << endl;
    }
    return min_ground_height;
}

void LidarPlaneExtractor::labelGroundAndNonGroundPoints() {
    assert( num_of_grid_cells == ground_height_of_each_cell.size() );
    assert( num_of_grid_cells == grid_cell.size() );
    for( size_t i = 0; i < num_of_grid_cells; i++ )
    {
        auto ground_height = ground_height_of_each_cell[i];
        for( auto & point_index : grid_cell[i] )
        {
            if( point_cloud->points[point_index].z > ground_height + GROUND_REGION_HEIGHT )
            {
                labels[point_index] = PointTypes::NON_GROUND;
            }
            else
            {
                labels[point_index] = PointTypes::GROUND;
            }
        }
    }
}
