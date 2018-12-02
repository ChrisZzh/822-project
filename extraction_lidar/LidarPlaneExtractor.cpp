//
// Created by chris on 10/17/18.
//

#include "LidarPlaneExtractor.h"
#include <iostream>
#include <cmath>

using namespace std;

LidarPlaneExtractor::LidarPlaneExtractor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_point_cloud)
{
    cout << "Constructor of LidarPlaneExtractor called." << endl;

    assert( nullptr != accumulated_point_cloud );
    point_cloud = accumulated_point_cloud;

    num_of_cells_per_row_and_column = static_cast<unsigned long>( MAX_X_Y_RANGE_METERS * 2.0f / CELL_RESOLUTION_METERS );
    num_points_per_scan = accumulated_point_cloud->points.size();
    num_of_grid_cells = num_of_cells_per_row_and_column * num_of_cells_per_row_and_column;

    grid_cell.resize( num_of_grid_cells );

    ground_height_of_each_cell.resize( num_of_grid_cells );

    labels.resize( num_points_per_scan );

}

void LidarPlaneExtractor::putPointsInCells() {

    assert( point_cloud != nullptr );
    assert( grid_cell.size() );

    for( size_t i = 0; i < num_points_per_scan; i++ )
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
    auto new_coordinate = floor(coordinate / CELL_RESOLUTION_METERS );
    if (coordinate < 0.f) {
        cout << "y < 0: " << coordinate << endl;
    }
    assert(new_coordinate >= 0.f);
    return static_cast<size_t>( new_coordinate );
}

void LidarPlaneExtractor::find_ground_plane_in_each_cell() {
    size_t cell_id = 0;
    for( const auto & cell : grid_cell )
    {
        ground_height_of_each_cell[cell_id] = findGroundHeight( cell );
//        cout << "cell id: " << cell_id << " height = " << ground_height_of_each_cell[cell_id] << endl;
        cell_id++;
    }
}

float LidarPlaneExtractor::findGroundHeight(const vector<size_t> &cell) const{
    float min_ground_height = 100.f;
    bool found = false;
    for( const auto point_index : cell )
    {
        auto z = point_cloud->points[point_index].z;
        if( min_ground_height > z )
        {
            min_ground_height = z;
            found = true;
        }
    }
    if( !found )
    {
        cerr << "LidarPlaneExtractor::findGroundHeight: not found." << endl;
    }
    return min_ground_height;
}

void LidarPlaneExtractor::labelGroundAndNonGroundPoints() {
    assert( num_of_grid_cells == ground_height_of_each_cell.size() );
    assert( num_of_grid_cells == grid_cell.size() );

    vector<size_t> ground_points_indices = {};
    for( size_t i = 0; i < num_of_grid_cells; i++ )
    {
        auto ground_height = ground_height_of_each_cell[i];
        for( auto & point_index : grid_cell[i] )
        {
            if( point_cloud->points[point_index].z > ground_height + GROUND_REGION_HEIGHT )
            {
                // label non-ground points as green
                point_cloud->points[point_index].r = 0;
                point_cloud->points[point_index].g = 255;
//                labels[point_index] = PointTypes::NON_GROUND;
            }
            else
            {
                // label ground points as red
                point_cloud->points[point_index].r = 255;
                point_cloud->points[point_index].g = 0;
                ground_points_indices.push_back(point_index);
//                labels[point_index] = PointTypes::GROUND;
            }
        }
    }
    ground_points_lidar.resize(ground_points_indices.size(), 3);
    size_t row = 0;
    double sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;
    for (const auto index : ground_points_indices) {
        ground_points_lidar(row, 0) = point_cloud->points[index].x;
        sum_x += point_cloud->points[index].x;
        ground_points_lidar(row, 1) = point_cloud->points[index].y;
        sum_y += point_cloud->points[index].y;
        ground_points_lidar(row, 2) = point_cloud->points[index].z;
        sum_z += point_cloud->points[index].z;
        row++;
    }
    sum_x /= ground_points_indices.size();
    sum_y /= ground_points_indices.size();
    sum_z /= ground_points_indices.size();
    mean = MatrixXd(ground_points_indices.size(), 3);
    mean << sum_x * VectorXd::Ones(ground_points_indices.size())
            , sum_y * VectorXd::Ones(ground_points_indices.size())
            , sum_z * VectorXd::Ones(ground_points_indices.size());
}

void LidarPlaneExtractor::computePlaneParameters() {
    MatrixXd normalized_ground_points = ground_points_lidar - mean;
    MatrixXd cov_mat = normalized_ground_points.transpose() * normalized_ground_points;
    JacobiSVD<MatrixXd> svd(cov_mat, ComputeFullU | ComputeFullV);
    updatePlaneInfo(svd);
}

void LidarPlaneExtractor::updatePlaneInfo(const JacobiSVD<MatrixXd> &svd) {
    size_t smallest_singular_value_index = 3;
    double smallest_singular_value = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 3; i++) {
        if (smallest_singular_value > svd.singularValues()[i]) {
            smallest_singular_value_index = i;
            smallest_singular_value = svd.singularValues()[i];
        }
    }
    assert(smallest_singular_value_index != 3);
    ground.normal = svd.matrixU().col(smallest_singular_value_index);
    cout << "normal vector of ground plane: " << endl << ground.normal << endl;
    VectorXd d = ground_points_lidar * ground.normal;
    ground.displacement = d.mean();
    cout << "displacement of ground plane: " << endl << ground.displacement << endl;
}

MatrixXd LidarPlaneExtractor::assembleTransformation(const Quaterniond &q, const Translation3d &t) const {
    MatrixXd T(4,4);
    T.setIdentity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = t.vector();
//    cout << T << endl;
    return T;
}

void LidarPlaneExtractor::findPlaneInImage() {
    transformGroundPointsLtoC();
    cameraProjection();
}

void LidarPlaneExtractor::transformGroundPointsLtoC() {
    // extrinsic matrices from IMU to camera/LiDAR
    Quaterniond q_I_C(0.499079072301088f, -0.505950871509703f, -0.497347934968811f, 0.497572936152999f);
    Translation3d t_I_C(Vector3d(-0.13537f, -0.11358f, 0.015839f));
    auto T_I_C = assembleTransformation(q_I_C, t_I_C);

    // extrinsic matrices from IMU to world for camera
    Quaterniond q_I_O_camera(0.016494677694310656835f, -0.22574939817782699314f, 0.022210423637006669606f, 0.97379249941246470712f);
    Translation3d t_I_O_camera(Vector3d(-0.44890212813470076192f, 0.03871945017262484745f, -0.0081686908410265879343f));
    auto T_I_O = assembleTransformation(q_I_O_camera, t_I_O_camera);

    MatrixXd tmp_ground_points_camera(ground_points_lidar.rows(), ground_points_lidar.cols() + 1);
    tmp_ground_points_camera << ground_points_lidar, VectorXd::Ones(ground_points_lidar.rows());
    ground_points_camera = (T_I_C.inverse() * T_I_O * tmp_ground_points_camera.transpose()).transpose();
    for (size_t row = 0; row < ground_points_lidar.rows(); row++) {
        ground_points_camera.row(row) = ground_points_camera.row(row) / ground_points_camera(row, 3);
    }
}

void LidarPlaneExtractor::cameraProjection() {
    MatrixXd K = assembleCameraIntrinsic();
    ground_points_pixel_location = (K * ground_points_camera.transpose()).transpose();
    for (size_t row = 0; row < ground_points_camera.rows(); row++) {
        ground_points_pixel_location.row(row) = ground_points_pixel_location.row(row) / ground_points_pixel_location(row, 2);
    }
//    cout << ground_points_pixel_location << endl;
}

MatrixXd LidarPlaneExtractor::assembleCameraIntrinsic() const {
    MatrixXd K(3,4);
    K << fx, 0.f, cx, 0.f
        , 0.f, fy, cy, 0.f
        , 0.f, 0.f, 1.f, 0.f;
    return K;
}


