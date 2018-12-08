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
                point_cloud->points[point_index].r = 150;
                point_cloud->points[point_index].g = 0;
            }
            else
            {
                point_cloud->points[point_index].r = 0;
                point_cloud->points[point_index].g = 255;
                ground_points_indices.push_back(point_index);
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
    auto T_C_I = assembleTransformation(q_I_C, t_I_C);

//    // extrinsic matrices from IMU to camera world
//    Quaterniond q_I_O_camera(0.97379249941246470712f, 0.016494677694310656835f, -0.22574939817782699314f, 0.022210423637006669606f);
//    Translation3d t_I_O_camera(Vector3d(-0.44890212813470076192f, 0.03871945017262484745f, -0.0081686908410265879343f));
//    auto T_I_O_cam = assembleTransformation(q_I_O_camera, t_I_O_camera);
//
//    // extrinsic matrices from IMU to LiDAR world
//    Quaterniond q_I_O_lidar(0.973729454714922f, 0.01769039122573788f, -0.2261052466702405f, 0.02035722247878263f);
//    Translation3d t_I_O_lidar(Vector3d(-0.4502058634836024f, 0.02398935409709984f, -0.00151897710861594f));
//    auto T_I_O_lidar = assembleTransformation(q_I_O_lidar, t_I_O_lidar);

    // extrinsic matrices from IMU to camera world
    Quaterniond q_I_O_camera(0.97399948023159754751f, 0.0060250375937415927979f, -0.22577667468113121751f, -0.017708884789934188631f);
    Translation3d t_I_O_camera(Vector3d(-3.1387316334198058776f, 0.26906105374385391737f, 0.0069626112659987851247f));
    auto T_I_O_cam = assembleTransformation(q_I_O_camera, t_I_O_camera);

    // extrinsic matrices from IMU to LiDAR world
    Quaterniond q_I_O_lidar(0.9742605554849183f, 0.008888275035702423f, -0.2247186173041979f, -0.01545676647937188f);
    Translation3d t_I_O_lidar(Vector3d(-3.266430592029979f, 0.2188575629109428f, 0.02326167409746047f));
    auto T_I_O_lidar = assembleTransformation(q_I_O_lidar, t_I_O_lidar);

//    // extrinsic matrices from IMU to camera world
//    Quaterniond q_I_O_camera(0.78995810436514446451f, 0.13836578452231249048f, -0.18253557440092629816f, 0.56877224536670167865f);
//    Translation3d t_I_O_camera(Vector3d(-10.239143036573635115f, 0.87149041046356989781f, -0.022001021595639710243f));
//    auto T_I_O_cam = assembleTransformation(q_I_O_camera, t_I_O_camera);
//
//    // extrinsic matrices from IMU to LiDAR world
//    Quaterniond q_I_O_lidar(0.7960698900379954f, 0.1364074164281219f, -0.1812260756921947f, 0.5610907737679401f);
//    Translation3d t_I_O_lidar(Vector3d(-10.31616196497376f, 0.733600654462199f, 0.0540135676578862f));
//    auto T_I_O_lidar = assembleTransformation(q_I_O_lidar, t_I_O_lidar);

//    // extrinsic matrices from IMU to camera world
//    Quaterniond q_I_O_camera(0.78693478681640960382f, 0.14014795846479677355f, -0.18097161553942892054f, 0.57301087721387211626f);
//    Translation3d t_I_O_camera(Vector3d(-3.6365926623709867727f, -0.50092593699118237449f, -0.016328461018303045554f));
//    auto T_I_O_cam = assembleTransformation(q_I_O_camera, t_I_O_camera);
//
//    // extrinsic matrices from IMU to LiDAR world
//    Quaterniond q_I_O_lidar(0.7980270657340848f, 0.1392954775680357f, -0.1836968017901212f, 0.5567809778512737f);
//    Translation3d t_I_O_lidar(Vector3d(-3.708476202021089f, -0.5381995096949547f, 0.03146552372088337f));
//    auto T_I_O_lidar = assembleTransformation(q_I_O_lidar, t_I_O_lidar);

//    // extrinsic matrices from IMU to camera world
//    Quaterniond q_I_O_camera(0.9059524645188308245f, -0.076457688329491638179f, -0.22443160123385019755f, -0.35076318263919575857f);
//    Translation3d t_I_O_camera(Vector3d(-21.114414690557083532f, -3.9903353270451700219f, -0.091603466765364877089f));
//    auto T_I_O_cam = assembleTransformation(q_I_O_camera, t_I_O_camera);
//
//    // extrinsic matrices from IMU to LiDAR world
//    Quaterniond q_I_O_lidar(0.9026178872343066f, -0.07888020917187256f, -0.2065189910426705f, -0.3693355771982921f);
//    Translation3d t_I_O_lidar(Vector3d(-20.87744519129228f, -4.438633356847971f, 0.1682614346989345f));
//    auto T_I_O_lidar = assembleTransformation(q_I_O_lidar, t_I_O_lidar);

    // extrinsic matrix from IMU to LiDAR
    MatrixXd T_from_lidar_to_IMU(4,4);
    T_from_lidar_to_IMU << 0.f, -1.f, 0.f, -0.028030000000000f,
            1.f,  0.f, 0.f,  0.034960000000000f,
            0.f,  0.f, 1.f, -0.087869000000000f,
            0.f,  0.f, 0.f, 1.f;

    auto T_C_O = T_I_O_cam * T_C_I;
//    auto transformationFromLidarToCamera = T_C_I.inverse() * T_I_O_cam;
//    cout << T_C_O << endl;

    MatrixXd tmp_ground_points_camera(ground_points_lidar.rows(), ground_points_lidar.cols() + 1);
    tmp_ground_points_camera << ground_points_lidar, VectorXd::Ones(ground_points_lidar.rows());
    ground_points_camera = (T_C_O.inverse() * tmp_ground_points_camera.transpose()).transpose();
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


