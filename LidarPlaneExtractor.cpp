//
// Created by chris on 10/17/18.
//

#include "LidarPlaneExtractor.h"

LidarPlaneExtractor::LidarPlaneExtractor(Vector3d * accumulated_point_cloud)
{
    assert( nullptr != accumulated_point_cloud );
    point_cloud = accumulated_point_cloud;
}