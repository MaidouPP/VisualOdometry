#ifndef VO_HPP
#define VO_HPP

#include "common.hpp"
#include "orb.hpp"

namespace rgbd_vo {

// 2D point -> 3D point
// input is 3D is because the 3rd one is depth.
cv::Point3f inline point2DTo3D(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera) {
    cv::Point3f p; // allocentric, camera coordinate
    p.z = double(point.z)/camera.scale;
    p.x = (point.x - camera.cx)*p.z/camera.fx;
    p.y = (point.y - camera.cy)*p.z/camera.fy;
    return p;
}

PnP poseEstimate( FRAME& f1, FRAME& f2, CAMERA_INTRINSIC_PARAMETERS& camera, orbFeature& orb);

}

#endif // VO_HPP

/***
 * author: Shixin Li @ SCU
 * date of create: 10/02/2016
***/
