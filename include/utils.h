#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Matrix3d quat2rotm(
    const double& q_w,
    const double& q_x,
    const double& q_y,
    const double& q_z
){
    Eigen::Quaterniond q (q_w, q_x, q_y, q_z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    
    return R;
}

#endif //UTILS_H