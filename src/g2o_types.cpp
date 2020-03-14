//
// Created by hc on 2020/3/14.
//

#include <g2o_types.h>

namespace Aruco_LOCA{

    Matrix6d JRInv(const SE3d &e) {
        Matrix6d J;
        J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
        J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
        J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
        J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());
        J = J * 0.5 + Matrix6d::Identity();
        return J;
    }
}