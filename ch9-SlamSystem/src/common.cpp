//
// Created by magictz on 2020/11/20.
//

#include "common.h"

template<typename PT, typename C>
Eigen::Vector2d projection(const PT &pt, const C &cameraPose) {
    Eigen::Vector2d proj = Eigen::Vector2d::Identity();


    return proj;
}

Eigen::Matrix3d VecToAntiSymmetricMat(Eigen::Vector3d vec) {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    mat << 0, -vec[0], vec[1],
            vec[0], 0, -vec[2],
            -vec[1], vec[2], 0;

    return mat;
}

Eigen::Matrix3d VecToMatRotation(Eigen::Vector3d vec){
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();

    double n_norm = vec.norm(); // 得到n
    Eigen::AngleAxisd rotation_vec(n_norm, vec/n_norm);

    rotation = rotation_vec.toRotationMatrix();
    return rotation;
}

Eigen::Vector3d MatToVecRotation(Eigen::Matrix3d R) {
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    Eigen::Vector3d w((R(2, 1) - R(1, 2)) / 2,
                      (R(0, 2) - R(2, 0)) / 2,
                      (R(1, 0) - R(0, 1)) / 2);
    const double costheta = (tr - 1.0) * 0.5;
    if (costheta > 1 || costheta < -1) {
        return w;
    }
    const double theta = acos(costheta);
    const double s = sin(theta);
    if (fabs(s) < 1e-4) {
        return w;
    } else {
        return (theta * w / s);
    }
}