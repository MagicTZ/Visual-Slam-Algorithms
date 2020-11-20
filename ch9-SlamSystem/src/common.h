//
// Created by magictz on 2020/11/20.
//
// 用来存放一些公共的函数

#ifndef MYBA_EIGEN_COMMON_H
#define MYBA_EIGEN_COMMON_H

#include "BAL.h"

template <typename PT, typename C>
Eigen::Vector2d projection(const PT &pt, const C &cameraPose);

/**
 * 向量转反对称矩阵
 * @param vec 3维向量
 * @return matrix3d 反对称矩阵
 */
Eigen::Matrix3d VecToAntiSymmetricMat(Eigen::Vector3d vec);

/**
 * 旋转向量转旋转矩阵
 * @param vec
 * @return R 旋转矩阵3*3
 */
Eigen::Matrix3d VecToMatRotation(Eigen::Vector3d vec);

Eigen::Vector3d MatToVecRotation(Eigen::Matrix3d mat);

#endif //MYBA_EIGEN_COMMON_H
