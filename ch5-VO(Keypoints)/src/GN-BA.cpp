//
// Edited by MagicTZ on 2020/10/23.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./../p3d.txt";
string p2d_file = "./../p2d.txt";

int main(int argc, char **argv) {
  VecVector2d p2d;
  VecVector3d p3d;
  Matrix3d K;  // Camera intrinsic parameter matrix
  double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
  K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

  // load points in to p3d and p2d
  ifstream if_p3d(p3d_file);
  ifstream if_p2d(p2d_file);

  if (!if_p2d || !if_p3d) {
    cout << "Empty file or files are not right loaded!" << endl;
  }

  string line;
  //   load points into p3d
  while (getline(if_p3d, line)) {
    Vector3d point3d;
    stringstream s(line);
    while (s >> point3d[0] >> point3d[1] >> point3d[2]) {
    }                        // stringstream 默认按空格分隔字符串
                             // cout << point3d << endl;
    p3d.push_back(point3d);  //将点存进p3d的容器中
  }

  //   load points into p2d
  while (getline(if_p2d, line)) {
    Vector2d point2d;
    stringstream s(line);
    while (s >> point2d[0] >> point2d[1]) {
    }
    // cout<<point2d<<endl;
    p2d.push_back(point2d);
  }

  assert(p3d.size() == p2d.size());

  int iterations = 100;
  double cost = 0, lastCost = 0;
  int nPoints = p3d.size();
  cout << "points: " << nPoints << endl;

  Sophus::SE3d T_esti;  // estimated pose

  for (int iter = 0; iter < iterations; iter++) {
    Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
    Vector6d b = Vector6d::Zero();

    cost = 0;
    // compute cost
    for (int i = 0; i < nPoints; i++) {
      // compute cost for p3d[I] and p2d[I]

      // 计算每个三维点的重投影
      Vector3d p3d_obs = T_esti * p3d[i];
      Vector2d p2d_obs(fx * p3d_obs[0] / p3d_obs[2] + cx, fy * p3d_obs[1] / p3d_obs[2] + cy);
      //   计算误差值 2*1
      Vector2d e = p2d[i] - p2d_obs;
      //cout << "原始坐标： " << p2d[i].transpose() << "重投影坐标：" << p2d_obs.transpose() << endl;
      // 计算累积重投影误差
      cost += (e[0]*e[0] + e[1]*e[1]);

      // compute jacobian
      Matrix<double, 2, 6> J;
      double inv_z = 1.0 / p3d_obs[2];
      double inv_z2 = inv_z * inv_z;
      double x = p3d_obs[0], y = p3d_obs[1];
      J << -fx * inv_z, 0, fx * x * inv_z2, fx * x * y * inv_z2, -fx - fx * x * x * inv_z2, fx * y * inv_z, 0,
          -fy * inv_z, fy * y * inv_z2, fy + fy * y * y * inv_z2, -fy * x * y * inv_z2, -fy * x * inv_z;

      H += J.transpose() * J;
      b += -J.transpose() * e;
    }

    // solve dx
    Vector6d dx;

    dx = H.ldlt().solve(b);

    if (isnan(dx[0])) {
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      // cost increase, update is not good
      cout << "cost: " << cost << ", last cost: " << lastCost << endl;
      break;
    }

    // update your estimation
    // START YOUR CODE HERE
    T_esti = Sophus::SE3d::exp(dx) * T_esti;
    // END YOUR CODE HERE

    lastCost = cost;

    cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;

    if (dx.norm() < 1e-8) {
      // converge
      cout << "Converge earlier!!" << endl;
      break;
    }
  }

  cout << "estimated pose: \n" << T_esti.matrix() << endl;
  return 0;
}
