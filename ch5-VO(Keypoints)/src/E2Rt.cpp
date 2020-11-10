//
// Edited by MagicTZ on 2020/10/23.
// 从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

/**
 * @brief Decompose Essential Matrix E and get four possible solutions [R1,t1^] [R1,t1^] [R2,t2^] [R2,t2^]
 * Using t^R = E to rebuild Essential Matrix
 * Using Sophus::SO3d::vee(t^) to compute vector t
 *
 * @param E
 * @param R1
 * @param R2
 * @param t_wedge1 t1^
 * @param t_wedge2 t2^
 */
void E2Rt(const Matrix3d &E, Matrix3d &R1, Matrix3d &R2, Matrix3d &t_wedge1, Matrix3d &t_wedge2);

int main(int argc, char **argv) {
  // 给定Essential矩阵
  Matrix3d E;
  E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097, 0.3939270778216369, -0.03506401846698079,
      0.5857110303721015, -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

  // 待计算的R,t
  Matrix3d R1;
  Matrix3d R2;
  Matrix3d t_wedge1;
  Matrix3d t_wedge2;

  E2Rt(E, R1, R2, t_wedge1, t_wedge2);

  cout << "R1 = " << R1 << endl;
  cout << "R2 = " << R2 << endl;
  cout << "t1 = " << Sophus::SO3d::vee(t_wedge1) << endl;
  cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

  // check t^R=E up to scale
  Matrix3d tR = t_wedge1 * R1;
  cout << "t^R = " << tR << endl;

  return 0;
}

void E2Rt(const Matrix3d &E, Matrix3d &R1, Matrix3d &R2, Matrix3d &t_wedge1, Matrix3d &t_wedge2) {
  Eigen::JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV);
  Matrix3d V = svd.matrixV();
  Matrix3d U = svd.matrixU();
  // 直接使用内置函数得到奇异值向量
  Vector3d sigma = svd.singularValues();  //得到的是3*1的向量
  // Matrix3d D = U.inverse() * E * V.transpose().inverse();  // compute the diagonal matrix

  cout << "The Singular values vector is: " << endl << sigma << endl;
  // cout << D << endl << D(0, 0) << endl;

  // process the singular value of the matrix D, remove the 3rd component of the singular value, the form is like this:
  // diag((sigma1+sigma2)/2, (sigma1+sigma2)/2, 0)
  Matrix3d DIAG;
  DIAG << (sigma(0) + sigma(1)) / 2, 0, 0, 0, (sigma(0) + sigma(1)) / 2, 0, 0, 0, 0;
  //第一种方法：通过sigma计算
  //  DIAG <<  1, 0, 0
  //                    0, 1, 0
  //                    0, 0, 0
  // Matrix3d DIAG1;
  // DIAG1 << (D(0, 0) + D(1, 1)) / 2, 0, 0, 0, (D(0, 0) + D(1, 1)) / 2, 0, 0, 0, 0;  // 第二种方式
  cout << DIAG << endl;
  // cout<<DIAG1<<endl;

  // Compute Rz(pi/2)
  //  The rotation matrix which rotate pi/2 around axis Z
  Matrix3d Rz_pos = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  Matrix3d Rz_neg = AngleAxisd(-M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();

  // Compute t1^ and t2^
  t_wedge1 = U * Rz_pos * DIAG * U.transpose();
  t_wedge2 = U * Rz_neg * DIAG * U.transpose();

  // R1 and R2
  R1 = U * Rz_pos.transpose() * V.transpose();
  R2 = U * Rz_neg.transpose() * V.transpose();
}