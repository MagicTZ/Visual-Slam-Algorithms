//
// Created by magictz on 2020/11/7.
//

#include "BAL.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVecRef;  //作用是什么?

// 中值查找,返回的是中值的索引
double Median(std::vector<double> *data) {
  int n = data->size();
  std::vector<double>::iterator mid_point = data->begin() + n / 2;
  std::nth_element(data->begin(), mid_point, data->end());
  return *mid_point;
}

BAL::BAL(const string &filename) {
  ifstream file(filename);

  if (!file) {
    std::cerr << "Error: unable to open the file" << endl;
    return;
  }

  string line;
  size_t row = 0;
  int num_1 = 0, num_2 = 0;
  while (getline(file, line)) {
    stringstream ss(line);  // 对每一行进行划分
    // 读取第一行数据
    if (row == 0) {
      ss >> num_camera_ >> num_pts_ >> num_obs_;
      cout << "num_cameras: " << num_camera_ << " num_points: " << num_pts_ << " num_obs_" << num_obs_ << endl;
      num_1 = num_obs_ + num_camera_ * 9;
      num_2 = num_obs_ + num_camera_ * 9 + num_pts_ * 3;
      // 给数组分配内存空间
      point_index_ = new int[num_obs_];
      camera_index_ = new int[num_obs_];
      obs_x_ = new double[num_obs_];
      obs_y_ = new double[num_obs_];

      camera_para_ = new double[9 * num_camera_];
      pts_ = new double[3 * num_pts_];
    }

    // 根据观测值数量读取第二类数据(相机索引 点索引 观测值x,y)
    if (row > 0 && row <= num_obs_) {
      ss >> camera_index_[row - 1] >> point_index_[row - 1] >> obs_x_[row - 1] >> obs_y_[row - 1];
      // cout<<camera_index_[row-1]<<point_index_[row-1]<<obs_x_[row-1]<<obs_y_[row-1]<<endl; // 检查
    }

    // 读取相机内外参数
    if (row > num_obs_ && row <= num_1) {
      ss >> camera_para_[row - num_obs_ - 1];
    }

    // 读取三维点数据
    if (row > num_1 && row <= num_2) {
      ss >> pts_[row - num_1 - 1];
    }
    row++;
  }

  // 旋转向量通过罗德里格斯公式将3*1向量转换成3*3矩阵
}

// 归一化处理,使原始数据的光心置零,并按照一定比例进行缩放
void BAL::Normalize() {
  vector<double> tmp(num_pts_);
  Eigen::Vector3d median;
  double *points = pts_add();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < num_pts_; j++) {
      tmp[j] = points[j * 3 + i];
    }
    median[i] = Median(&tmp);
  }

  for (int i = 0; i < num_pts_; ++i) {
    VectorRef point(points + 3 * i, 3);
    tmp[i] = (point - median).lpNorm<1>();
  }

  const double median_absolute_deviation = Median(&tmp);

  // Scale so that the median absolute deviation of the resulting reconstruction is 100

  const double scale = 100.0 / median_absolute_deviation;

  // X = scale * (X -median)
  for (int i = 0; i < num_pts_; i++) {
    VectorRef point(points + 3 * i, 3);
    point = scale * (point - median);
  }

  double *cameras = camera_add();
  double angle_axis[3];
  double center[3];
  for (int i = 0; i < num_camera_; i++) {
    double *camera = cameras + camera_block_size() * i;
    CameraToAngleAxisAndCenter(camera, angle_axis, center);
    // center = scale * (center -median)
    VectorRef(center, 3) = scale * (VectorRef(center, 3) - median);
    AngleAxisAndCenterToCamera(angle_axis, center, camera);
  }
}

void BAL::Perturb() {}

void BAL::CameraToAngleAxisAndCenter(const double *camera, double *angle_axis, double *center) const {
  VectorRef angle_axis_ref(angle_axis, 3);
  angle_axis_ref = ConstVecRef(camera, 3);

  // c = -R't
  Eigen::VectorXd inverse_rotation = -angle_axis_ref;

  AngleAxisRotatePoint(inverse_rotation.data(), camera + camera_block_size() - 6, center);
  VectorRef(center, 3) *= -1.0;
}

void BAL::AngleAxisAndCenterToCamera(const double *angle_axis, const double *center, double *camera) const {
  ConstVecRef angle_axis_ref(angle_axis, 3);
  VectorRef(camera, 3) = angle_axis_ref;

  // t = -R * c
  AngleAxisRotatePoint(angle_axis, center, camera + camera_block_size() - 6);
  VectorRef(camera + camera_block_size() - 6, 3) *= -1.0;
}

// Write the problem to a PLY file for inspection in Meshlab or CloudCompare
// 格式相当重要, 否则显示会出错
void BAL::WriteToPLY(const string &filename) {
  ofstream of(filename.c_str());

  of << "ply" << '\n'
     << "format ascii 1.0" << '\n'
     << "element vertex " << num_camera_ + num_pts_ << '\n'
     << "property float x" << '\n'
     << "property float y" << '\n'
     << "property float z" << '\n'
     << "property uchar red" << '\n'
     << "property uchar green" << '\n'
     << "property uchar blue" << '\n'
     << "end_header" << std::endl;

  // Export extrinsic data (i.e. camera centers) as green points
  double angle_axis[3];
  double center[3];
  for (int i = 0; i < num_camera_; i++) {
    const double *camera = cameras() + camera_block_size() * i;

    CameraToAngleAxisAndCenter(camera, angle_axis, center);
    of << center[0] << ' ' << center[1] << ' ' << center[2]
       << " 0 255 0" << '\n';

  }

  // Export 3D points as white points
  for (int i = 0; i < num_pts_; i++) {
    const double *point = pts() + point_block_size() * i;
    for (int j = 0; j < point_block_size(); ++j) {
      of << point[j] << ' ';
    }
    of << " 255 255 255\n";

  }
  of.close();
}