//
// Created by magictz on 2020/11/7.
//


#ifndef G2O_BAL_H
#define G2O_BAL_H

#include <string>
#include <vector>
#include "rotation.h"
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

class BAL {
 public:
  explicit BAL(const string &filename);  // initialize

  ~BAL() {
    delete[] camera_index_;  // 每个obs对应的camera index
    delete[] point_index_;   // 每个obs对应的point index
    delete[] obs_x_;
    delete[] obs_y_;
    delete[] camera_para_;  // 相机位姿(可以通过索引camera_para_[camera_index_[i]]得到)
    delete[] pts_;          //  点的坐标(索引pts_[point_index_[i]])
  }

  void Normalize();

  void Perturb();

  void WriteToPLY(const string &filename);

  void Optimization(const int iteration = 50);

  void CameraToAngleAxisAndCenter(const double *camera, double *angle_axis, double *center) const;

  void AngleAxisAndCenterToCamera(const double *angle_axis, const double *center, double *camera) const;

  int point_block_size() const { return 3; }

  int camera_block_size() const { return 9; }

  int num_cameras() const { return num_camera_; }

  int num_points() const { return num_pts_; }

  int num_obs() const { return num_obs_; }

  const int *camera_idx() const { return camera_index_; }

  const int *point_idx() const {return  point_index_;}

  const double *obs_x() const { return obs_x_; }

  const double *obs_y() const { return obs_y_; }

  const double *cameras() const { return camera_para_; }

  const double *pts() const { return pts_; }

  double *camera_add() const { return camera_para_; }

  double *pts_add() const { return pts_; }

 private:
  int num_camera_;     // 相机位姿的数量
  int num_pts_;        // 点的数量
  int num_obs_;        // 观测的数量
  int *camera_index_;  // 每个obs对应的camera index
  int *point_index_;   // 每个obs对应的point index
  double *obs_x_;
  double *obs_y_;
  double *camera_para_;  // 相机位姿(可以通过索引camera_para_[camera_index_[i]]得到)
  double *pts_;          //  点的坐标(索引pts_[point_index_[i]])
};

#endif  // G2O_BAL_H
