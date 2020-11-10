//
// Created by Zhen Tan on 25/10/2020
//

#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include <pangolin/pangolin.h>

using namespace std;

const string trajectory = "./../compare.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;  //定义一个轨迹类型

// 轨迹读取函数, 将读取的轨迹存储在est和gt的容器中,平移分量存在est_t和gt_t中
void ReadTrajectory(const string &path, TrajectoryType &est, TrajectoryType &gt, vector<Eigen::Vector3d> &est_t,
                    vector<Eigen::Vector3d> &gt_t);

// 在同一个视图中显示两根轨迹
void DrawTwoTrajectory(const TrajectoryType &gt, const TrajectoryType &est);

// 计算点云的质心
void ComputeCentroid(const vector<Eigen::Vector3d> &pt, Eigen::Vector3d &centroid);

int main(int argc, char **argv) {
  TrajectoryType pt_est, pt_gt;             // 原始轨迹(存储了平移量和四元数)
  vector<Eigen::Vector3d> pt_est_, pt_gt_;  // 平移分量
  // 读取文件
  // 将数据存入到容器中
  ReadTrajectory(trajectory, pt_est, pt_gt, pt_est_, pt_gt_);

  //   画出未对齐的轨迹
  // DrawTwoTrajectory(pt_gt, pt_est);

  // 遍历所有数据点，计算两条轨迹的质心
  Eigen::Vector3d est_centroid, gt_centroid;
  ComputeCentroid(pt_est_, est_centroid);
  ComputeCentroid(pt_gt_, gt_centroid);

  cout << "估计轨迹质心: " << est_centroid.transpose() << endl;
  cout << "真实轨迹质心: " << gt_centroid.transpose() << endl;
  // SVD 方法
  // 依次遍历每个点，计算损失函数，进行优化
  // double cost = 0.0;
  Eigen::Vector3d est_, gt_;
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();

  for (size_t i = 0; i < pt_est_.size(); i++) {
    // 计算去质心坐标
    est_ = pt_est_[i] - est_centroid;
    gt_ = pt_gt_[i] - gt_centroid;

    // 计算 W 矩阵，即 W = sum(pt_gt[i]*pt_est[i].T()), 是3*3矩阵
    W += gt_ * est_.transpose();
  }
  cout << "轨迹点数: " << pt_est_.size() << endl;
  cout << "W = " << W << endl;
  // 对W矩阵SVD分解，得到U，V
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  cout << "U = " << U << endl;
  cout << "V = " << V << endl;

  // R = U*V.T()
  Eigen::Matrix3d R = U * V.transpose();

  // 判断det(R)是否为负,为负,则R = -R
  if (R.determinant() < 0) R = -R;
  cout << "R = " << R << endl;

  // 计算平移向量 t = p - R*p'
  Eigen::Vector3d t = gt_centroid - R * est_centroid;
  cout << "t = " << t.transpose() << endl;
  cout << "gt_centroid = " << gt_centroid << endl;
  cout << "est_entroid = " << est_centroid << endl;
  // 改正轨迹
  Sophus::SE3d T(R, t);
  TrajectoryType pt_est_after;
  for (auto &p : pt_est) {
    p = T * p;
    pt_est_after.push_back(p);
  }
  // 画出纠正后的轨迹图
  DrawTwoTrajectory(pt_gt, pt_est_after);

  return 0;
}

void ReadTrajectory(const string &path, TrajectoryType &est, TrajectoryType &gt, vector<Eigen::Vector3d> &est_t,
                    vector<Eigen::Vector3d> &gt_t) {
  // read trajectory file
  ifstream fin(path);

  if (!fin) {
    cerr << "trajectory" << path << " not found." << endl;
  }

  string line;
  double p1[8];
  double p2[8];
  while (!fin.eof()) {
    fin >> p1[0] >> p1[1] >> p1[2] >> p1[3] >> p1[4] >> p1[5] >> p1[6] >> p1[7] >> p2[0] >> p2[1] >> p2[2] >> p2[3] >>
        p2[4] >> p2[5] >> p2[6] >> p2[7];
    // stringstream s(line);
    // while (s >> p1[0] >> p1[1] >> p1[2] >> p1[3] >> p1[4] >> p1[5] >> p1[6] >> p1[7]) {
    // }
    Sophus::SE3d est_(Eigen::Quaterniond(p1[4], p1[5], p1[6], p1[7]), Eigen::Vector3d(p1[1], p1[2], p1[3]));
    Sophus::SE3d gt_(Eigen::Quaterniond(p2[4], p2[5], p2[6], p2[7]), Eigen::Vector3d(p2[1], p2[2], p2[3]));

    est.push_back(est_);
    gt.push_back(gt_);
    est_t.push_back(Eigen::Vector3d(p1[1], p1[2], p1[3]));
    gt_t.push_back(Eigen::Vector3d(p2[1], p2[2], p2[3]));
  }
}

void ComputeCentroid(const vector<Eigen::Vector3d> &pt, Eigen::Vector3d &centroid) {
  for (size_t i = 0; i < pt.size(); i++) {
    centroid[0] += pt[i][0];
    centroid[1] += pt[i][1];
    centroid[2] += pt[i][2];
  }
  centroid = centroid / pt.size();
}

void DrawTwoTrajectory(const TrajectoryType &gt, const TrajectoryType &est) {
  if (gt.empty() || est.empty()) {
    cerr << "Trajectory is empty!" << endl;
    return;
  }

  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(2);
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0f, 1.0f, 0.0f);  // green for ground truth
      glBegin(GL_LINES);
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    for (size_t i = 0; i < est.size() - 1; i++) {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      auto p1 = est[i], p2 = est[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);  // sleep 5 ms
  }
}