//
// Created by magictz on 2020/11/7.
//

#include <iostream>

//#include <Eigen/Geometry>
#include <g2o/core/base_binary_edge.h>  // 引入基础二元边头文件, 基于该类派生定义自己需要的边
#include <g2o/core/base_vertex.h>  // 引入基础顶点头文件, 然后可以根据该类派生定义自己的顶点
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>  // 引入LM优化算法头文件
#include <g2o/core/robust_kernel_impl.h>                // 引入鲁棒核函数
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>  // 引入线性稠密求解方法

#include <sophus/se3.hpp>

#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "BAL.h"

using namespace std;
using namespace Sophus;

const string bal_file = "./../problem-16-22106-pre.txt";  // BAL file name

//========================Vertex==============================

// 相机位姿和相机内参的结构体
// 包含了9个参数(rotation[3*3], translation[3*1], f, k1 ,k2)
struct CameraParameter {
  CameraParameter() {}

  Sophus::SO3d rotation;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double f = 0;
  double k1 = 0, k2 = 0;

  // Initialization
  explicit CameraParameter(double *data) {
    rotation = Sophus::SO3d::exp(Eigen::Vector3d(data[0], data[1], data[2]));
    translation = Eigen::Vector3d(data[3], data[4], data[5]);
    f = data[6];
    k1 = data[7];
    k2 = data[8];
  }

  void loadEstimate(double *data) {
    auto se3 = rotation.log();
    for (int i = 0; i < 3; i++) data[i] = se3[i];
    for (int i = 0; i < 3; i++) data[i + 3] = translation[i];
    data[6] = f;
    data[7] = k1;
    data[8] = k2;
  }
};

// 基于BaseVertex类自己定义了相机顶点, 并且重新改写了setToOriginImpl和oplusImpl函数
class CameraVertex : public g2o::BaseVertex<9, CameraParameter> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  CameraVertex() {}

  virtual void setToOriginImpl() override  // reset
  {
    _estimate = CameraParameter();
  }

  // update
  virtual void oplusImpl(const double *update) override {
    _estimate.rotation = SO3d::exp(Vector3d(update[0], update[1], update[2])) * _estimate.rotation;  //左扰动模型
    _estimate.translation += Vector3d(update[3], update[4], update[5]);
    _estimate.f += update[6];
    _estimate.k1 += update[7];
    _estimate.k2 += update[8];
  }

  // 读盘
  virtual bool read(istream &in) {}
  // 存盘
  virtual bool write(ostream &out) const {}
};

// 三维观测点的派生类
class PointVertex : public g2o::BaseVertex<3, Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override { _estimate = Vector3d(0, 0, 0); }

  virtual void oplusImpl(const double *update) override { _estimate += Vector3d(update[0], update[1], update[2]); }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

//===========================Edge==============================
// 基于BaseBinaryEdge类定义Edge, 观测是x,y, 类型是Vector2d,
// 连接Edge的两边顶点已经定义出来,分别是CameraVertex和PointVertex
class ObsEdge : public g2o::BaseBinaryEdge<2, Vector2d, CameraVertex, PointVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // 计算projection
  // K: 相机内外参, 9*1
  // pt: 三维点坐标, 3*1
  Vector2d project(const CameraParameter &K, const Vector3d &pt) {
    Vector3d pt_camera = K.rotation * pt + K.translation;  // 世界坐标系转到相机坐标系
    // 相机坐标系转到图像坐标系
    Vector2d pt_image = Vector2d(K.f * pt_camera[0] / pt_camera[2], K.f * pt_camera[1] / pt_camera[2]);
    // 加入畸变模型
    double r2 = K.k1 * K.k1 + K.k2 * K.k2;
    double x = 0, y = 0;
    x = pt_image[0] * (1 + K.k1 * r2 + K.k2 * r2 * r2);
    y = pt_image[1] * (1 + K.k1 * r2 + K.k2 * r2 * r2);

    return Vector2d(-x, -y);  // BAL 投影模型多了一个负号
  }

  // 计算误差
  virtual void computeError() override {
    auto v0 = (CameraVertex *)_vertices[0];  // 相机顶点
    auto v1 = (PointVertex *)_vertices[1];   // 观测点
    auto proj = project(v0->estimate(), v1->estimate());
    _error = proj - _measurement;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

void BA(BAL &bal);

int main(int argc, char **argv) {
  // check if input rightly
  string bal_input;
  // bal_input = *argv[1];
  // 命令行参数(首选)或者函数内部给定文件路径
  if (argc != 2) {
    if (!bal_file.empty()) {
      bal_input = bal_file;
    } else {
      cout << "Input format wrong! The format: binaryfile  bal_data.txt" << endl;
      return 1;
    }
  }

  // Read BAL data file
  BAL bal_problem(bal_input);

  // Initial Status
  bal_problem.WriteToPLY("initial.ply");

  // Normalize
  bal_problem.Normalize();

  // Add noise by Perturb
  bal_problem.Perturb();

  // BA algortithm
  BA(bal_problem);

  // Write to PLY file
  bal_problem.WriteToPLY("final.ply");

  return 0;
}

void BA(BAL &bal) {
  const int point_block_size = bal.point_block_size();
  const int camera_block_size = bal.camera_block_size();

  double *points = bal.pts_add();      // 3*num_pts
  double *cameras = bal.camera_add();  // 9*num_camera

  // pose dimension 9, landmark is 3
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BlockSolverType;
  typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

  // use LM
  // 选择使用LM算法进行优化
  auto solver =
      new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  //====================build g2o problem=================================
  const double *obs_x = bal.obs_x();
  const double *obs_y = bal.obs_y();

  // 相机Vertex和三维点
  vector<CameraVertex *> camera_vertex;
  vector<PointVertex *> point_vertex;
  cout << "The number of cameras " << bal.num_cameras() << endl;
  cout << "The number of 3d points " << bal.num_points() << endl;
  // 添加相机(Vertex)
  for (int i = 0; i < bal.num_cameras(); i++) {
    CameraVertex *v = new CameraVertex();
    double *camera = cameras + camera_block_size * i;
    // cout<<"camera paras: " <<cameras[0]<<' '<<cameras[1]<<' '<<cameras[2]<<' '<<camera_block_size<<endl;
    cout << "camera paras: " << camera[0] << ' ' << camera[1] << ' ' << camera[2] << endl;
    v->setId(i);
    v->setEstimate(CameraParameter(camera));  // 第i组相机参数进行估计
    optimizer.addVertex(v);
    camera_vertex.push_back(v);
  }
  // 添加路标(Vertex)
  for (int i = 0; i < bal.num_points(); i++) {
    PointVertex *v = new PointVertex();
    double *pt = points + point_block_size * i;
    v->setId(i + bal.num_cameras());  // 每个点的id是唯一的, 不管是相机顶点还是路标顶点
    v->setEstimate(Vector3d(pt[0], pt[1], pt[2]));
    v->setMarginalized(true);  // g2o在BA中需要手动设置Marg的顶点
    optimizer.addVertex(v);
    point_vertex.push_back(v);
  }
  // 添加边(Edge)
  for (int i = 0; i < bal.num_obs(); i++) {
    ObsEdge *edge = new ObsEdge();
    edge->setVertex(0, camera_vertex[bal.camera_idx()[i]]);
    edge->setVertex(1, point_vertex[bal.point_idx()[i]]);
    edge->setMeasurement(Vector2d(obs_x[i], obs_y[i]));
    edge->setInformation(Matrix2d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(edge);
  }
  //=======================================================================

  // 调用g2o
  optimizer.initializeOptimization();
  optimizer.optimize(50);

  // set to bal problem
  for (int i = 0; i < bal.num_cameras(); i++) {
    double *camera = cameras + camera_block_size * i;
    auto vertex = camera_vertex[i];
    auto estimate = vertex->estimate();
    estimate.loadEstimate(camera);
  }
  for (int i = 0; i < bal.num_points(); i++) {
    double *point = points + point_block_size * i;
    auto vertex = point_vertex[i];
    for (int j = 0; j < 3; j++) point[j] = vertex->estimate()[j];
  }
}