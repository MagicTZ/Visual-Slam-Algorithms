//
// Edited by Zhen Tan on 11/09/2020
//

#include <iostream>

using namespace std;

#include <g2o/core/base_binary_edge.h>
//#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>

using namespace Eigen;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "./../poses.txt";
string points_file = "./../points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
  uchar *data = &img.data[int(y) * img.step + int(x)];
  float xx = x - floor(x);
  float yy = y - floor(y);
  return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] + (1 - xx) * yy * data[img.step] +
               xx * yy * data[img.step + 1]);
}

inline bool bInImage(float u, float v, int w, int h) {
  if (u >= 0 && u < w && v >= 0 && v < h)
    return true;
  else
    return false;
}

// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSophus() {}

  ~VertexSophus() {}

  bool read(std::istream &is) {}

  bool write(std::ostream &os) const {}

  virtual void setToOriginImpl() { _estimate = Sophus::SE3d(); }

  virtual void oplusImpl(const double *update_) {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
    setEstimate(Sophus::SE3d::exp(update) * estimate());
  }
};

// 三维观测点的派生类
class PointVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override { _estimate = Vector3d(0, 0, 0); }

  virtual void oplusImpl(const double *update) override { _estimate += Vector3d(update[0], update[1], update[2]); }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double, 16, 1> Vector16d;
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, PointVertex, VertexSophus> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeDirectProjection(float *color, cv::Mat &target) {
    this->origColor = color;
    this->targetImg = target;

    this->w = targetImg.cols;
    this->h = targetImg.rows;
  }

  ~EdgeDirectProjection() {}

  virtual void computeError() override {
    // TODO START YOUR CODE HERE
    // compute projection error ...
    // const g2o::VertexSBAPointXYZ *vertexPw = static_cast<const g2o::VertexSBAPointXYZ *>(vertex(0));
    auto *vertexPw = static_cast<const PointVertex *>(_vertices[0]);
    //李群相机位姿
    const VertexSophus *vertexTcw = static_cast<const VertexSophus *>(_vertices[1]);
    Eigen::Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();
    float u = Pc[0] / Pc[2] * fx + cx;
    float v = Pc[1] / Pc[2] * fy + cy;

    // 判断是否出边框
    if (!bInImage(u - 3, v - 3, w, h) || !bInImage(u + 2, v + 2, w, h)) {
      this->setLevel(1);
      for (int n = 0; n < 16; n++) _error[n] = 0;
    } else {
      for (int i = -2; i < 2; i++) {
        for (int j = -2; j < 2; j++) {
          int num = 4 * i + j + 10;
          _error[num] = origColor[num] - GetPixelValue(targetImg, u + i, v + j);
        }
      }
    }
    // END YOUR CODE HERE
  }

  // Let g2o compute jacobian for you

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

 private:
  cv::Mat targetImg;           // the target image
  float *origColor = nullptr;  // 16 floats, the color of this point
  int w;
  int h;
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points);

int main(int argc, char **argv) {
  // read poses and points
  VecSE3 poses;
  VecVec3d points;
  ifstream fin(pose_file);

  while (!fin.eof()) {
    double timestamp = 0;
    fin >> timestamp;
    if (timestamp == 0) break;
    double data[7];
    for (auto &d : data) fin >> d;
    poses.push_back(Sophus::SE3d(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                                 Eigen::Vector3d(data[0], data[1], data[2])));
    if (!fin.good()) break;
  }
  fin.close();

  vector<float *> color;
  fin.open(points_file);
  while (!fin.eof()) {
    double xyz[3] = {0};
    for (int i = 0; i < 3; i++) fin >> xyz[i];
    if (xyz[0] == 0) break;
    points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
    cout << Eigen::Vector3d(xyz[0], xyz[1], xyz[2]).transpose() << endl;
    float *c = new float[16];
    for (int i = 0; i < 16; i++) fin >> c[i];
    cout << c[0] << ' ' << c[1] << ' ' << c[2] << endl;
    color.push_back(c);

    if (fin.good() == false) break;
  }
  fin.close();

  cout << "poses: " << poses.size() << ", points: " << points.size() << endl;

  // read images
  vector<cv::Mat> images;
  boost::format fmt("./../%d.png");
  for (int i = 0; i < 7; i++) {
    images.push_back(cv::imread((fmt % i).str(), 0));
  }

  // build optimization problem
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> DirectBlock;  // 求解的向量是6＊1的
  std::unique_ptr<DirectBlock::LinearSolverType> linearSolver(
      new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>());
  std::unique_ptr<DirectBlock> solver_ptr(new DirectBlock(std::move(linearSolver)));  // 矩阵快求解器
  auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // TODO add vertices, edges into the graph optimizer
  // START YOUR CODE HERE
  // 添加路标三维点
  for (int i = 0; i < points.size(); i++) {
    // g2o::VertexSBAPointXYZ *vertexPw = new g2o::VertexSBAPointXYZ();
    PointVertex *vertexPw = new PointVertex();
    vertexPw->setEstimate(points[i]);
    vertexPw->setId(i);
    cout << points[i].transpose() << endl;
    vertexPw->setMarginalized(true);
    optimizer.addVertex(vertexPw);
  }
  // 添加相机位姿Vertex
  for (int j = 0; j < poses.size(); j++) {
    VertexSophus *vertexTcw = new VertexSophus();
    vertexTcw->setEstimate(poses[j]);
    vertexTcw->setId(j + points.size());
    cout << poses[j].matrix() << endl;  //检验
    optimizer.addVertex(vertexTcw);
  }

  // 添加残差边Edge
  // 先添加
  for (int c = 0; c < poses.size(); c++) {
    for (int p = 0; p < points.size(); p++) {
      EdgeDirectProjection *edge = new EdgeDirectProjection(color[p], images[c]);
      edge->setVertex(0, dynamic_cast<PointVertex *>(optimizer.vertex(p)));
      edge->setVertex(1, dynamic_cast<VertexSophus *>(optimizer.vertex(c + points.size())));
      edge->setInformation(Eigen::Matrix<double, 16, 16>::Identity());
      g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;  // 定义鲁棒核函数Huber
      rk->setDelta(0.5);
      edge->setRobustKernel(rk);
      optimizer.addEdge(edge);
    }
  }
  // END YOUR CODE HERE

  // perform optimization
  optimizer.initializeOptimization();
  optimizer.optimize(200);

  // TODO fetch data from the optimizer
  // START YOUR CODE HERE
  for (int c = 0; c < poses.size(); c++) {
    for (int p = 0; p < points.size(); p++) {
      Eigen::Vector3d Pw = dynamic_cast<PointVertex *>(optimizer.vertex(p))->estimate();
      points[p] = Pw;
      Sophus::SE3d Tcw = dynamic_cast<VertexSophus *>(optimizer.vertex(c + points.size()))->estimate();
      poses[c] = Tcw;
    }
  }
  // END YOUR CODE HERE

  // plot the optimized points and poses
  Draw(poses, points);

  // delete color data
  for (auto &c : color) delete[] c;
  return 0;
}

void Draw(const VecSE3 &poses, const VecVec3d &points) {
  if (poses.empty() || points.empty()) {
    cerr << "parameter is empty!" << endl;
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
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // draw poses
    float sz = 0.1;
    int width = 640, height = 480;
    for (auto &Tcw : poses) {
      glPushMatrix();
      Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
      glMultMatrixf((GLfloat *)m.data());
      glColor3f(1, 0, 0);
      glLineWidth(2);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
      glEnd();
      glPopMatrix();
    }

    // points
    glPointSize(2);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points.size(); i++) {
      glColor3f(0.0, points[i][2] / 4, 1.0 - points[i][2] / 4);
      glVertex3d(points[i][0], points[i][1], points[i][2]);
    }
    glEnd();

    pangolin::FinishFrame();
    usleep(5000);  // sleep 5 ms
  }
}
