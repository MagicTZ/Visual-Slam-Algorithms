//
// Created by magictz on 2020/11/7.
//

#include "BAL.h"
#include "common.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <float.h>


using namespace std;

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVecRef;  //作用是什么?
typedef Eigen::Matrix<double, 9, 1> Vector9d;

// 相机位姿和相机内参的结构体
// 包含了9个参数(rotation[3*1], translation[3*1], f, k1 ,k2)
struct CameraParameter {
    CameraParameter() {}

    // Define
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    double f = 0.0;
    double k1 = 0.0, k2 = 0.0;

    // Initialization
    explicit CameraParameter(double *data) {
        rotation = VecToMatRotation(Eigen::Vector3d(data[0], data[1], data[2])); // vector-> matrix
        translation = Eigen::Vector3d(data[3], data[4], data[5]);
        f = data[6];
        k1 = data[7];
        k2 = data[8];
    }

    // update
    void update(Vector9d data) {
        auto delta_rotate = VecToMatRotation(Eigen::Vector3d(data[0], data[1], data[2]));
        rotation = delta_rotate * rotation;
        translation += Eigen::Vector3d(data[3], data[4], data[5]);
        f += data[6];
        k1 += data[7];
        k1 += data[8];
    }
};

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

typedef vector<CameraParameter, Eigen::aligned_allocator<CameraParameter>> VecCameraParas;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecPoints;

void BAL::Optimization(const int iteration) {
    // 创建相机参数和三维点的自定义容器对象
    VecCameraParas camParas;
    VecPoints Pts;
    for (size_t i = 0; i < num_camera_; i++) {
        double *cam = camera_para_ + camera_block_size() * i;
        CameraParameter camPar(cam);    // 使用自定义结构初始化
        camParas.push_back(camPar);     // 将相机参数放入camParas的容器中
    }
    for (size_t i = 0; i < num_pts_; i++) {
        double *point = pts_ + point_block_size() * i;
        Eigen::Vector3d pts(point[0], point[1], point[2]);
        Pts.push_back(pts);
    }

    // 进行迭代
    double lastcost = DBL_MAX; //包含在<float.h>
    for (int iter = 0; iter < iteration; iter++) {
        // H,b
        int H_dim = 9 * num_camera_ + 3 * num_pts_;
        Eigen::MatrixXd H = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>();
        H.resize(H_dim, H_dim); // TODO::数组超界问题
        H.setZero();
        Eigen::VectorXd b{H_dim};
        b.setZero();

        double cost = 0.0;
        // 遍历所有观测(edge)
        for (size_t idx = 0; idx < num_obs_; idx++) {
            auto cam_idx = camera_idx()[idx];
            auto pt_idx = point_idx()[idx];

            auto Points = Pts[pt_idx]; // 三维点
            auto rotation = camParas[cam_idx].rotation;
            auto translation = camParas[cam_idx].translation;
            double k1 = camParas[cam_idx].k1, k2 = camParas[cam_idx].k2;
            double f = camParas[cam_idx].f;
            // 图像坐标
            auto pc = rotation * Points + translation; // pc = R*P+T
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            // 归一化坐标(z=1)
            double x = -pc[0] / pc[2]; //负号是因为BAL数据集特有的
            double y = -pc[1] / pc[2];

            // 加上畸变改正
            double r2 = x * x + y * y; // r^2 = x^2+y^2
            double x_ = (1 + k1 * r2 + k2 * r2 * r2) * x;
            double y_ = (1 + k1 * r2 + k2 * r2 * r2) * y;
            // 像素坐标
            x_ = f * x_;
            y_ = f * y_;

            Eigen::Vector2d proj(x_, y_);    // 重投影坐标(像素坐标系)
            Eigen::Vector2d obs(obs_x_[idx], obs_y_[idx]);  // 观测值
            Eigen::Vector2d error = obs - proj; // 计算重投影误差 error = obs - proj

            cost += error.squaredNorm() / 2.0; // 计算所有观测累计cost

            // ==========================================Jocabian=======================================================
            Eigen::MatrixXd J{2, 12};
            // 相机部分的雅格比矩阵
            Eigen::Matrix<double, 2, 9> J_cam = Eigen::Matrix<double, 2, 9>::Zero();
            J_cam(0, 0) = f * inv_z;
            J_cam(0, 2) = -f * pc[0] * inv_z2;
            J_cam(0, 3) = -f * pc[0] * pc[1] * inv_z2;
            J_cam(0, 4) = f + f * pc[0] * pc[0] * inv_z2;
            J_cam(0, 5) = -f * pc[1] * inv_z;
            J_cam(0, 6) = -pc[0] * inv_z * (1 + k1 * r2 + k2 * r2 * r2);
            J_cam(0, 7) = -f * pc[0] * r2 * inv_z;
            J_cam(0, 8) = -f * pc[0] * r2 * r2 * inv_z;
            J_cam(1, 1) = f * inv_z;
            J_cam(1, 2) = -f * pc[1] * inv_z2;
            J_cam(1, 3) = -f - f * pc[1] * pc[1] * inv_z2;
            J_cam(1, 4) = f * pc[0] * pc[1] * inv_z2;
            J_cam(1, 5) = f * pc[0] * inv_z;
            J_cam(1, 6) = -pc[1] * inv_z * (1 + k1 * r2 + k2 * r2 * r2);
            J_cam(1, 7) = -f * pc[1] * r2 * inv_z;
            J_cam(1, 8) = -f * pc[1] * r2 * r2 * inv_z;

            // 三维点的雅格比矩阵 (超级复杂)
            Eigen::Matrix<double, 2, 3> J_pt = Eigen::Matrix<double, 2, 3>::Zero();
            J_pt(0, 0) =
                    -f * inv_z - 3 * f * k1 * pc[0] * pc[0] * inv_z * inv_z2 - f * k1 * pc[1] * pc[1] * inv_z * inv_z2 -
                    5 * f * k2 * pc[0] * pc[0] * pc[0] * pc[0] * inv_z2 * inv_z2 * inv_z -
                    f * k2 * pc[1] * pc[1] * pc[1] * pc[1] * inv_z2 * inv_z2 * inv_z -
                    6 * f * k2 * pc[0] * pc[0] * pc[1] * pc[1] * inv_z2 * inv_z2 * inv_z;
            J_pt(0, 1) = -2 * f * k1 * x * y * inv_z2 * inv_z -
                         4 * f * k2 * x * pc[1] * pc[1] * y * inv_z2 * inv_z2 * inv_z -
                         4 * f * k2 * pc[0] * pc[0] * x * y * inv_z2 * inv_z2 * inv_z;
            J_pt(0, 2) = f * x * inv_z2 + 3 * f * k1 * pc[0] * pc[0] * x * inv_z2 * inv_z2 +
                         3 * f * k1 * x * pc[1] * pc[1] * inv_z2 * inv_z2 +
                         5 * f * k2 * pc[0] * pc[0] * pc[0] * pc[0] * x * inv_z2 * inv_z2 * inv_z2 +
                         5 * f * k2 * x * pc[1] * pc[1] * pc[1] * pc[1] * inv_z2 * inv_z2 * inv_z2 +
                         10 * f * k2 * pc[0] * pc[0] * x * pc[1] * pc[1] * inv_z2 * inv_z2 * inv_z2;
            J_pt(1, 0) = -2 * f * k1 * x * y * inv_z2 * inv_z -
                         4 * f * k2 * pc[0] * pc[0] * x * y * inv_z2 * inv_z2 * inv_z -
                         4 * f * k2 * x * pc[1] * pc[1] * y * inv_z2 * inv_z2 * inv_z;
            J_pt(1, 1) =
                    -f * inv_z - f * k1 * pc[0] * pc[0] * inv_z2 * inv_z - 3 * f * k1 * pc[1] * pc[1] * inv_z2 * inv_z -
                    f * k2 * pc[0] * pc[0] * pc[0] * pc[0] * inv_z2 * inv_z2 * inv_z -
                    5 * f * k2 * pc[1] * pc[1] * pc[1] * pc[1] * inv_z2 * inv_z2 * inv_z -
                    6 * f * k2 * pc[0] * pc[0] * pc[1] * pc[1] * inv_z2 * inv_z2 * inv_z;
            J_pt(1, 2) = f * y * inv_z2 + 3 * f * k1 * pc[0] * pc[0] * y * inv_z2 * inv_z2 +
                         3 * f * k1 * pc[1] * pc[1] * y * inv_z2 * inv_z2 +
                         5 * f * k2 * pc[0] * pc[0] * pc[0] * pc[0] * y * inv_z2 * inv_z2 * inv_z2 +
                         5 * f * k2 * pc[1] * pc[1] * pc[1] * pc[1] * y * inv_z2 * inv_z2 * inv_z2 +
                         10 * f * k2 * pc[0] * pc[0] * pc[1] * pc[1] * y * inv_z2 * inv_z2 * inv_z2;

            // 将两部分雅格比矩阵按照稀疏性加入到整体的Jacobian当中
            // 添加方式按照第cam_idx个J_cam添加到J.block(0, cam_idx*9)
            // 第pt_idx个J_pt添加到J.block(0, (nums_cam-1) * 9 + pt_idx*3)
            J.block<2, 9>(0, cam_idx * 9) = J_cam;
            J.block<2, 3>(0, (num_camera_ - 1) * 9 + pt_idx * 3) = J_pt;
            //==========================================================================================================

            // 计算H,b
            H += J.transpose() * J;
            b += -J.transpose() * error;

        }

        //====================================使用Marginalization (Schur消元)============================================
        int B_dim = 9 * num_camera_;
        int C_dim = 3 * num_pts_;
        Eigen::MatrixXd B = H.block(0, 0, B_dim, B_dim);
        Eigen::MatrixXd C = H.block(B_dim, B_dim, C_dim, C_dim);
        Eigen::MatrixXd E = H.block(0, B_dim, B_dim, C_dim);
        Eigen::VectorXd v = b.head(B_dim), w = b.tail(C_dim);

        // 先求出x_c的值, 大小: B_dim*1
        Eigen::MatrixXd M = B - E * C.inverse() * E.transpose(); // B-E*C^-1*ET
        auto x_c = M.ldlt().solve(v - E * C.inverse() * w);
        // 求解路标的增量方程, 解得x_p, 大小: C_dim*1
        auto x_p = C.ldlt().solve(w - E.transpose() * x_c);

        if (isnan(x_c[0] || isnan(x_p[0]))) {
            cout << "result is nan" << endl;
            break;
        }

        if (iter > 0 && cost > lastcost) {
            cout << "converge not right" << endl;
            cout << "cost: " << cost << "last cost: " << lastcost << endl;
            break;
        }
        lastcost = cost;
        cout << "iteration = " << iter << " cost = " << cout.precision(12) << lastcost << endl;

        //====================================更新增量========================================
        // 相机参数增量
        for (size_t i = 0; i < num_camera_; i++) {
            Vector9d update = x_c.segment<9>(i*9);
            camParas[i].update(x_c);
        }
        // 三维点增量
        for(size_t i = 0;i<num_pts_;i++){
            Pts[i] += x_p.segment<3>(i*3);
        }
    }//迭代完成

    // 更新数据
    for (auto i = 0; i < num_camera_; ++i) {
        double *camera = camera_para_ + camera_block_size() * i;
        for (auto j = 0; j < 3; ++j) {
            auto rot = MatToVecRotation(camParas[i].rotation);
            camera[j] = rot[j];
        }
        for (auto j = 0; j < 3; j++) {
            camera[j + 3] = camParas[i].translation[j];
        }
        camera[6] = camParas[i].f;
        camera[7] = camParas[i].k1;
        camera[8] = camParas[i].k2;
    }

    for (auto i = 0; i < num_pts_; ++i) {
        double *point = pts_ + point_block_size() * i;
        for (auto j = 0; j < 3; ++j) {
            point[j] = Pts[i][j];
        }
    }

}