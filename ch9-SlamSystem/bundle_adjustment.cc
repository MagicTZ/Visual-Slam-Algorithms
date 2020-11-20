#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <fstream>
#include <iostream>

#include "../include/common.h"

using namespace std;
using namespace Eigen;

struct PoseAndIntrinsics {
    PoseAndIntrinsics () {}

    explicit PoseAndIntrinsics(double *data_addr) {
        rotation = Vector3d(data_addr[0], data_addr[1], data_addr[2]);
        translation = Vector3d(data_addr[3], data_addr[4], data_addr[5]);
        focal = data_addr[6];
        k1 = data_addr[7];
        k2 = data_addr[8];
    }

    Vector3d rotation = Vector3d::Zero();
    Vector3d translation = Vector3d::Zero();
    double focal{0.0};
    double k1{0.0}, k2{0.0};
};

typedef vector<PoseAndIntrinsics, Eigen::aligned_allocator<PoseAndIntrinsics>> VecPoseAndIntrinsics;
typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;

Matrix<double, 3, 3> ExpSO3(const Vector3d &v) {
    Matrix<double, 3, 3> I = MatrixXd::Identity(3, 3);
    const double x = v[0];
    const double y = v[1];
    const double z = v[2];
    const double d2 = x * x + y * y + z * z;
    const double d = sqrt(d2);
    Matrix<double, 3, 3> W;
    W(0,0) = 0;
    W(0,1) = -z;
    W(0,2) = y;
    W(1,0) = z;
    W(1,1) = 0;
    W(1,2) = -x;
    W(2,0) = -y;
    W(2,1) = x;
    W(2,2) = 0;

    if (d < 1e-4) {
        return (I + W + 0.5 * W * W);
    } else {
        return (I + W*sin(d)/d + W*W*(1.0-cos(d))/d2);
    }
}

Vector3d LogSO3(const Matrix<double, 3, 3> &R) {
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    Vector3d w((R(2, 1) - R(1, 2)) / 2,
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

void Bundle_Adjustment(BALProblem &bal_problem) {
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    double *points = bal_problem.mutable_points();
    double *cameras = bal_problem.mutable_cameras();
    int num_cameras = bal_problem.num_cameras();
    int num_points = bal_problem.num_points();

    VecPoseAndIntrinsics pose_intrinsics;
    VecVector3d mappoints;
    for (auto i = 0; i < num_cameras; ++i) {
        double *camera = cameras + camera_block_size * i;
        struct PoseAndIntrinsics pose(camera);
        pose_intrinsics.push_back(pose);
    }

    for (auto i = 0; i < num_points; ++i) {
        double *point = points + point_block_size * i;
        mappoints.push_back(Vector3d(point[0], point[1], point[2]));
    }

    int iterations{100};
    double cost = 0, lastCost = 0;
    auto dim = 9 * num_cameras + 3 * num_points;
    const double *observations = bal_problem.observations();
    for (auto iter = 0; iter < iterations; ++iter) {
        MatrixXd H{dim, dim};
        H.setZero();
        VectorXd b{dim};
        b.setZero();

        cost = 0;
        for (auto i = 0; i < bal_problem.num_observations(); ++i) {
            auto camera_id = bal_problem.camera_index()[i];
            auto point_id = bal_problem.point_index()[i];
            Vector3d pc = ExpSO3(pose_intrinsics[camera_id].rotation) * mappoints[point_id] + pose_intrinsics[camera_id].translation;
            double x = -pc[0];
            double y = -pc[1];
            double z = -pc[2];
            double x2 = x * x;
            double y2 = y * y;
            double z2 = z * z;
            double f = pose_intrinsics[camera_id].focal;
            double k1 = pose_intrinsics[camera_id].k1;
            double k2 = pose_intrinsics[camera_id].k2;
            Vector2d pc_norm(x / z, y / z);
            double r2 = pc_norm.squaredNorm();
            double distortion = 1.0 + r2 * (k1 + k2 * r2);
            Vector2d pc_distorted(pc_norm[0] * distortion, pc_norm[1] * distortion);
            Vector2d proj(f * pc_distorted[0], f * pc_distorted[1]);
            Vector2d obs(observations[2 * i], observations[2 * i + 1]);
            Vector2d e = obs - proj;
            cost += e.squaredNorm() / 2;

            MatrixXd J{2, dim};
            J.setZero();

            Matrix<double, 2, 3> J_error_p;
            J_error_p(0, 0) = -f / z - 3 * f * k1 * x2 / z2 / z - f * k1 * y2 / z2 / z - 5 * f * k2 * x2 * x2 / z2 / z2 / z - f * k2 * y2 * y2 / z2 / z2 / z -6 * f * k2 * x2 * y2 / z2 / z2 / z;
            J_error_p(0, 1) = -2 * f * k1 * x * y / z2 / z - 4 * f * k2 * x * y2 * y / z2 / z2 / z - 4 * f * k2 * x2 * x * y / z2 / z2 / z;
            J_error_p(0, 2) = f * x / z2 + 3 * f * k1 * x2 * x / z2 / z2 + 3 * f * k1 * x * y2 / z2 / z2 + 5 * f * k2 * x2 * x2 * x / z2 / z2 / z2 + 5 * f * k2 * x * y2 * y2 / z2 / z2 / z2 + 10 * f * k2 * x2 * x * y2 / z2 / z2 / z2;
            J_error_p(1, 0) = -2 * f * k1 * x * y / z2 / z - 4 * f * k2 * x2 * x * y / z2 / z2 / z - 4 * f * k2 * x * y2 * y / z2 / z2 / z;
            J_error_p(1, 1) = -f / z - f * k1 * x2 / z2 / z - 3 * f * k1 * y2 / z2 / z - f * k2 * x2 * x2 / z2 / z2 / z - 5 * f * k2 * y2 * y2 / z2 / z2 / z - 6 * f * k2 * x2 * y2 / z2 / z2 / z;
            J_error_p(1, 2) = f * y / z2 + 3 * f * k1 * x2 * y / z2 / z2 + 3 * f * k1 * y2 * y / z2 / z2 + 5 * f * k2 * x2 * x2 * y / z2 / z2 / z2 + 5 * f * k2 * y2 * y2 * y / z2 / z2 / z2 + 10 * f * k2 * x2 * y2 * y / z2 / z2 / z2;

            Matrix<double, 3, 6> J_p_xi;
            J_p_xi << 1, 0, 0, 0, z, -y,
                      0, 1, 0, -z, 0, x,
                      0, 0, 1, y, -x, 0;

            Matrix<double, 2, 3> J_error_intrinsics;
            J_error_intrinsics(0, 0) = -distortion * x / z;
            J_error_intrinsics(0, 1) = -f * r2 * x / z;
            J_error_intrinsics(0, 2) = -f * r2 * r2 * x / z;
            J_error_intrinsics(1, 0) = -distortion * y / z;
            J_error_intrinsics(1, 1) = -f * r2 * y / z;
            J_error_intrinsics(1, 2) = -f * r2 * r2 * y / z;
            
            J.block<2, 6>(0, camera_id * 9) = J_error_p * J_p_xi;
            J.block<2, 3>(0, camera_id * 9 + 6) = J_error_intrinsics;
            J.block<2, 3>(0, 9 * num_cameras + point_id * 3) = J_error_p * ExpSO3(pose_intrinsics[camera_id].rotation);
            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // VectorXd dx{dim};
        // dx = H.ldlt().solve(b);
        MatrixXd B = H.topLeftCorner(9 * num_cameras, 9 * num_cameras);
        MatrixXd E = H.topRightCorner(9 * num_cameras, 3 * num_points);
        MatrixXd C = H.bottomRightCorner(3 * num_points, 3 * num_points);
        VectorXd v = b.head(9 * num_cameras);
        VectorXd w = b.tail(3 * num_points);

        VectorXd dx_c{9 * num_cameras};
        dx_c = (B - E * C.inverse() * E.transpose()).ldlt().solve(v - E * C.inverse() * w);
        VectorXd dx_p = C.inverse() * (w - E.transpose() * dx_c);  

        if (isnan(dx_c[0]) || isnan(dx_p[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        for (auto i = 0; i < num_cameras; ++i) {
            Vector3d update_rotation = dx_c.segment<3>(i * 9);
            pose_intrinsics[i].rotation = LogSO3(ExpSO3(update_rotation) * ExpSO3(pose_intrinsics[i].rotation));
            pose_intrinsics[i].translation += dx_c.segment<3>(i * 9 + 3);
            pose_intrinsics[i].focal += dx_c[i * 9 + 6];
            pose_intrinsics[i].k1 += dx_c[i * 9 + 7];
            pose_intrinsics[i].k2 += dx_c[i * 9 + 8];
        }

        for (auto i = 0; i < num_points; ++i) {
            mappoints[i] += dx_p.segment<3>(i * 3);
        }        
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    for (auto i = 0; i < num_cameras; ++i) {
        double *camera = cameras + camera_block_size * i;
        for (auto j = 0; j < 3; ++j) {
            camera[j] = pose_intrinsics[i].rotation[j];
        }
        for (auto j = 0; j < 3; j++) {
            camera[j + 3] = pose_intrinsics[i].translation[j];
        }
        camera[6] = pose_intrinsics[i].focal;
        camera[7] = pose_intrinsics[i].k1;
        camera[8] = pose_intrinsics[i].k2;
    }

    for (auto i = 0; i < num_points; ++i) {
        double *point = points + point_block_size * i;
        for (auto j = 0; j < 3; ++j) {
            point[j] = mappoints[i][j];
        }   
    }    
}

int main(int argc, char **argv) {
    if (argc != 2) {
        cerr << "Usage: bundle_adjustment bal_data.txt" << endl;
    }

    BALProblem bal_problem(argv[1]);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1, 0.5, 0.5);
    bal_problem.WriteToPLYFile("before_bundle_adjustment.ply");
    Bundle_Adjustment(bal_problem);
    bal_problem.WriteToPLYFile("after_bundle_adjustment.ply");
}