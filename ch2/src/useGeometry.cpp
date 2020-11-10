#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main(int argc, char **argv)
{
    Matrix3d rotation_matrix = Matrix3d::Identity();

    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1)); // 沿z轴旋转45°
    cout.precision(3);                                       // 精度到小数点后第三位
    cout << "rotation matrix = \n"
         << rotation_vector.matrix() << endl; // 用matrix()转换成矩阵也可以直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();

    // 用AngleAxis可以进行坐标变换
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by matrix) =" << v_rotated.transpose() << endl;

    // 欧拉角：可以将旋转矩阵直接转换成欧拉角
    

    return 0;
}