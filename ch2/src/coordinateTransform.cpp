#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    // 世界坐标系下的物体位姿
    Quaterniond q1(0.55, 0.3, 0.2, 0.2), q2(-0.1, 0.3, -0.7, 0.2);
    Vector3d t1(0.7, 1.1, 0.2), t2(-0.1, 0.4, 0.8);

    // Quaterniond Normalization
    q1.normalize();
    q2.normalize();

    // 小萝卜1号坐标系下的坐标
    Vector3d p1(0.5, -0.1, 0.2);

    // Quaterniond -> Transform Matrix
    Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    // Coordinate Transformation
    Vector3d p2 = T2w * T1w.inverse() * p1;
    //cout.precision(3);
    cout << "该向量在小萝卜二号坐标系下的坐标为： " << p2.transpose() << endl;

    return 0;
}