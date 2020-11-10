//
// Created by 高翔 on 2017/12/15.
// Edited by 谭臻 on 2020/10/13.
//

#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "../../undistorted/test.png";   

int main(int argc, char **argv) {

    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file,0);   // 图像是灰度图，CV_8UC1
    cv::imshow("gg",image);
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图

    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {

            double u_distorted = 0, v_distorted = 0;

            double x = (u-cx) / fx;
            double y = (v-cy) / fy;
            double r =  sqrt(x*x + y*y);
            // 加入畸变系数后的坐标
            // 相机坐标系（单位：m）
            double x_distorted = x*(1+k1*r*r+k2*r*r*r*r) + 2*p1*x*y + p2*(r*r+2*x*x);
            double y_distorted = y*(1+k1*r*r+k2*r*r*r*r) + 2*p2*x*y + p1*(r*r+2*y*y);
            // 像素坐标系（单位：像素）
            u_distorted = fx*x_distorted + cx;
            v_distorted = fy*y_distorted + cy;

     

            // 赋值 (最近邻插值)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }

    // 画图去畸变后图像
    cv::imshow("image distorted", image);
    cv::imshow("image undistorted", image_undistort);
    cv::waitKey();
    cv::imwrite("../../undistorted/image_undistorted.png",image_undistort);

    return 0;
}
