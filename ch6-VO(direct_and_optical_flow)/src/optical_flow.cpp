//
// Edited by Zhen Tan on 29/10/2020
//

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono> // 计时库

using namespace std;
using namespace cv;

// this program shows how to use optical flow

string file_1 = "./../left.png";  // first image
string file_2 = "./../right.png";  // second image
string file_disparity = "./../disparity.png"; // disparity image

/**
 * single level optical flow using parallel calculation
 * @param range
 */
void OpticalFlowSingleParallel(const Range *range);

/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);


/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 *
 * @param img1
 * @param img2
 * @param kp1
 * @param kp2
 * @param success
 * @param inverse
 */
void DisparityOpticalFlow(const Mat &img1, const Mat &img2, const vector<KeyPoint> &kp1, vector<KeyPoint> &kp2,
                          vector<bool> &success, bool inverse = false);


/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


int main(int argc, char **argv) {

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // now lets track these key points in the second image
    // first use single level LK in the validation picture
    vector<KeyPoint> kp2_single;
    vector<bool> success_single;
    auto start = chrono::steady_clock::now();
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single, true);
    auto end = chrono::steady_clock::now();
    auto duration = chrono::duration_cast<chrono::duration<double>>(end - start);
    cout << "optical flow by gauss-newton count" << duration.count() << "s" << endl;

    // then test multi-level LK
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    start = chrono::steady_clock::now();
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true);
    end = chrono::steady_clock::now();
    duration = chrono::duration_cast<chrono::duration<double>>(end - start);
    cout << "optical flow by multi-level gauss-newton (pyramid) count" << duration.count() << "s" << endl;

    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    start = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, cv::Size(8, 8));
    end = chrono::steady_clock::now();
    duration = chrono::duration_cast<chrono::duration<double>>(end - start);
    cout << "optical flow by opencv" << duration.count() << "s" << endl;

    // plot the differences of those functions
    Mat img2_single;
    cv::cvtColor(img2, img2_single, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++) {
        if (success_single[i]) {
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_CV);

    // 利用光流法计算视差
    DisparityOpticalFlow(img1, img2, kp1, kp2_multi, success_multi, true);

    cv::waitKey(0);
    // write image
//    cv::imwrite("img_single_flow.jpg", img2_single);
//    cv::imwrite("img_multi_level.jpg", img2_multi);
//    cv::imwrite("img_opencv.jpg", img2_CV);

    return 0;
}

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    //parallel_for_(Range(0, kp1.size()), std::bind(&))

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            if (inverse == false) H = Eigen::Matrix2d::Zero(); // inverse approach only compute H once
            b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    float x1 = x + kp.pt.x;
                    float y1 = y + kp.pt.y;
                    double error = GetPixelValue(img1, x1, y1) - GetPixelValue(img2, x1 + dx, y1 + dy);

                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        J[0] = -(GetPixelValue(img2, x1 + dx + 1, y1 + dy) -
                                 GetPixelValue(img2, x1 + dx - 1, y1 + dy)) /
                               2.0; // Ix/dx
                        J[1] = -(GetPixelValue(img2, x1 + dx, y1 + dy + 1) -
                                 GetPixelValue(img2, x1 + dx, y1 + dy - 1)) /
                               2.0; // Iy/dy
                    } else if (iter == 0) {
                        // Inverse Jacobian (Based on template)
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J[0] = -(GetPixelValue(img1, x1 + 1, y1) - GetPixelValue(img1, x1 - 1, y1)) / 2.0;
                        J[1] = -(GetPixelValue(img1, x1, y1 + 1) - GetPixelValue(img1, x1, y1 - 1)) / 2.0;
                    }

                    // compute H, b and set cost;
                    if (inverse == false || iter == 0) {
                        H += J * J.transpose(); // Only forward approach will update Hessian matrix
                    }

                    b += -J * error;
                    cost += error * error;
                }

            // compute update
            Eigen::Vector2d update;
            update = H.inverse() * b;
            //update = H.ldlt().solve(b);

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                //cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // 对图像进行分层
    for (int i = 0; i < pyramids; i++) {
        if (i == 0) {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        } else {
            Mat temp1, temp2;
            cv::resize(pyr1[i - 1], temp1,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], temp2,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(temp1);
            pyr2.push_back(temp2);
        }
    }

    // 对特征点也要进行分层,首先提取金字塔顶层特征点,存入kp_pyr1,kp_pyr2
    vector<KeyPoint> kp_pyr1, kp_pyr2;
    for (auto &kp:kp1) {
        auto tmp = kp;
        tmp.pt = kp.pt * scales[pyramids - 1];
        kp_pyr1.push_back(tmp);
        kp_pyr2.push_back(tmp);
    }

    // coarse-to-fine LK tracking in pyramids

    for (int level = pyramids - 1; level >= 0; level--) {
        // from coarse to fine
        success.clear();
        //t1 = chrono::steady_clock::now();
        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp_pyr1, kp_pyr2, success, true);
        //t2 = chrono::steady_clock::now();
        //auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        //cout << "track pyr " << level << " cost time: " << time_used.count() << endl;

        // 提取下一层特征点
        if (level > 0) {
            for (auto &kp: kp_pyr1)
                kp.pt /= pyramid_scale;
            for (auto &kp: kp_pyr2)
                kp.pt /= pyramid_scale;
        }
    }

    // don't forget to set the results into kp2
    for (auto &kp: kp_pyr2)
        kp2.push_back(kp);
}

void DisparityOpticalFlow(const Mat &img1, const Mat &img2, const vector<KeyPoint> &kp1, vector<KeyPoint> &kp2,
                          vector<bool> &success, bool inverse) {
    // read disparity image
    Mat disparity = imread(file_disparity, 0);
    imshow("disparity image", disparity);

    OpticalFlowMultiLevel(img1, img2, kp1, kp2, success, false);

    Mat img1_offset, dif_disparity;
    cv::cvtColor(img1, img1_offset, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img1, dif_disparity, cv::COLOR_GRAY2BGR);
    Mat match_img;
    vector<cv::DMatch> matches;
    double error = 0.;
    // Generate pointcloud;
    //vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    for (int i = 0; i < kp1.size(); i++) {
        if (success[i]) {
            DMatch tmp;
            // the disparity value derived from disparity image
            double disparity_gt = double(disparity.at<uchar>(kp1[i].pt.y, kp1[i].pt.x));
            double disparity_est = double(kp1[i].pt.x - kp2[i].pt.x);
            tmp.queryIdx = i;
            tmp.trainIdx = i;
            tmp.distance = float(abs(disparity_gt - disparity_est));
            cout << disparity_gt << "  " << disparity_est << "  " << tmp.distance << endl;
            error += tmp.distance;
            matches.push_back(tmp);

            // plot bias in x axis
            cv::line(img1_offset, kp1[i].pt, cv::Point2d(kp1[i].pt.x + disparity_est, kp1[i].pt.y),
                     cv::Scalar(0, 250, 0));
            cv::circle(dif_disparity,kp1[i].pt,1,cv::Scalar(0,200,0));
            cv::line(dif_disparity, kp1[i].pt, cv::Point2d(kp1[i].pt.x + tmp.distance, kp1[i].pt.y), cv::Scalar(0, 250, 0));
        }
    }
    cout << "error = " << error << endl;
    cout << "the average of the error is " << error / kp2.size() << endl;

    cv::drawMatches(img1, kp1, img2, kp2, matches, match_img);
    imshow("Match Image", match_img);
    imshow("the x offset image", img1_offset);
    imshow("the comparison of disparity between groud truth and estimation", dif_disparity);

}

