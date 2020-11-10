#include "sophus/se3.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
using namespace Sophus;

// path to trajectory file
string trajectory_file = "../trajectory.txt";
string groundTruth = "../groundtruth.txt";
string estimated = "../estimated.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(const TrajectoryType &poses);
void DrawTwoTrajectory(const TrajectoryType &gt, const TrajectoryType &est);

//轨迹读取函数
TrajectoryType ReadTrajectory(const string &path); 

int main(int argc, char **argv) {

    TrajectoryType poses, gt, est;

    /// implement pose reading code
    poses = ReadTrajectory(trajectory_file);
    gt = ReadTrajectory(groundTruth);
    est = ReadTrajectory(estimated);
    
    // compute rmse (root-mean-squared error)
    double rmse = 0;
    for(size_t i = 0; i <est.size(); i++){
        Sophus::SE3d p1 = est[i], p2 = gt[i];
        double error = (gt[i].inverse()*est[i]).log().norm();
        rmse += error *error;
    }

    rmse = rmse/double(est.size());
    rmse = sqrt(rmse);
    cout<<"RMSE is equal to :" << rmse<<endl;

    // draw trajectory in pangolin
    //DrawTrajectory(poses);
    //DrawTwoTrajectory(gt,est);
    return 0;
}

/******************************************************************************************/
TrajectoryType ReadTrajectory(const string &path){
    // read trajectory file
    ifstream fin(path);
    // define a trajectory object
    TrajectoryType trajectory;
    if(!fin){
        cerr<<"trajectory"<<path<<" not found."<<endl;
        return trajectory;
    }
    
    while(!fin.eof()){
        // file format: time, tranlational components, quaternion part
        double t, tx, ty, tz, qx, qy, qz, qw;
        
        fin>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qx,qy,qz,qw), Eigen::Vector3d(tx,ty,tz));
        
        trajectory.push_back(p1);
    }

    return trajectory;
}

/********************************单一轨迹画图*************************************************/
void DrawTrajectory(const TrajectoryType &poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

    /******************************轨迹对比*************************************/
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

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f,1.0f,0.0f); // green for ground truth
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        
        for (size_t i = 0; i < est.size() - 1; i++) {
            glColor3f(1.0f,0.0f,0.0f); // red for estimated
            glBegin(GL_LINES);
            auto p1 = est[i], p2 = est[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
