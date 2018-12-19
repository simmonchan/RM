#ifndef TWO_CAMERA_H
#define TWO_CAMERA_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class Two_camera
{
public:
    Two_camera();
    void get_distance(vector<Point2d> point_center_L,vector<Point2d> point_center_R, double &distance);

private:
    // 相机内参
    Mat cameraMatrixL;
    Mat distCoeffL;
    Mat cameraMatrixR;
    Mat distCoeffR;
    // 旋转矩阵，投影矩阵，反投影矩阵
    Mat Rl;
    Mat Rr;
    Mat Pl;
    Mat Pr;
    Mat Q;
    // 目标点
    vector<Point2d> center_point_L;
    vector<Point2d> center_point_R;

};

#endif // TWO_CAMERA_H
