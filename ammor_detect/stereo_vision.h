#ifndef STEREO_VISION_H
#define STEREO_VISION_H
#include "Header.h"

//#define SHOW_DEBUG

class stereo_vision
{
public:
    stereo_vision();
    void stereo_get_distance(vector<Point2f> &Left, vector<Point2f> &Right,vector<Armordata> &L_data, vector<Armordata> &R_data);
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
};

#endif // STEREO_VISION_H
