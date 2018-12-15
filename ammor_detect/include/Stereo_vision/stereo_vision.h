#ifndef STEREO_VISION_H
#define STEREO_VISION_H
#include "include/Header.h"

class stereo_vision
{
public:
    stereo_vision();
    void Init(const string &yaml);
    void setAxis(float x, float y, float z){
        x_tranz = x;
        y_tranz = y;
        z_tranz = z;
    }
    void get_location(const vector<Point2f> &left, const vector<Point2f> &right, vector<AbsPosition> &Result);
private:
    Mat cameraMatrixL;
    Mat cameraMatrixR;
    Mat distCoeffL;
    Mat distCoeffR;
    Mat Rl;
    Mat Pl;
    Mat Rr;
    Mat Pr;
    Mat Q;

    float x_tranz, y_tranz, z_tranz;
};

#endif // STEREO_VISION_H
