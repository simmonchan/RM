#ifndef STEREO_VISION_H
#define STEREO_VISION_H
#include "include/Header.h"



class stereo_vision
{
public:
    stereo_vision();
    stereo_vision(const string &yamlfile);
    void Init(const string &yamlfile);
    //void cal_coordinate(const Point2f &left, const Point2f &right, AbsPosition &Position);
private:
    // 反投影矩阵
    Mat Q;
};

#endif // STEREO_VISION_H
