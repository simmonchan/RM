#include "include/Stereo_vision/stereo_vision.h"

stereo_vision::stereo_vision()
{
}

stereo_vision::stereo_vision(const string &yamlfile)
{
    FileStorage stereo_yaml(yamlfile,FileStorage::READ);
    stereo_yaml["Q"] >> Q;
    stereo_yaml.release();
}

void stereo_vision::Init(const string &yamlfile){
    FileStorage stereo_yaml(yamlfile,FileStorage::READ);
    stereo_yaml["Q"] >> Q;
    stereo_yaml.release();
}

/*void stereo_vision::cal_coordinate(const Point2f &left, const Point2f &right, AbsPosition &Position)
{
    int disp_x = left.x - right.x;
    Mat image_xyd = (Mat_<int>(4,1) << left.x,left.y,disp_x,1);
    Mat xyzw = (Mat_<float>(4,1) << 0,0,0,0);
    xyzw = Q*image_xyd;
    Position.z = xyzw.at<float>(2,0)/xyzw.at<float>(3,0);
    Position.y = xyzw.at<float>(1,0)/xyzw.at<float>(3,0);
    Position.x = xyzw.at<float>(0,0)/xyzw.at<float>(3,0);
}*/

