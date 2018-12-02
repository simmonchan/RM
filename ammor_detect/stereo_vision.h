#ifndef STEREO_VISION_H
#define STEREO_VISION_H
#include "Header.h"

//#define DEBUG

class stereo_vision
{
public:
    stereo_vision();
    void setTargetSize(double width, double height){
        width_world_target = width;
        height_world_target = height;
    }
    void setScaleZ(double scale) {scale_z = scale;}
    void setRelationPoseCameraPTZ(const cv::Mat & rot_camera_ptz, const cv::Mat & trans_camera_ptz){
        rot_camera_ptz.copyTo(rot_camera2ptz);
        trans_camera_ptz.copyTo(trans_camera2ptz);
    }

    void solvePnP4Points(const Mat & cameraMatrix, const Mat & distCoeff,const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);

    void getTarget2dPoinstion(cv::Point vertices[4], std::vector<cv::Point2f> & target2d, const cv::Point & offset);

    void tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos);

    void monocular_get_distance(vector<Armordata> &data,bool mode,const cv::Point offset);

    void stereo_get_distance( vector<Point> &Left, vector<Point> &Right,vector<Armordata> &data);

public:
    cv::Mat position_in_camera;
    cv::Mat position_in_ptz;
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
    vector<float> distances;
    // pnp问题
    double width_world_target;
    double height_world_target;
    cv::Mat trans_camera2ptz;
    cv::Mat rot_camera2ptz;
    double min_distance;
    double max_distance;
    double scale_z;

};

#endif // STEREO_VISION_H
