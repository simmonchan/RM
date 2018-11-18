#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class AngleSolver{
public:
    // 构造函数
    AngleSolver(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff,
                double target_width, double target_height, double z_scale = 1.0,
                double min_dist = 0, double max_dist = 1000){

        camera_matrix.copyTo(cam_matrix); // 相机内参
        dist_coeff.copyTo(distortion_coeff); // 畸变系数

        width_world_target = target_width; // 物体的宽
        height_world_target = target_height; // 物体的长
        min_distance = min_dist;
        max_distance = max_dist;

        rot_camera2ptz = cv::Mat::eye(3,3,CV_64FC1);
        trans_camera2ptz = cv::Mat::zeros(3,1,CV_64FC1);
        offset_y_barrel_ptz = 10e-5;
        scale_z = z_scale;
    }

    // 设置目标的尺寸
    void setTargetSize(double width, double height){
        width_world_target = width;
        height_world_target = height;
    }

    // 设置相机的参数，内参以及畸变系数
    void setCameraParam(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff){
        camera_matrix.copyTo(cam_matrix);
        dist_coeff.copyTo(distortion_coeff);
    }

    // 设置z的比例系数 和相机到云台的位姿
    void setScaleZ(double scale) {scale_z = scale;}
    void setRelationPoseCameraPTZ(const cv::Mat & rot_camera_ptz, const cv::Mat & trans_camera_ptz, double y_offset_barrel_ptz){
        rot_camera_ptz.copyTo(rot_camera2ptz);
        trans_camera_ptz.copyTo(trans_camera2ptz);
        offset_y_barrel_ptz = y_offset_barrel_ptz;
    }

    /* 求出图像的旋转矩阵和平移矩阵
       需要传入目标的四个像素坐标系，
       输出的图片的外参数        */
    void solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);

    // 得到目标的二维坐标
    void getTarget2dPoinstion(const cv::RotatedRect & rect, std::vector<cv::Point2f> & target2d, const cv::Point2f & offset);

    // 把相机坐标系转为云台坐标系
    void tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos);

    // 三种情况，抵消炮管偏移的影响
    void adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed = 0.0, double current_ptz_angle = 0.0);

    // 得到角度
    bool getAngle(const cv::RotatedRect & rect, double & angle_x, double & angle_y, double bullet_speed = 0, double current_ptz_angle = 0.0, const cv::Point2f & offset = cv::Point2f());


public:
    cv::Mat position_in_camera; // 目标在相机坐标系中的位置
    cv::Mat position_in_ptz; // 目标在云台坐标系的位置
private:
    cv::Mat cam_matrix; // 相机内参
    cv::Mat distortion_coeff; // 畸变系数
    double width_world_target; // 实际装甲板的宽
    double height_world_target; // 实际装甲板的长
    cv::Mat trans_camera2ptz; // 相机到云台的平移矩阵
    cv::Mat rot_camera2ptz; // 相机到云台的旋转矩阵
    double offset_y_barrel_ptz; // 炮台到云台的坐标系y的距离
    double min_distance; // 最小距离
    double max_distance; // 最大距离
    double scale_z; // z的比例系数
};
