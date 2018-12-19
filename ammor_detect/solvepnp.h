#include "Header.h"
using namespace std;
using namespace cv;

class AngleSolver{
public:
    AngleSolver(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff,
                double target_width, double target_height, double z_scale = 1.0,
                double min_dist = 500, double max_dist = 7000){

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
    void setTargetSize(double width, double height){
        width_world_target = width;
        height_world_target = height;
    }

    void setCameraParam(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff){
        camera_matrix.copyTo(cam_matrix);
        dist_coeff.copyTo(distortion_coeff);
    }

    void setScaleZ(double scale) {scale_z = scale;}
    void setRelationPoseCameraPTZ(const cv::Mat & rot_camera_ptz, const cv::Mat & trans_camera_ptz, double y_offset_barrel_ptz){
        rot_camera_ptz.copyTo(rot_camera2ptz);
        trans_camera_ptz.copyTo(trans_camera2ptz);
        offset_y_barrel_ptz = y_offset_barrel_ptz;
    }
    void solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);
    void getTarget2dPoinstion(Point points2d[4], std::vector<cv::Point2f> & target2d, const cv::Point2f & offset);
    void tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos);
    void adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle);
    bool getAngle(Armordata &armordata, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f & offset);
    void get_distance(vector<Armordata> &Armordatas,const cv::Point2f & offset);



public:
    cv::Mat position_in_camera;
    cv::Mat position_in_ptz;
private:
    cv::Mat cam_matrix;
    cv::Mat distortion_coeff;

    double width_world_target;
    double height_world_target;

    cv::Mat trans_camera2ptz;
    cv::Mat rot_camera2ptz;

    double offset_y_barrel_ptz;
    double min_distance;
    double max_distance;
    double scale_z;
};
