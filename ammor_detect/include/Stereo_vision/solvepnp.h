#ifndef SOLVEPNP_H
#define SOLVEPNP_H
#include "include/Header.h"

class AngleSolver
{
public:
    AngleSolver();
    void Init(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff,
                double target_width, double target_height);

    void setTargetSize(double width, double height){
        width_world_target = width;
        height_world_target = height;
    }

    void setCameraParam(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff){
        camera_matrix.copyTo(cam_matrix);
        dist_coeff.copyTo(distortion_coeff);
    }

    void set_Axis(float x, float y, float z)
    {
        x_trans = x;
        y_trans = y;
        z_trans = z;
    }

    void solvePnP4Points(cv::Mat &trans);
    void getTarget2dPoinstion(Point2f points2d[4]);
    void get_location(vector<Armordata> &Armor,vector<AbsPosition> Result);
    void clear();

private:
    Mat cam_matrix;
    Mat distortion_coeff;
    float width_world_target;
    float height_world_target;
    float x_trans,y_trans,z_trans;
    Mat position_in_camera;
    vector<Point2f> target2d;
    vector<Point3f> target3d;
};
#endif
