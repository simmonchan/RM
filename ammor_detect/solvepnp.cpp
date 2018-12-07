#include "solvepnp.h"

/*********************AngleSolver类***************/

// 求出图片的外参数
void AngleSolver::solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans)
{
    // 如果目标检测不到，返回
    if(width_world_target < 10e-5 || height_world_target < 10e-5){
        rot = cv::Mat::eye(3,3,CV_64FC1);
        trans = cv::Mat::zeros(3,1,CV_64FC1);
        return;
    }

    // 以中心为(0,0,0)，目标的四个坐标作为已知的世界坐标
    std::vector<cv::Point3f> point3d;
    double half_x = width_world_target / 2.0;
    double half_y = height_world_target / 2.0;

    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));

    // 旋转向量转为旋转矩阵
    cv::Mat r;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);
}

// 得到目标的像素坐标
void AngleSolver::getTarget2dPoinstion(Point points2d[4],std::vector<cv::Point2f> & target2d, const cv::Point2f & offset = Point2f(0,0)){
    // 确定装甲板的四个像素
    Point2f lu, ld, ru, rd;
    lu = points2d[0];
    ld = points2d[1];
    ru = points2d[2];
    rd = points2d[3];

    target2d.clear();
    // 加上偏移的坐标，默认为0
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}


// 把相机坐标系转到云台坐标系
void AngleSolver::tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos){
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}


// 抵消炮管下移的影响，得到角度
void AngleSolver::adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle){
    const double *_xyz = (const double *)pos_in_ptz.data;

    double down_t = 0.0;
    if (bullet_speed > 10e-3)
        down_t = _xyz[2] / 100.0 / bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 100;
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};

    double alpha = 0.0, theta = 0.0;
    alpha = asin(offset_y_barrel_ptz/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));
    if(xyz[1] < 0){
        theta = atan(-xyz[1]/xyz[2]);
        angle_y = -(alpha+theta);  // camera coordinate
    }
    else if (xyz[1] < offset_y_barrel_ptz){
        theta = atan(xyz[1]/xyz[2]);
        angle_y = -(alpha-theta);  // camera coordinate
    }
    else{
        theta = atan(xyz[1]/xyz[2]);
        angle_y = (theta-alpha);   // camera coordinate
    }
    angle_x = atan2(xyz[0], xyz[2]);

    angle_x = angle_x * 180 / 3.1415926;
    angle_y = angle_y * 180 / 3.1415926;
}


// 解出角度的函数
bool AngleSolver::getAngle(Armordata armordata, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f & offset){
    vector<Point2f> target2d;
    getTarget2dPoinstion(armordata.armor_points, target2d, offset);

    cv::Mat r;
    solvePnP4Points(target2d, r, position_in_camera);

    tranformationCamera2PTZ(position_in_camera, position_in_ptz);

    position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);

    adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y, bullet_speed, current_ptz_angle);

    return true;
}

// 单目计算距离
void AngleSolver::get_distance(vector<Armordata> Armordatas,const cv::Point2f & offset)
{
    for(size_t i=0;i<Armordatas.size();i++){

        vector<Point2f> target2d;
        getTarget2dPoinstion(Armordatas[i].armor_points, target2d, offset);

        cv::Mat r;
        solvePnP4Points(target2d, r, position_in_camera);

        tranformationCamera2PTZ(position_in_camera, position_in_ptz);

        double z = scale_z * position_in_camera.at<double>(2, 0);
        double x = position_in_camera.at<double>(0, 0);
        double y = position_in_camera.at<double>(1, 0);

        Armordatas[i].distance = sqrt(z*z + x*x + y*y);
    }
}
