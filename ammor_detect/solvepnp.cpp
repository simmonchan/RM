#include "solvepnp.h"

/*********************AngleSolver类***************/

// 求出图片的外参数
void AngleSolver::solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans)
{
    // 如果目标检测不到，返回
    if(width_world_target < 10e-5 || height_world_target < 10e-5){
        rot = cv::Mat::eye(3,3,CV_64FC1);
        trans = cv::Mat::zeros(3,1,CV_64FC1);
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
void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect & rect, std::vector<cv::Point2f> & target2d, const cv::Point2f & offset = Point2f(0,0)){
    Point2f vertices[4];
    rect.points(vertices);
    // 确定装甲板的四个像素
    Point2f lu, ld, ru, rd;
    // 给四个点的x排序
    sort(vertices, vertices + 4, [](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; });
    // 给四个点的y排序
    if (vertices[0].y < vertices[1].y){
        lu = vertices[0];
        ld = vertices[1];
    }
    else{
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)	{
        ru = vertices[2];
        rd = vertices[3];
    }
    else {
        ru = vertices[3];
        rd = vertices[2];
    }

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
    // 定义一个指向平移向量的指针
    const double *_xyz = (const double *)pos_in_ptz.data;
    // 下落时间
    double down_t = 0.0;
    // 近似为匀速运动，算出子弹到达目标的时间
    if (bullet_speed > 10e-3)
        down_t = _xyz[2] / 100.0 / bullet_speed;
    // 计算y轴上下落的距离
    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 100;
    // 重新更新位置
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
    // 两个角度
    double alpha = 0.0, theta = 0.0;
    // 云台需要转的角度
    alpha = asin(offset_y_barrel_ptz/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));
    // 目标在上方
    if(xyz[1] < 0){
        theta = atan(-xyz[1]/xyz[2]);
        angle_y = -(alpha+theta);  // camera coordinate
    }
    // 目标在z轴和炮管之间
    else if (xyz[1] < offset_y_barrel_ptz){
        theta = atan(xyz[1]/xyz[2]);
        angle_y = -(alpha-theta);  // camera coordinate
    }
    // 目标在炮管下面
    else{
        theta = atan(xyz[1]/xyz[2]);
        angle_y = (theta-alpha);   // camera coordinate
    }
    angle_x = atan2(xyz[0], xyz[2]);

    // 得到角度
    angle_x = angle_x * 180 / 3.1415926;
    angle_y = angle_y * 180 / 3.1415926;
}


// 解出角度的函数
bool AngleSolver::getAngle(const cv::RotatedRect & rect, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f & offset){
    if (rect.size.height < 1)
        return false;

    // 得到这个目标旋转举行的四个坐标
    vector<Point2f> target2d;
    getTarget2dPoinstion(rect, target2d, offset);

    // 解pnp，得到外参，position_in_camera就是目标在相机坐标系下的位置
    cv::Mat r;
    solvePnP4Points(target2d, r, position_in_camera);

    // 目标在z轴上的距离
    position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);
    cout << "z轴的距离为:" << position_in_camera.at<double>(2, 0) <<endl;
    // 如果超出范围就返回
    if (position_in_camera.at<double>(2, 0) < min_distance || position_in_camera.at<double>(2, 0) > max_distance){
        cout << "out of range: [" << min_distance << ", " << max_distance << "]\n";
        return false;
    }


    // 把相机坐标系转到云台坐标系
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);

    // 得出角度
    adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y, bullet_speed, current_ptz_angle);

    return true;
}
