#include "stereo_vision.h"

stereo_vision::stereo_vision()
{
    FileStorage fs2("camera_calibrate.yaml",FileStorage::READ);
    fs2["cameraMatrixL"] >> cameraMatrixL;
    fs2["distCoeffL"] >> distCoeffL;
    fs2["cameraMatrixR"] >> cameraMatrixR;
    fs2["distCoeffR"] >> distCoeffR;
    fs2["Rl"] >> Rl;
    fs2["Rr"] >> Rr;
    fs2["Pl"] >> Pl;
    fs2["Pr"] >> Pr;
    fs2["Q"] >> Q;
    width_world_target = 13.5;
    height_world_target = 6.5;
    rot_camera2ptz = cv::Mat::eye(3,3,CV_64FC1);
    trans_camera2ptz = cv::Mat::zeros(3,1,CV_64FC1);
    scale_z = 1;
    min_distance = 50;
    max_distance = 700;
}

void stereo_vision::solvePnP4Points(const Mat & cameraMatrix, const Mat & distCoeff,const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans)
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
    cv::solvePnP(point3d, points2d, cameraMatrix, distCoeff, r, trans);
    Rodrigues(r, rot);
}

void stereo_vision::getTarget2dPoinstion(cv::Point vertices[4], std::vector<cv::Point2f> & target2d, const cv::Point & offset)
{
    target2d.clear();
    target2d.push_back(vertices[0] + offset);
    target2d.push_back(vertices[2] + offset);
    target2d.push_back(vertices[3] + offset);
    target2d.push_back(vertices[1] + offset);
}

void stereo_vision::tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos)
{
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}

void stereo_vision::monocular_get_distance(vector<Armordata> &data,bool mode,const cv::Point offset)
{
    for(size_t i=0;i<data.size();i++)
     {
        vector<Point2f> target2d;
        getTarget2dPoinstion(data[i].armor_points, target2d, offset);
        cv::Mat r;
        if(mode == LEFT){
            solvePnP4Points(cameraMatrixL,distCoeffL,target2d, r, position_in_camera);
        }
        else{
            solvePnP4Points(cameraMatrixR,distCoeffR,target2d, r, position_in_camera);
        }

        tranformationCamera2PTZ(position_in_camera, position_in_ptz);
        double x = position_in_ptz.at<double>(0,0);
        double y = position_in_ptz.at<double>(1,0);
        double z = position_in_ptz.at<double>(2,0);
        data[i].distance = sqrt(x*x + y*y + z*z);
    }
}

void stereo_vision::stereo_get_distance( vector<Point> &Left, vector<Point> &Right,vector<Armordata> &data)
{
#ifdef DEBUG
     cout << "矫正前的:" << left_center << endl;
     cout << "校正前的:" << right_center << endl;
#endif
    undistortPoints(Left,Left,cameraMatrixL,distCoeffL,Rl,Pl);
    undistortPoints(Right,Right,cameraMatrixR,distCoeffR,Rr,Pr);
    for(size_t i=0;i<Left.size();i++)
    {
        int disp_x = Left[i].x - Right[i].x;
        Mat image_xyd = (Mat_<double>(4,1) << 0,0,disp_x,1);
        Mat xyzw = (Mat_<double>(4,1) << 0,0,0,0);
        xyzw = Q*image_xyd;
        double z = xyzw.at<double>(2,0)/xyzw.at<double>(3,0);
        double x = xyzw.at<double>(0,0)*z/Q.at<double>(2,3);
        double y = xyzw.at<double>(1,0)*z/Q.at<double>(2,3);
        data[i].distance = z;
    }
#ifdef DEBUG
     cout << "矫正后的:" << left_center << endl;
     cout << "校正后的:" << right_center << endl;
#endif
}

