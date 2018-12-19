#include "two_camera.h"

Two_camera::Two_camera()
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
}


void Two_camera::get_distance(vector<Point2d> point_center_L,vector<Point2d> point_center_R, double &distance)
{
    center_point_L = point_center_L;
    center_point_R = point_center_R;
    cout << "校正前的左图：center_point_L" << center_point_L<< endl;
    cout << "校正前的右图：center_point_R" << center_point_R<< endl;

    // 对两个中心点进行校正
    undistortPoints(center_point_L,center_point_L,cameraMatrixL,distCoeffL,Rl,Pl);
    undistortPoints(center_point_R,center_point_R,cameraMatrixR,distCoeffR,Rr,Pr);
    cout << "校正后的左图：center_point_L" << center_point_L << endl;
    cout << "校正后的右图：center_point_R" << center_point_R << endl;
    if(center_point_L[0].x < 0 || center_point_R[0].x< 0)
    {
        cout << "z:" << 0 << endl;
    }
    // 计算出实际的三维坐标
    else
    {
        int disp_x = center_point_L[0].x - center_point_R[0].x;
        Mat image_xyd = (Mat_<double>(4,1) << 0,0,disp_x,1);
        Mat xyzw = (Mat_<double>(4,1) << 0,0,0,0);
        xyzw = Q*image_xyd;
        cout << "z:" << xyzw.at<double>(2,0)/xyzw.at<double>(3,0) << endl;
    }
}
