#include "stereo_vision.h"

stereo_vision::stereo_vision()
{
    FileStorage stereo_xml("camera_calibrate.yaml",FileStorage::READ);
    stereo_xml["cameraMatrixL"] >> cameraMatrixL;
    stereo_xml["distCoeffL"] >> distCoeffL;
    stereo_xml["cameraMatrixR"] >> cameraMatrixR;
    stereo_xml["distCoeffR"] >> distCoeffR;
    stereo_xml["Rl"] >> Rl;
    stereo_xml["Rr"] >> Rr;
    stereo_xml["Pl"] >> Pl;
    stereo_xml["Pr"] >> Pr;
    stereo_xml["Q"] >> Q;
    stereo_xml.release();
}

void stereo_vision::stereo_get_distance( vector<Point> &Left, vector<Point> &Right,vector<Armordata> &L_data, vector<Armordata> R_data )
{
#ifdef SHOW_DEBUG
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
        //double x = xyzw.at<double>(0,0)*z/Q.at<double>(2,3);
        //double y = xyzw.at<double>(1,0)*z/Q.at<double>(2,3);
        L_data[i].distance = z;
        R_data[i].distance = z;
    }
#ifdef SHOW_DEBUG
     cout << "矫正后的:" << left_center << endl;
     cout << "校正后的:" << right_center << endl;
#endif
}

