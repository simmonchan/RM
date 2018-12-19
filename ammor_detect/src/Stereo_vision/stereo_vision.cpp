#include "include/Stereo_vision/stereo_vision.h"

stereo_vision::stereo_vision()
{
    FileStorage stereo_yaml("/home/s305/build-ammor_detect-Desktop-Debug/camera_calibrate.yaml",FileStorage::READ);
    stereo_yaml["cameraMatrixL"] >> cameraMatrixL;
    stereo_yaml["cameraMatrixR"] >> cameraMatrixR;
    stereo_yaml["distCoeffL"] >> distCoeffL;
    stereo_yaml["distCoeffR"] >> distCoeffR;
    stereo_yaml["Rl"] >> Rl;
    stereo_yaml["Pl"] >> Pl;
    stereo_yaml["Rr"] >> Rr;
    stereo_yaml["Pr"] >> Pr;
    stereo_yaml["Q"] >> Q;
    stereo_yaml.release();
}

void stereo_vision::Init(const string &yaml){
    FileStorage stereo_yaml(yaml,FileStorage::READ);
    stereo_yaml["cameraMatrixL"] >> cameraMatrixL;
    stereo_yaml["cameraMatrixR"] >> cameraMatrixR;
    stereo_yaml["distCoeffL"] >> distCoeffL;
    stereo_yaml["distCoeffR"] >> distCoeffR;
    stereo_yaml["Rl"] >> Rl;
    stereo_yaml["Pl"] >> Pl;
    stereo_yaml["Rr"] >> Rr;
    stereo_yaml["Pr"] >> Pr;
    stereo_yaml["Q"] >> Q;
    stereo_yaml.release();
}

/**
  * @brief get the stereo_vision location
  * @param Left_Points: the left armor points
  * @param Right_Points: the Right armor points
  * @param get the location vector
  * @return none
  */
void stereo_vision::get_location(const vector<Point2f> &Left_Points, const vector<Point2f> &Right_Points, vector<AbsPosition> &Result)
{

    vector<Point2f> left_distort_points,Right_distort_points;
    undistortPoints(Left_Points,left_distort_points,cameraMatrixL,distCoeffL,Rl,Pl);
    undistortPoints(Right_Points,Right_distort_points,cameraMatrixR,distCoeffR,Rr,Pr);

    auto sort_point = [](const Point &a1,const Point &a2){
        return a1.x < a2.x;
    };
    sort(left_distort_points.begin(),left_distort_points.end(),sort_point);
    sort(Right_distort_points.begin(),Right_distort_points.end(),sort_point);

    size_t Lsize = Left_Points.size();

    for(size_t i=0;i<Lsize;i++){

        if(left_distort_points[i].x > Right_distort_points[i].x){

            AbsPosition Pos;
            double disp_x = left_distort_points[i].x - Right_distort_points[i].x;
            Mat image_xyd = (Mat_<double>(4,1) << left_distort_points[i].x,Right_distort_points[i].y,disp_x,1);
            Mat xyzw = (Mat_<double>(4,1) << 0,0,0,0);
            xyzw = Q*image_xyd;
            Pos.z = xyzw.at<double>(2,0)/xyzw.at<double>(3,0);
            Pos.y = xyzw.at<double>(1,0)/xyzw.at<double>(3,0);
            Pos.x = xyzw.at<double>(0,0)/xyzw.at<double>(3,0);

            if(Pos.z > 50){
                Pos.x -= x_tranz;
                Pos.y -= y_tranz;
                Pos.z -= z_tranz;
                Pos.index = i;
                Result.push_back(Pos);
            }
        }
    }
}

