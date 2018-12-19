#include "include/Stereo_vision/solvepnp.h"

AngleSolver::AngleSolver(){

}

/**
  * @brief init the AngleSolver
  * @param
  * @return none
  */
void AngleSolver::Init(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff,
                       double target_width, double target_height){

    camera_matrix.copyTo(cam_matrix);
    dist_coeff.copyTo(distortion_coeff);

    width_world_target = target_width;
    height_world_target = target_height;
}

/**
  * @brief get the Target2d Position
  * @param  Point2d[4]: the armor four points (lu,ld,ru,rd)
  * @return none
  */
void AngleSolver::getTarget2dPoinstion(Point2f points2d[4]){
    /* pt
     * 0 2
     * 1 3
     * */
    target2d.push_back(points2d[0]);
    target2d.push_back(points2d[1]);
    target2d.push_back(points2d[2]);
    target2d.push_back(points2d[3]);
}

/**
  * @brief get the Target3d Position
  * @param  trans: the location in the xyz in camara
  * @return none
  */
void AngleSolver::solvePnP4Points(cv::Mat & trans)
{
    if(width_world_target < 10e-5 || height_world_target < 10e-5){
        trans = cv::Mat::zeros(3,1,CV_64FC1);
        return;
    }

    float half_x = width_world_target / 2.0;
    float half_y = height_world_target / 2.0;

    target3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    target3d.push_back(cv::Point3f(-half_x, half_y, 0));
    target3d.push_back(cv::Point3f(half_x, -half_y, 0));
    target3d.push_back(cv::Point3f(half_x, half_y, 0));

    cv::Mat r;
    cv::solvePnP(target3d, target2d, cam_matrix, distortion_coeff, r, trans,false,SOLVEPNP_EPNP);
}

/**
  * @brief get the Target3d Position
  * @param  Armor:to get the target2D points
  * @param Result get the offset Pos
  * @return none
  */
void AngleSolver::get_location(vector<Armordata> &Armor,vector<AbsPosition> &Result)
{
    size_t size = Armor.size();
    for(size_t i=0;i<size;i++){
        clear();
        getTarget2dPoinstion(Armor[i].armor_points);
        solvePnP4Points(position_in_camera);

        AbsPosition Pos;
        Pos.x = position_in_camera.at<double>(0, 0) - x_trans;
        Pos.y = position_in_camera.at<double>(1, 0) - y_trans;
        Pos.z = position_in_camera.at<double>(2, 0) - z_trans;

        if(Pos.z > 50){
            Pos.x -= x_trans;
            Pos.y -= y_trans;
            Pos.z -= z_trans;
            Pos.index = i;
            Result.push_back(Pos);
        }
    }
}

/**
  * @brief to clear the vector
  * @param  none
  * @return none
  */
void AngleSolver::clear(){
    target2d.clear();
    target3d.clear();
}
