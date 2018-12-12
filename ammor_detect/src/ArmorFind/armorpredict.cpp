#include "include/ArmorFind/armorpredict.h"

ArmorPredict::ArmorPredict()
{
    FileStorage stereo_yaml("camera_calibrate.yaml",FileStorage::READ);
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

ArmorPredict::~ArmorPredict()
{

}

void ArmorPredict::Fresh()
{
    Positions.clear();
    yaw_old = yaw_out;
    pitch_old = pitch_out;
    yaw_out = 0;
    pitch_out = 0;
}

void ArmorPredict::Predict(vector<Point2f> &Left_Points, vector<Point2f> &Right_Points, vector<Armordata> &Left_Armor_data,vector<Armordata> &Right_Armor_data){

    Fresh();
    size_t Left_size = Left_Points.size();
    size_t Right_size = Right_Points.size();
    if(Left_size == 0 && Right_size == 0){

        memset(&Vision,0,sizeof(VisionData));
    }
    else if(Left_size!=0 || Right_size != 0){

        if(Left_size == Right_size){
            SetAxis(110,110,90);
            PredictStereo(Left_Points,Right_Points);
        }
        else if(Left_size > Right_size){
            SetAxis(110,110,90);
            PredictMono(Left_Armor_data,cameraMatrixL,distCoeffL);
        }
        else if(Left_size < Right_size){
            SetAxis(110,-110,90);
            PredictMono(Right_Armor_data,cameraMatrixR,distCoeffR);
        }
    }
}

void ArmorPredict::PredictStereo(vector<Point2f> &Left_Points, vector<Point2f> &Right_Points)
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
            float disp_x = left_distort_points[i].x - Right_distort_points[i].x;
            Mat image_xyd = (Mat_<double>(4,1) << Left_Points[i].x,Left_Points[i].y,disp_x,1);
            Mat xyzw = (Mat_<double>(4,1) << 0,0,0,0);
            xyzw = Q*image_xyd;
            Pos.z = xyzw.at<double>(2,0)/xyzw.at<double>(3,0);
            Pos.y = xyzw.at<double>(1,0)/xyzw.at<double>(3,0);
            Pos.x = xyzw.at<double>(0,0)/xyzw.at<double>(3,0);

            if(Pos.z > 50){
                Pos.x -= x_trans;
                Pos.y -= y_trans;
                Pos.z -= z_trans;
                Pos.index = i;
                Positions.push_back(Pos);
            }
        }
    }

    if(Positions.size() != 0){
        if(Positions.size() != 1){
            sort(Positions.begin(),Positions.end(),PosSort);
        }

        if(Positions.size() > 1){
//            if(fabs(Positions[0].z - Positions[1].z) < 100 && OldResult.z !=0){
//                float q0 = fabs(OldResult.x - Positions[0].x);
//                float q1 = fabs(OldResult.x - Positions[1].x);
//                Result = q0 < q1 ? Positions[0] : Positions[1];
//            }else{
                Result = Positions[0];
            //}
        }else{
            Result = Positions[0];
        }

        if(Result.z < 30){
            yaw_out = 0;
            pitch_out = 0;
        }else{
            AngleFit(Result);
        }
    }

    Vision = {pitch_out,yaw_out,0,0,1};
    OldPositions = Positions;
    OldResult = Result;
}

void ArmorPredict::AngleFit(const AbsPosition input){

    float flytime,gravity_offset;
    float shoot_distance = sqrtf(input.x * input.x + input.y * input.y + input.z * input.z)*0.001; // m
    shoot_speed = shoot_distance*10.0;
    int shootspeedmax = 30;
    if(shoot_speed > shootspeedmax){
        shoot_speed = shootspeedmax;
        flytime = shoot_distance/shoot_speed;
        gravity_offset = 4.905 * flytime * flytime;
    }else if(shoot_speed < 14){
        shoot_speed = 14;
        flytime = shoot_distance/shoot_speed;
        gravity_offset = 4.905 * flytime * flytime;
    }else{
        gravity_offset = 0.04905;
    }
    yaw_out = atan(input.x/input.z) * 180/CV_PI;
    pitch_out = atan((input.y-gravity_offset)/input.z) * 180/CV_PI;
}

void ArmorPredict::PredictMono(vector<Armordata> &Armor_datas,Mat cameraMatrix, Mat distCoeff)
{
    size_t Armor_size = Armor_datas.size();
    for(size_t i=0;i<Armor_size;i++){
        vector<Point2f> target2d(4);
        target2d[0] = Armor_datas[i].armor_points[0];
        target2d[1] = Armor_datas[i].armor_points[2];
        target2d[2] = Armor_datas[i].armor_points[3];
        target2d[3] = Armor_datas[i].armor_points[1];


        vector<Point3f> target3d(4);
        target3d.push_back(Point3f(-67.5,-27.5,0));
        target3d.push_back(Point3f(67.5,-27.5,0));
        target3d.push_back(Point3f(67.5,27.5,0));
        target3d.push_back(Point3f(-67.5,27.5,0));

        Mat r;
        Mat position_in_camera;
        solvePnP(target3d,target2d,cameraMatrix,distCoeff,r,position_in_camera,false,SOLVEPNP_EPNP);

        if(position_in_camera.at<double>(2,0) > 50){

            AbsPosition Pos;
            Pos.x = position_in_camera.at<double>(0,0) - x_trans;
            Pos.y = position_in_camera.at<double>(1,0) - y_trans;
            Pos.z = position_in_camera.at<double>(2,0) - z_trans;
            Pos.index = i;
            Positions.push_back(Pos);
        }

        if(Positions.size() != 0){
            if(Positions.size() != 1){
                sort(Positions.begin(),Positions.end(),PosSort);
            }

            if(Positions.size() > 1){
    //            if(fabs(Positions[0].z - Positions[1].z) < 100 && OldResult.z !=0){
    //                float q0 = fabs(OldResult.x - Positions[0].x);
    //                float q1 = fabs(OldResult.x - Positions[1].x);
    //                Result = q0 < q1 ? Positions[0] : Positions[1];
    //            }else{
                    Result = Positions[0];
                //}
            }else{
                Result = Positions[0];
            }

            if(Result.z < 30){
                yaw_out = 0;
                pitch_out = 0;
            }else{
                AngleFit(Result);
            }
        }

        Vision = {pitch_out,yaw_out,0,0,1};
        OldPositions = Positions;
        OldResult = Result;
    }

}


/**
  * @brief calculate the distance between each 2 3-D points
  * @param a1 a2: 3-D points
  * @return distance
  */
float ArmorPredict::CenterDis(const AbsPosition a1,const AbsPosition a2){
    float dx2 = a2.x - a1.x;
    float dy2 = a2.y - a1.y;
    float dz2 = a2.z - a1.z;
    return sqrtf(dx2*dx2 + dy2*dy2 + dz2*dz2);
}
