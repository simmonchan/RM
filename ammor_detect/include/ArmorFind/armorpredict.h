#ifndef ARMORPREDICT_H
#define ARMORPREDICT_H
#include "include/Header.h"
#include "include/Serial/serialport.h"

typedef struct{
    float x;
    float y;
    float z;
    int index;
}AbsPosition;

static inline bool distance_sort(const Armordata & L1, const Armordata & L2){
    return L1.distance < L2.distance;
}

static inline bool PosSort(const AbsPosition a1,const AbsPosition a2){
    return a1.z < a2.z;
}

class ArmorPredict
{
public:
    ArmorPredict();
    ~ArmorPredict();
    void SetAxis(float x, float y, float z){
        x_trans = x;
        y_trans = y;
        z_trans = z;
    }
    void Predict(vector<Point2f> &Left_Points, vector<Point2f> &Right_Points, vector<Armordata> &Left_Armor_data,vector<Armordata> &Right_Armor_data);
    void PredictStereo(vector<Point2f> &Left_Points, vector<Point2f> &Right_Points);
    void AngleFit(const AbsPosition input);
    void PredictMono(vector<Armordata> &Armor_datas,Mat cameraMatrix, Mat distCoeff);
private:
    float CenterDis(const AbsPosition a1, const AbsPosition a2);
    void Fresh();
public:
    AbsPosition Result, OldResult;
    VisionData Vision;

private:
    float x_trans,y_trans,z_trans;
    vector<AbsPosition> Positions,OldPositions;
    float yaw_out,pitch_out;
    float yaw_old,pitch_old;
    float shoot_speed;

    Mat cameraMatrixL;
    Mat cameraMatrixR;
    Mat distCoeffL;
    Mat distCoeffR;
    Mat Rl;
    Mat Pl;
    Mat Rr;
    Mat Pr;
    Mat Q;
};

#endif // ARMORPREDICT_H
