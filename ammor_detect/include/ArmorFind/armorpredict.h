#ifndef ARMORPREDICT_H
#define ARMORPREDICT_H
#include "include/Header.h"
#include "include/Serial/serialport.h"
#include "include/Stereo_vision/solvepnp.h"
#include "include/Stereo_vision/stereo_vision.h"

static inline bool PosSort(const AbsPosition a1,const AbsPosition a2){
    return a1.z < a2.z;
}

class ArmorPredict
{
public:
    ArmorPredict();
    void Predict(vector<AbsPosition> Positions);

private:
    void AngleFit(const AbsPosition input);
    void Fresh();
public:
    AbsPosition Result, OldResult;
    VisionData Vision;
private:
    vector<AbsPosition> OldPositions;
    float yaw_out,pitch_out;
    float yaw_old,pitch_old;
    float shoot_speed;
};

#endif // ARMORPREDICT_H
