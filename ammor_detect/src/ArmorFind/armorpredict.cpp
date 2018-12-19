#include "include/ArmorFind/armorpredict.h"

ArmorPredict::ArmorPredict(){
}

void ArmorPredict::Fresh()
{
    yaw_old = yaw_out;
    pitch_old = pitch_out;
    yaw_out = 0;
    pitch_out = 0;
}

void ArmorPredict::Predict(vector<AbsPosition> &Positions){
    Fresh();
    if(Positions.size() != 0){
        if(Positions.size() != 1){
            sort(Positions.begin(),Positions.end(),PosSort);
        }

        if(Positions.size() > 1){
            if(fabs(Positions[0].z - Positions[1].z) < 100 && OldResult.z !=0){
                float q0 = fabs(OldResult.x - Positions[0].x);
                float q1 = fabs(OldResult.x - Positions[1].x);
                Result = q0 < q1 ? Positions[0] : Positions[1];
            }
            else{
                Result = Positions[0];
            }
        }else{
            Result = Positions[0];
        }

        if(Result.z < 30){
            yaw_out = 0;
            pitch_out = 0;
        }else{
            AngleFit(Result);
        }
        cout << "z" << Result.z  << "x" << Result.x << "y:" << Result.y << endl;
        cout << "pitch:" << pitch_out << "yaw_out:" << yaw_out << endl;
        cout << endl << endl << endl << endl;
        Vision = {yaw_out,pitch_out,Result.z,0,1};
        OldResult = Result;
    }
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
    yaw_out = atan(input.x/input.z) * 180/3.14159265;
    pitch_out = atan((input.y - gravity_offset)/input.z) * 180/3.14159265;
}
