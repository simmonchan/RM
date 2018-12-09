#ifndef HEADER_H
#define HEADER_H

#include <QTime>
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <iostream>
#include <thread>

//#define SHOW_DEBUG
#define IMAGE_DEBUG
#define PORT_SEND

#define BLUE_DETECT true
#define RED_DETECT false

#define GET_BLUE 0x02
#define GET_RED 0x01
#define GET_NO 0x00

#define LEFT true
#define RIGHT false

using namespace cv;
using namespace std;



struct InitParams{
    uchar armor_thres_whole;
    uchar armor_thres_red;
    uchar armor_thres_blue;

    InitParams(){
        armor_thres_whole = 30;
        armor_thres_red = 40;
        armor_thres_blue = 80;
    }

};

typedef struct{
    typedef enum{
        small_armor,
        big_armor
    }armor_type;
    Point armor_center = Point(0,0);
    float distance = 0.0;//mm
    Point2f armor_points[4] = {Point(0,0),Point(0,0),Point(0,0),Point(0,0)};
    armor_type armor = small_armor;
}Armordata;

#endif // HEADER_H
