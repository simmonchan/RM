#ifndef HEADER_H
#define HEADER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <iostream>

#define BLUE_DETECT true
#define RED_DETECT false

#define GET_BLUE 0x02
#define GET_RED 0x01
#define GET_NO 0x00

using namespace cv;
using namespace std;

typedef struct{
    typedef enum{
        small_armor,
        big_armor
    }armor_type;
    Point armor_center;
    float distance;//mm
    armor_type armor;
    Point sort_points[4];
    int Last_xids;
}Armordata;

#endif // HEADER_H
