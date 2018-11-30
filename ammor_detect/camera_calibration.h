#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

/* 图片的数目传进*/
void camera_calibration(int image_num,Size chessboard_one);

void camera_two_calibration();

#endif // CAMERA_CALIBRATION_H
