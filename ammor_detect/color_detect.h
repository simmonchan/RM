#ifndef COLOR_DETECT_H
#define COLOR_DETECT_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class color_detect
{
public:
    color_detect(){
        mode_color = 'e';
        flag = false;
        pre_value[0] = 0;
        pre_value[1] = 0;
        pre_value[2] = 0;
        pre_value[3] = 0;
    }
    void img_cut();
    void blue_detect();
    void red_detect();
    void Rotated_detect(vector<RotatedRect> &Led_rRect);
    void ammor_detect(vector<RotatedRect> &Led_rRect);
    void detect(Mat &image,char color_mode);
    RotatedRect get_final_ammor();
    ~color_detect();

public:
    RotatedRect final_ammor;
    double pre_value[4];
    bool flag;
private:
    Mat src;
    Mat src_color;
    char mode_color;


};

#endif // COLOR_DETECT_H
