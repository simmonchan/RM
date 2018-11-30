#include <QTime>
#include <QDebug>
#include "camera_calibration.h"
#include "RM_v4l2.h"
#include "serialport.h"
#include "solvepnp.h"
#include "sudoku.h"
#include "color_detect.h"
#include "two_camera.h"
#include "ammor_find.h"
#include "Header.h"

//#define USE_PNP

using namespace cv;
using namespace std;


int main()
{
    /************* 摄像头设置××××××××××××××*/
    RM_v4l2 v4l2("/dev/video0",10,100);
    VideoCapture camera(0);
    camera.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    cout << "当前的分辨率为" << "1280*720" << endl;
    int fps = camera.get(CV_CAP_PROP_FPS);
    cout << "当前的帧率是：" << fps << endl;
    /************   串口设置******************/
    //SerialPort port;
    //port.initSerialPort();
    VisionData vision = {0,0,0,0,0};
    /**************解pnp参数设置****************/

    Mat camera_matrix = (Mat_<double>(3,3) << 468.4123, 0, 309.6381,0, 472.1558, 240.7268,0, 0, 1 );
    Mat dist_coeff = (Mat_<double>(5,1) << -0.4502, 0.33525, 0.002628, 0.0039192, -0.1998109);
    double target_width = 13.5;
    double target_height = 6.5;
    AngleSolver angle(camera_matrix,dist_coeff,target_width,target_height);
    double angle_x = 0;
    double angle_y = 0;
    RotatedRect last_real_ammor;
    /***************主函数******************/
    Ammor_find Color;
    vector<Point> ArmorPoints;
    while(camera.isOpened())
    {
        QTime time;
        time.start(); // 计时间
        Mat frame;
        camera >> frame;
        if(frame.empty())
        {
            cout << "没有图片" << endl;
            continue;
        }
        int mode = GET_BLUE;
        //port.get_Mode(mode);
        switch (mode)
        {
            case(GET_NO):
                vision = {0,0,0,0,0};
                break;

            case(GET_RED):
            Color.detect(frame,RED_DETECT);
#ifdef USE_PNP
                last_real_ammor = Color.final_ammor;
                angle.getAngle(last_real_ammor,angle_x,angle_y,0);
                vision = {(float)angle_x,(float)angle_y,0,0,1};
#endif
            ArmorPoints = Color._ArmorPoints;
            Color.clear();
            vision = {(float)ArmorPoints[0].x,(float)ArmorPoints[0].y,0,0,1};
            break;

            case(GET_BLUE):
            Color.detect(frame,BLUE_DETECT);
#ifdef USE_PNP
                last_real_ammor = Color.final_ammor;
                angle.getAngle(last_real_ammor,angle_x,angle_y,0);
                vision = {(float)angle_x,(float)angle_y,0,0,1};
#endif
            ArmorPoints = Color._ArmorPoints;
            Color.clear();
            vision = {(float)ArmorPoints[0].x,(float)ArmorPoints[0].y,0,0,1};
            break;

            default:break;
        }
        //port.TransformData(vision);
        //port.send();

        imshow("原图",frame);
        waitKey(1);
        qDebug()<<"time:"<<time.elapsed()<<"ms";
    }
    return 0;
}
