#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <QTime>
#include <QDebug>
#include "RM_v4l2.h"
#include "serialport.h"
#include "solvepnp.h"
#include "sudoku.h"
#include "color_detect.h"

using namespace cv;
using namespace std;


int main()
{
    /************* 摄像头设置××××××××××××××*/
    RM_v4l2 v4l2("/dev/video1",20,100);
    VideoCapture camera(1);
    camera.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    cout << "当前的分辨率为" << "1280*720" << endl;
    int fps = camera.get(CV_CAP_PROP_FPS);
    cout << "当前的帧率是：" << fps << endl;
    /************   串口设置******************/
    SerialPort port;
    port.initSerialPort();
    VisionData vision = {0,0,0,0,0};
    /**************解pnp参数设置****************/

    Mat camera_matrix = (Mat_<double>(3,3) << 468.4123, 0, 309.6381,0, 472.1558, 240.7268,0, 0, 1 );
    Mat dist_coeff = (Mat_<double>(5,1) << -0.4502, 0.33525, 0.002628, 0.0039192, -0.1998109);
    double target_width = 13.5;
    double target_height = 12.5;
    AngleSolver angle(camera_matrix,dist_coeff,target_width,target_height);
    double angle_x = 0;
    double angle_y = 0;
    RotatedRect last_real_ammor;
    /***************主函数******************/
    color_detect Color;
    sudoku symbol;
    while(camera.isOpened())
    {
        QTime time;
        time.start(); // 计时间

        // 加载图片
        Mat frame;
        camera >> frame;
        if(frame.empty())
        {
            cout << "没有图片" << endl;
            return -1;
        }

        //得到电控的指令并发送
        int mode = 0x02;
        port.get_Mode(mode);
        switch (mode)
        {
            case(0x00):
                vision = {(float)angle_x,(float)angle_y,0,0,0};
                break;

            case(0x01):
                Color.detect(frame,'R');
                last_real_ammor = Color.final_ammor;
                angle.getAngle(last_real_ammor,angle_x,angle_y,0);
                vision = {(float)angle_x,(float)angle_y,0,0,1};
                break;
            case(0x02):
                Color.detect(frame,'B');
                last_real_ammor = Color.final_ammor;
                angle.getAngle(last_real_ammor,angle_x,angle_y,0);
                vision = {(float)angle_x,(float)angle_y,0,0,1};
                break;
            case(0x03):
                symbol.detect(frame);
                break;
            case(0x04):break;
        }
        port.TransformData(vision);
        port.send();


        cout << "angle_x:" << angle_x << ",angle_y:" << angle_y << endl;
        imshow("原图",frame);
        waitKey(1);

        qDebug()<<"time:"<<time.elapsed()<<"ms";
    }

    return 0;
}

