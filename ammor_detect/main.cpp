#include <QTime>
#include <QDebug>
#include "camera_calibration.h"
#include "RM_v4l2.h"
#include "serialport.h"
#include "solvepnp.h"
#include "color_detect.h"
#include "ammor_find.h"
#include "stereo_vision.h"
#include "Header.h"

using namespace cv;
using namespace std;

int detects()
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
    SerialPort port;
    port.initSerialPort();
    VisionData vision = {0,0,0,0,0};
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
            ArmorPoints = Color._ArmorPoints;
            Color.clear();
            if(ArmorPoints.size() !=0)
            {
                vision = {(float)ArmorPoints[0].x,(float)ArmorPoints[0].y,0,0,1};
            }
            break;

            case(GET_BLUE):
            Color.detect(frame,BLUE_DETECT);
            ArmorPoints = Color._ArmorPoints;
            Color.clear();
            if(ArmorPoints.size() !=0)
            {
                vision = {(float)ArmorPoints[0].x,(float)ArmorPoints[0].y,0,0,1};
            }
            break;

            default:break;
        }
        port.TransformData(vision);
        port.send();

        imshow("原图",frame);
        waitKey(1);
        qDebug()<<"time:"<<time.elapsed()<<"ms";
    }
    return 0;
}


int stereo_detect()
{
    SerialPort port;
    port.initSerialPort();
    VisionData vision = {0,0,0,0,0};

    RM_v4l2 camera_left("/dev/video0",10,100);
    RM_v4l2 camero_right("/dev/video1",10,100);

    VideoCapture cap_left(0);
    VideoCapture cap_right(1);

    cap_left.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap_left.set(CV_CAP_PROP_FRAME_HEIGHT,720);

    cap_right.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap_right.set(CV_CAP_PROP_FRAME_HEIGHT,720);

    Ammor_find left;
    Ammor_find right;

    vector<Point> left_points;
    vector<Point> right_points;

    vector<Armordata> left_armor_data;
    vector<Armordata> right_armor_data;

    stereo_vision stereo;

    while(cap_left.isOpened() || cap_right.isOpened())
    {
        QTime time;
        time.start();

        Mat frame_left;
        Mat frame_right;

        cap_left >> frame_left;
        cap_right >> frame_right;

        if(frame_left.empty() && frame_right.empty())
        {
            cout << "no picture" << endl;
            continue;
        }

        left.detect(frame_left,BLUE_DETECT);
        right.detect(frame_right,BLUE_DETECT);

        left_points = left._ArmorPoints;
        right_points = right._ArmorPoints;

        left_armor_data = left._Armordatas;
        right_armor_data = right._Armordatas;

        left.clear();
        right.clear();

        if(left_points.size() > 0 || right_points.size() > 0){
            if(left_points.size() == right_points.size()){
                stereo.stereo_get_distance(left_points,right_points,left_armor_data);
                sort(left_armor_data.begin(), left_armor_data.end(),
                     [](const Armordata & L1, const Armordata & L2) { return L1.distance < L2.distance;});
                Point final = (left_armor_data[0].armor_center);
                vision = {(float)final.x,(float)final.y,left_armor_data[0].distance,0,1};
            }
//            else if(left_points.size() > right_points.size()){
//                stereo.monocular_get_distance(left_armor_data,LEFT,Point(0,0));
//                sort(left_armor_data.begin(), left_armor_data.end(),
//                     [](const Armordata & L1, const Armordata & L2) { return L1.distance < L2.distance;});
//                Point final = (left_armor_data[0].armor_center);
//                vision = {(float)final.x,(float)final.y,left_armor_data[0].distance,0,1};
//            }
//            else{
//                stereo.monocular_get_distance(right_armor_data,RIGHT,Point(0,0));
//                sort(right_armor_data.begin(), right_armor_data.end(),
//                     [](const Armordata & L1, const Armordata & L2) { return L1.distance < L2.distance;});
//                Point final = (left_armor_data[0].armor_center);
//                vision = {(float)final.x,(float)final.y,left_armor_data[0].distance,0,1};
//            }

           port.TransformData(vision);
           port.send();
        }
        else
        {
            vision = {0,0,0,0,0};
            port.TransformData(vision);
            port.send();
        }
    }
    return 0;
}

int main()
{
    camera_two_calibration();
}
