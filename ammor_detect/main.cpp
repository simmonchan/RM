#include <QTime>
#include <QDebug>
#include "camera_calibration.h"
#include "serialport.h"
#include "solvepnp.h"
#include "ammor_find.h"
#include "stereo_vision.h"
#include "RMVideoCapture.h"
#include "Header.h"
#include "Imagethread.h"
#include <thread>

using namespace cv;
using namespace std;

//#define MONOCALUR
//#define IMAGE_DEBUG
//#define PORT_SEND

static inline bool distance_sort(const Armordata & L1, const Armordata & L2){
    return L1.distance < L2.distance;
}

// get the image thread
void ProImageThread(RMVideoCapture &cap, Mat &src)
{
    cap >> src;
}


int main()
{
#ifdef PORT_SEND
    //set the port
    SerialPort port;
    port.initSerialPort();
    VisionData vision = {0,0,0,0,0};
#endif

    // open the camera and local the camera
    RMVideoCapture cap0("/dev/video1",3);
    if(cap0.fd!=-1){
        cap0.setVideoFormat(1280,720,1);
        cap0.setExposureTime(0, 10);
        cap0.startStream();
        cap0.info();
    }
    else{
        cap0.camnum = -1;
        cout << "打开相机失败" << endl;
        return 0;
    }

    RMVideoCapture cap1("/dev/video2",3);
    if(cap1.fd!=-1){
        cap1.setVideoFormat(1280,720,1);
        cap1.setExposureTime(0,10);
        cap1.startStream();
        cap1.info();
    }
    else{
        cap1.camnum = -1;
        cout << "打开相机失败" << endl;
        return 0;
    }
    RMVideoCapture cap_left,cap_right;
    bool camstatus[2] = {false,false};
    if(cap0.camnum == 1)
    {
        cap_left = cap0;
        camstatus[0] = true;
    }
    else if(cap1.camnum == 1)
    {
        cap_left = cap1;
        camstatus[0] = true;
    }
    else{
        camstatus[0] = false;
    }

    if(cap0.camnum == 2)
    {
        cap_right = cap0;
        camstatus[1] = true;
    }
    else if(cap1.camnum == 2)
    {
        cap_right = cap1;
        camstatus[1] = true;
    }
    else{
        camstatus[1] = false;
    }

    // define the using things of detect
    Ammor_find Armor_find_left;
    Ammor_find Armor_find_right;

    Mat frame_left;
    Mat frame_right;

    vector<Point> Left_points;
    vector<Point> Right_points;

    vector<Armordata> Left_armor_data;
    vector<Armordata> Right_armor_data;

    bool Left_flag = false;
    bool Right_flag = false;

    Mat cameraMatrixL;
    Mat distCoeffL;
    Mat cameraMatrixR;
    Mat distCoeffR;

    double target_width = 13.5;
    double target_height = 6.5;

    FileStorage camera_xml("camera_calibrate.yaml",FileStorage::READ);
    camera_xml["cameraMatrixL"] >> cameraMatrixL;
    camera_xml["distCoeffL"] >> distCoeffL;
    camera_xml["cameraMatrixR"] >> cameraMatrixR;
    camera_xml["distCoeffR"] >> distCoeffR;
    camera_xml.release();

    AngleSolver Left_Solver(cameraMatrixL,distCoeffL,target_width,target_height);
    AngleSolver Right_Solver(cameraMatrixR,distCoeffR,target_width,target_height);
    double angle_x = 0.0, angle_y = 0.0;

    stereo_vision stereo;

    cout << "自瞄开始" << endl;
    while(camstatus[0] && camstatus[1])
    {
        QTime time;
        time.start();

        thread LeftImagePro(ProImageThread,ref(cap_left),ref(frame_left));
        thread RightImagePro(ProImageThread,ref(cap_right),ref(frame_right));

        LeftImagePro.join();
        RightImagePro.join();
#ifdef IMAGE_DEBUG
        imshow("left",frame_left);
        imshow("right",frame_right);
        waitKey(1);
#endif
        thread LeftImageCon(&Ammor_find::detect,Armor_find_left,ref(frame_left),BLUE_DETECT,ref(Left_armor_data),ref(Left_points),ref(Left_flag));
        thread RightImageCon(&Ammor_find::detect,Armor_find_right,ref(frame_right),BLUE_DETECT,ref(Right_armor_data),ref(Right_points),ref(Right_flag));

        LeftImageCon.join();
        RightImageCon.join();

        if(Left_flag && Right_flag){

            if(Left_points.size() == Right_points.size()){

                // sort the distance
                stereo.stereo_get_distance(Left_points,Right_points,Left_armor_data,Right_armor_data);

                sort(Left_armor_data.begin(), Left_armor_data.end(),distance_sort);
                sort(Right_armor_data.begin(), Right_armor_data.end(),distance_sort);

                //memory the last data
                Armor_find_left._LastArmor = Left_armor_data[0];
                Armor_find_right._LastArmor = Right_armor_data[0];
                Armor_find_left._flag = true;
                Armor_find_right._flag = true;
#ifdef PORT_SEND
                // send the data
                Left_Solver.getAngle(Left_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
                vision = {(float)angle_x,(float)angle_y,Left_armor_data[0].distance,0,1};
                port.TransformData(vision);
                port.send();
#endif
                Armor_find_left.clear();
                Armor_find_right.clear();
            }

            else if(Left_points.size() > Right_points.size())
            {
                // sort the distance
                Left_Solver.get_distance(Left_armor_data,Point(0,0));
                sort(Left_armor_data.begin(), Left_armor_data.end(),distance_sort);

                Right_Solver.get_distance(Right_armor_data,Point(0,0));
                sort(Right_armor_data.begin(),Right_armor_data.end(),distance_sort);

                // memory the last data
                Armor_find_left._LastArmor = Left_armor_data[0];
                Armor_find_right._LastArmor = Right_armor_data[0];
                Armor_find_left._flag = true;
                Armor_find_right._flag = true;
#ifdef PORT_SEND
                // send the data
                Left_Solver.getAngle(Left_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
                vision = {(float)angle_x,(float)angle_y,Left_armor_data[0].distance,0,1};
                port.TransformData(vision);
                port.send();
#endif
                Armor_find_left.clear();
                Armor_find_right.clear();
            }

            else{
                // sort the distance
                Left_Solver.get_distance(Left_armor_data,Point(0,0));
                sort(Left_armor_data.begin(), Left_armor_data.end(),distance_sort);

                Right_Solver.get_distance(Right_armor_data,Point(0,0));
                sort(Right_armor_data.begin(),Right_armor_data.end(),distance_sort);

                // memory the last data
                Armor_find_left._LastArmor = Left_armor_data[0];
                Armor_find_right._LastArmor = Right_armor_data[0];
                Armor_find_left._flag = true;
                Armor_find_right._flag = true;

#ifdef PORT_SEND
                // send the data
                Right_Solver.getAngle(Right_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
                vision = {(float)angle_x,(float)angle_y,Right_armor_data[0].distance,0,1};
                port.TransformData(vision);
                port.send();
#endif
                Armor_find_left.clear();
                Armor_find_right.clear();
            }
        }
        else if(Armor_find_left._flag){
            Left_Solver.get_distance(Left_armor_data,Point(0,0));
            sort(Left_armor_data.begin(), Left_armor_data.end(),distance_sort);

            // memory the last data
            Armor_find_left._flag = false;
            Armor_find_right._flag = false;
#ifdef PORT_SEND
            // send the data
            Left_Solver.getAngle(Left_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
            vision = {(float)angle_x,(float)angle_y,Left_armor_data[0].distance,0,1};
            port.TransformData(vision);
            port.send();
#endif
            Armor_find_left.clear();
            Armor_find_right.clear();
        }
        else if(Armor_find_right._flag){
            Right_Solver.get_distance(Right_armor_data,Point(0,0));
            sort(Right_armor_data.begin(), Right_armor_data.end(),distance_sort);

            // memory the last data
            Armor_find_left._flag = false;
            Armor_find_right._flag = false;
#ifdef PORT_SEND
            // send the data
            Right_Solver.getAngle(Right_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
            vision = {(float)angle_x,(float)angle_y,Right_armor_data[0].distance,0,1};
            port.TransformData(vision);
            port.send();
#endif
            Armor_find_left.clear();
            Armor_find_right.clear();
        }
        else
        {
#ifdef PORT_SEND
            vision = {0,0,0,0,0};
            port.TransformData(vision);
            port.send();
#endif
            Armor_find_left.clear();
            Armor_find_right.clear();
        }
        qDebug()<<"time:"<<time.elapsed()<<"ms";
#ifdef IMAGE_DEBUG
        char k = waitKey(1);
        if(k == 'q')
        {
            cap_left.closeStream();
            cap_right.closeStream();
            exit(1);
        }
#endif
    }
    cap0.closeStream();
    cap1.closeStream();
    return 0;
}

#ifdef MONOCALUR
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
#endif
