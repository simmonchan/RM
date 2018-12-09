#include "include/ArmorFind/ammor_find.h"
#include "include/Camera/camera_calibration.h"
#include "include/Camera/RMVideoCapture.h"
#include "include/Serial/CRC_Check.h"
#include "include/Serial/serialport.h"
#include "include/Stereo_vision/solvepnp.h"
#include "include/Stereo_vision/stereo_vision.h"
#include "include/Header.h"

using namespace cv;
using namespace std;

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
    RMVideoCapture cap0("/dev/video0",3);
    if(cap0.fd!=-1){
        cap0.setVideoFormat(1280,720,1);
        cap0.setExposureTime(0, 7);
        cap0.startStream();
        cap0.info();
    }
    else{
        cap0.camnum = -1;
        cout << "打开相机失败" << endl;
        return 0;
    }

    RMVideoCapture cap1("/dev/video1",3);
    if(cap1.fd!=-1){
        cap1.setVideoFormat(1280,720,1);
        cap1.setExposureTime(0,7);
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

    vector<Point2f> Left_points;
    vector<Point2f> Right_points;

    vector<Armordata> Left_armor_data;
    vector<Armordata> Right_armor_data;

    bool Left_flag = false;
    bool Right_flag = false;

    Mat cameraMatrixL;
    Mat distCoeffL;
    Mat cameraMatrixR;
    Mat distCoeffR;

    double target_width = 130;
    double target_height = 55;

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
                //Left_Solver.getAngle(Left_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
                Point point = Left_armor_data[0].armor_center;
                if(point.x < 660 && point.x > 620  && point.y > 320 && point.y < 400){
                    vision = {(float)point.x,(float)point.y,Left_armor_data[0].distance,0,2};
                }
                else{
                    vision = {(float)point.x,(float)point.y,Left_armor_data[0].distance,0,1};
                }
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

//            else if(Left_points.size() < Right_points.size()){
//                // sort the distance
//                Left_Solver.get_distance(Left_armor_data,Point(0,0));
//                sort(Left_armor_data.begin(), Left_armor_data.end(),distance_sort);

//                Right_Solver.get_distance(Right_armor_data,Point(0,0));
//                sort(Right_armor_data.begin(),Right_armor_data.end(),distance_sort);

//                // memory the last data
//                Armor_find_left._LastArmor = Left_armor_data[0];
//                Armor_find_right._LastArmor = Right_armor_data[0];
//                Armor_find_left._flag = true;
//                Armor_find_right._flag = true;

//#ifdef PORT_SEND
//                // send the data
//                Right_Solver.getAngle(Right_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
//                vision = {(float)angle_x,(float)angle_y,Right_armor_data[0].distance,0,1};
//                cout << "error" << endl;
//                port.TransformData(vision);
//                port.send();
//#endif
//                Armor_find_left.clear();
//                Armor_find_right.clear();

//            }
        }
        else if(Left_flag){
            Left_Solver.get_distance(Left_armor_data,Point(0,0));
            sort(Left_armor_data.begin(), Left_armor_data.end(),distance_sort);
            cout << "anglex:" << angle_x << "   angle_y:" << angle_y << endl;
            cout << "distance:" << Left_armor_data[0].distance << endl;
            // memory the last data
            Armor_find_left._flag = true;
            Armor_find_right._flag = false;
#ifdef PORT_SEND
            // send the data
            //Left_Solver.getAngle(Left_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
            Point point = Left_armor_data[0].armor_center;
            vision = {(float)point.x,(float)point.y,Left_armor_data[0].distance,0,1};
            port.TransformData(vision);
            port.send();
#endif
            Armor_find_left.clear();
            Armor_find_right.clear();
        }
//        else if(Right_flag){
//            Right_Solver.get_distance(Right_armor_data,Point(0,0));
//            sort(Right_armor_data.begin(), Right_armor_data.end(),distance_sort);
//            cout << "anglex:" << angle_x << "   angle_y:" << angle_y << endl;
//            // memory the last data
//            Armor_find_left._flag = false;
//            Armor_find_right._flag = true;
//            cout << "right_distance" << Right_armor_data[0].distance << endl;
//#ifdef PORT_SEND
//            // send the data
//            //Right_Solver.getAngle(Right_armor_data[0],angle_x,angle_y,0.0,0.0,Point(0,0));
//            Point point = Right_armor_data[0].armor_center;
//            vision = {(float)point.x,(float)point.y,Right_armor_data[0].distance,0,1};
//            //vision = {(float)angle_x,(float)angle_y,Right_armor_data[0].distance,0,1};
//            port.TransformData(vision);
//            port.send();
//#endif
//            Armor_find_left.clear();
//            Armor_find_right.clear();
//        }
        else
        {
            Armor_find_left._flag = false;
            Armor_find_right._flag = false;
            Armor_find_left.clear();
            Armor_find_right.clear();
        }
#ifdef IMAGE_DEBUG
        char k = waitKey(1);
        if(k == 'q')
        {
            cap_left.closeStream();
            cap_right.closeStream();
            exit(1);
        }
#endif
        qDebug()<<"time:"<<time.elapsed()<<"ms";
    }
    cap0.closeStream();
    cap1.closeStream();
    return 0;
}
