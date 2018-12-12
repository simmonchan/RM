#include "include/ArmorFind/ammor_find.h"
#include "include/ArmorFind/armorpredict.h"
#include "include/Camera/camera_calibration.h"
#include "include/Camera/RMVideoCapture.h"
#include "include/Serial/CRC_Check.h"
#include "include/Serial/serialport.h"
#include "include/Stereo_vision/solvepnp.h"
#include "include/Stereo_vision/stereo_vision.h"
#include "include/Header.h"

using namespace cv;
using namespace std;

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
#endif

    // open the camera and local the camera
    RMVideoCapture cap0("/dev/video0",3);
    if(cap0.fd!=-1){
        cap0.setVideoFormat(1280,720,1);
        cap0.setExposureTime(0, 10);
        cap0.startStream();
        cap0.info();
    }
    else{
        cap0.camnum = -1;
        cout << "can't open the camera" << endl;
        return 0;
    }

    RMVideoCapture cap1("/dev/video1",3);
    if(cap1.fd!=-1){
        cap1.setVideoFormat(1280,720,1);
        cap1.setExposureTime(0,10);
        cap1.startStream();
        cap1.info();
    }
    else{
        cap1.camnum = -1;
        cout << "can't open the camera" << endl;
        return 0;
    }
    RMVideoCapture cap_left,cap_right;
    bool camstatus[2] = {false,false};
    if(cap0.camnum == 1){
        cap_left = cap0;
        camstatus[0] = true;
    }
    else if(cap1.camnum == 1){
        cap_left = cap1;
        camstatus[0] = true;
    }
    else{
        camstatus[0] = false;
    }

    if(cap0.camnum == 2){
        cap_right = cap0;
        camstatus[1] = true;
    }
    else if(cap1.camnum == 2){
        cap_right = cap1;
        camstatus[1] = true;
    }
    else{
        camstatus[1] = false;
    }

    cout << "摄像机打开完成" << endl;

    // define the using things of detect
    Ammor_find Armor_find_left,Armor_find_right;
    Mat frame_left,frame_right;
    vector<Point2f> Left_points,Right_points;
    vector<Armordata> Left_armor_data, Right_armor_data;
    uchar Left_flag = 1, Right_flag = 2;

    ArmorPredict cal_angle;

    while(camstatus[0] && camstatus[1])
    {
        QTime time;
        time.start();

        thread LeftImagePro(ProImageThread,ref(cap_left),ref(frame_left));
        thread RightImagePro(ProImageThread,ref(cap_right),ref(frame_right));
        LeftImagePro.join();
        RightImagePro.join();

#ifdef IMAGE_DEBUG
        imshow("left_frame",frame_left);
        imshow("right_frmae",frame_right);
#endif

        thread LeftImageCon(&Ammor_find::detect,&Armor_find_left,ref(frame_left),BLUE_DETECT,Left_flag);
        thread RightImageCon(&Ammor_find::detect,&Armor_find_right,ref(frame_right),BLUE_DETECT,Right_flag);
        LeftImageCon.join();
        RightImageCon.join();

        Left_points = Armor_find_left._ArmorPoints;
        Right_points = Armor_find_right._ArmorPoints;
        Left_armor_data = Armor_find_left._Armordatas;
        Right_armor_data = Armor_find_right._Armordatas;

        cal_angle.Predict(Left_points,Right_points,Left_armor_data,Right_armor_data);

        Armor_find_left._LastArmor = Left_armor_data[cal_angle.Result.index];
        Armor_find_right._LastArmor = Right_armor_data[cal_angle.Result.index];

#ifdef PORT_SEND
        port.TransformData(cal_angle.Vision);
        port.send();
#endif


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
