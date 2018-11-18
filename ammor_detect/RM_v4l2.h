#ifndef RM_V4L2_H
#define RM_V4L2_H
#include <linux/videodev2.h>
#include <fcntl.h>//open
#include <unistd.h>//close
#include <sys/ioctl.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;
/*这个类用来设置摄像头的曝光，饱和度等参数*/

class RM_v4l2
{
public:
    RM_v4l2();
    RM_v4l2(const char* filename, int exposures, int saturations);
    // 设置曝光
    void v4l2_exposure(int exposures);
    // 设置饱和度
    void v4l2_saturation(int saturations);
    // 得到曝光
    void Get_v4l2_saturation();
    // 得到饱和度
    void Get_v4l2_exposure();
    ~RM_v4l2();

private:
    const char * file_name;     // 摄像头的路径
    int exposure;     // 曝光
    int saturation;   // 饱和度
    int equipment;    // 打开设备名字
};

#endif // RM_V4L2_H
