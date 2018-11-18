#include "RM_v4l2.h"


RM_v4l2::RM_v4l2()
{

}


RM_v4l2::RM_v4l2(const char* filename, int exposures, int saturations)
{
    // 设置设备名、曝光、饱和度
    file_name = filename;
    exposure = exposures;
    saturation = saturations;


    // 打开设备
    equipment = open(file_name,O_RDWR);
    if (equipment == -1)
    {
        cout << "没有插相机" << endl;
        return;
    }
    // 曝光模式
    struct v4l2_control EXPOSURE_AUTO;
    EXPOSURE_AUTO.id = V4L2_CID_EXPOSURE_AUTO;
    EXPOSURE_AUTO.value = 1;
    ioctl(equipment,VIDIOC_S_CTRL,&EXPOSURE_AUTO);
    // 设置曝光值
    struct v4l2_control EXPOSURE_ABSOLUTE;
    EXPOSURE_ABSOLUTE.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    EXPOSURE_ABSOLUTE.value = exposure;
    if(ioctl(equipment,VIDIOC_S_CTRL,&EXPOSURE_ABSOLUTE))
    {
        cout << "can't set the exposures";
        return;
    }
    // 设置饱和度
    struct v4l2_control SATURATION;
    SATURATION.id = V4L2_CID_SATURATION;
    SATURATION.value = saturation;
    if(ioctl(equipment,VIDIOC_S_CTRL,&SATURATION) == -1)
    {
        cout << "can't set the saturation";
        return ;
    }
    cout << "相机打开并且设置完成！" << endl;
}


void RM_v4l2::v4l2_exposure(int exposures)
{
    // 更改曝光
    struct v4l2_control exposure_s;
    exposure_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    exposure_s.value = exposures;
    exposure = exposures;
    if (ioctl(equipment,VIDIOC_S_CTRL,&exposure_s))
    {
        cout << "设置不了曝光" << endl;
    }
}


void RM_v4l2::Get_v4l2_exposure()
{
    std::cout << "当前的曝光为:"<<exposure << std::endl;
}


void RM_v4l2::v4l2_saturation(int saturations)
{
    // 更改饱和度
    struct v4l2_control saturation_s;
    saturation_s.id = V4L2_CID_SATURATION;
    saturation_s.value = saturations;
    saturation = saturations;
    if(ioctl(equipment,VIDIOC_S_CTRL,&saturation_s))
    {
        cout << "设置不了饱和度" << endl;
    }
}


void RM_v4l2::Get_v4l2_saturation()
{
    std::cout << saturation << std::endl;
    cout << "当前的饱和度为:" << saturation << endl;
}


RM_v4l2::~RM_v4l2()
{
    close(equipment);
}
