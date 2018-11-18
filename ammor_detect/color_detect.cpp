#include "color_detect.h"

void color_detect::img_cut()
{
    // 返回上一帧左上坐标
    int top = pre_value[1] - pre_value[3]/2;
    if (top < 0)
    {
        top = 1;
    }

    int bottom = pre_value[1] + pre_value[3]/2;
    if (bottom > 720)
    {
        bottom = 719;
    }

    int left = pre_value[0] - pre_value[2]/2;
    if (left < 0)
    {
        left = 1;
    }

    int right = pre_value[0] + pre_value[2]/2;
    if (right > 1280)
    {
        right = 1279;
    }
    if(left > right || top > bottom)
    {
        top = 1;
        bottom = 719;
        left = 1;
        right = 1279;
    }
    src = src(Range(top,bottom),Range(left,right));
}

void color_detect::blue_detect()
{
    // 转为HSV空间
    Mat src_HSV;
    cvtColor(src, src_HSV, COLOR_BGR2HSV);
    // 调节亮度，减小边缘的影响
    vector<Mat> planes;
    split(src_HSV, planes);
    planes[2] -= 50;
    merge(planes,src_HSV);

    // 蓝色识别
    const Scalar hsvBlueLow(100,43, 46);
    const Scalar hsvBlueHigh(150,255,255);
    inRange(src_HSV, hsvBlueLow, hsvBlueHigh, src_color);
    // 腐蚀，使连通
    Mat s = getStructuringElement(MORPH_RECT, Size(3,3));
    dilate(src_color,src_color,s);
}

void color_detect::red_detect()
{
    // 转为HSV空间
    Mat src_HSV;
    cvtColor(src , src_HSV, COLOR_BGR2HSV);
    // 调节亮度
    vector<Mat> planes;
    split(src_HSV, planes);
    planes[2] -= 30;
    merge(planes,src_HSV);
    // 红色检测
    Mat dst;
    dst.create(src_HSV.size(),CV_8UC1);
    for (int r = 0; r < src_HSV.rows; r++)
    {
        Vec3b * ptr = src_HSV.ptr<Vec3b>(r);
        uchar * ptr_dst = dst.ptr<uchar>(r);


        for (int c = 0; c < src_HSV.cols; c++)
        {
            if ((((ptr[c][0] <= 8)) || (ptr[c][0] >= 156))
                && (ptr[c][1] >= 43)
                && (ptr[c][2] >= 46))
            {
                 ptr_dst[c] = 255;
            }
            else
            {
                 ptr_dst[c] = 0;
            }
        }
     }

    // 使连通
    Mat s = getStructuringElement(MORPH_RECT, Size(3,3));
    dilate(dst,dst,s);
    // 使下一张能够使用
    src_color = dst.clone();
}

void color_detect::Rotated_detect(vector<RotatedRect> &Led_rRect)
{
    // 找出轮廓
    vector<vector<Point> > contours;
    findContours(src_color,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    // 拟合为旋转矩阵
    for (int i=0;i<(int)contours.size();i++)
    {
        // 条件1：点的集合
        if ((int)contours[i].size() > 10)
        {
            RotatedRect rRect =  minAreaRect(contours[i]);
            // 条件2：筛选掉横的
           if(((rRect.size.width >= rRect.size.height+5) && (rRect.angle < -60)) ||
                   ((rRect.size.width+5 <= rRect.size.height) && (rRect.angle > -30)))
           {
               // 筛选掉小的
               if (rRect.size.area() > 100)
               {
                   // 把检测到的蓝色用框画起来
                   Point2f points[4];
                   rRect.points(points);
                   for (int i=0;i<4;i++)
                   {
                       int j = (i+1)%4;
                       line (src,points[i],points[j],Scalar(255,0,255),2);
                   }
                   // 放入灯柱的向量
                   Led_rRect.push_back(rRect);
               }
           }
        }
    }
}

void color_detect::ammor_detect(vector<RotatedRect> &led_rRect)
{
    struct ammor_scores
    {
        double score;
        int left;
        int right;
    }localtion;
    vector<ammor_scores> scores;
    // 如果没有找到2个，就返回
    if(led_rRect.size() < 2)
    {   for (int i=0;i<4;i++)
        {
            pre_value[i] = 0;
        }
        final_ammor.center = Point(0,0);
        final_ammor.size = Size(0,0);
        flag = false;
    }
    else
    {
        // 逐个检测
        for(int i=0;i < (int)led_rRect.size()-1;i++)
        {
            for (int j=i+1;j < (int)led_rRect.size();j++)
            {
                // 计算周长的差
                double C_i = led_rRect[i].size.height+led_rRect[i].size.width;
                double C_j = led_rRect[j].size.height+led_rRect[j].size.width;
                double C = abs(C_i - C_j);

                //计算距离
                double dcenter_x = abs(led_rRect[i].center.x - led_rRect[j].center.x);
                double dcenter_y = abs(led_rRect[i].center.y - led_rRect[j].center.y);
                double distance = sqrt(pow(dcenter_x,2)+pow(dcenter_y,2));
                // 打分
                double dscore = 0.7*distance + 0.3*C;

                // 利用坐标信息，长度信息
                if(((dcenter_x > 20) &&
                    (dcenter_x < 500)) &&
                        (dcenter_y < 20))
                {
                    if (C <= 13)
                    {
                        // 记录这个分数
                        localtion.score = dscore;
                        localtion.left = i;
                        localtion.right = j;
                        scores.push_back(localtion);
                    }
                }
            }
        }
    }
    // 如果分数向量没有，就返回
    if (!scores.size())
    {
        for(int i=0;i<4;i++)
        {
            pre_value[i] = 0;
        }
        final_ammor.center = Point(0,0);
        final_ammor.size = Size(0,0);
        flag = false;
    }
    // 找出分数最小的
    else
    {
        double min = scores[0].score;

        int index = 0;
        for (int i=1;i<(int)scores.size();i++)
        {
            if (scores[i].score < min)
            {
                min = scores[i].score;
                index = i;
            }
        }
        // 左右两个序号
        int left = scores[index].left;
        int right = scores[index].right;
        // 最后两个灯柱的坐标
        int left_x = led_rRect[left].center.x;
        int left_y = led_rRect[left].center.y;
        int right_x = led_rRect[right].center.x;
        int right_y = led_rRect[right].center.y;
        // 计算最后的装甲板的中心坐标
        int x_now = (left_x + right_x)/2;
        int y_now = (left_y + right_y)/2;
        // 计算装甲板的x方向和y方向的差
        double dx = abs(left_x - right_x);
        double dy  = abs(left_y - right_y);
        // 计算半径
        double radius = sqrt((pow(dx,2)+pow(dy,2)))/2;
        // 画出圆形
        circle(src,Point(x_now,y_now),1,Scalar(255),1);
        circle(src,Point(x_now,y_now),radius,Scalar(255),2);
        // 最后装甲板在原图的的宽和长以及坐标
        if(flag)
        {
            pre_value[0] = (pre_value[0]-pre_value[2]/2) + x_now;
            pre_value[1] = (pre_value[1]-pre_value[3]/2) + y_now;
            pre_value[2] = dx*1.6;
            pre_value[3] = radius * 2.2;
        }
        else
        {
            pre_value[0] = x_now;
            pre_value[1] = y_now;
            pre_value[2] = dx*1.5 + 10;
            pre_value[3] = radius * 2 + 10;
        }

        // 实际的装甲板的像素大小
        double led_width = led_rRect[left].size.width;
        double led_height = led_rRect[left].size.height;
        double height = max(led_width,led_height);
        final_ammor.center.x = pre_value[0];
        final_ammor.center.y = pre_value[1];
        final_ammor.angle = 0;
        final_ammor.size.width = dx;
        final_ammor.size.height = height;
        cout << "装甲板的长:" << height << "装甲板的宽:" << dx << endl;
        flag = true;
    }
}

void color_detect::detect(Mat &image,char color_mode)
{
    cout << "坐标为:" << pre_value[0] << "," << pre_value[1] << endl;
    mode_color = color_mode;
    src = image.clone();
    if(flag)
    {
        img_cut();
    }
    if(mode_color == 'B')
    {
        blue_detect();
    }
    if(mode_color == 'R')
    {
        red_detect();
    }
    vector<RotatedRect> Led_rRect;
    Rotated_detect(Led_rRect);
    ammor_detect(Led_rRect);
    imshow("led_rect",src);
}

RotatedRect color_detect::get_final_ammor()
{
    return final_ammor;
}

color_detect::~color_detect()
{

}
