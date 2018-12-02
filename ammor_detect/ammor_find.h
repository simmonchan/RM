#ifndef AMMOR_FIND_H
#define AMMOR_FIND_H
#include <Header.h>

//#define SHOW_DEBUG

using namespace std;
using namespace cv;


static inline bool RotateRectSort(RotatedRect a1,RotatedRect a2){
    return a1.center.x < a2.center.x;
}

class Ammor_find
{
public:
    Ammor_find();
    void Color_process(const Mat &src);
    void Find_lightbar();
    void GetArmors();
    double GetK(Point2f L1,Point2f L2);
    void sort_Rotated_Point(Point2f _pt[4],Point2f pt[4]);
    void detect(Mat &image,bool mode);
    void clear();

public:
    vector<Armordata> _Armordatas;
    vector<Point> _ArmorPoints;
private:
    InitParams _params;
    bool _mode;
    Mat _src;
    Mat _binary;
    vector<RotatedRect> _Rect_led;
};

#endif // AMMOR_FIND_H
