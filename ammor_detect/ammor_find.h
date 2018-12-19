#ifndef AMMOR_FIND_H
#define AMMOR_FIND_H
#include <Header.h>

//#define SHOW_DEBUG
//#define IMAGE_DEBUG

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
    void img_cut();
    void detect(const Mat &image,Mat &deung_image,const bool mode,vector<Armordata> &Armordatas, vector<Point2f> &ArmorPoints, bool &flag);
    void clear();

public:
    vector<Armordata> _Armordatas;
    vector<Point2f> _ArmorPoints;
    Armordata _LastArmor;
    bool _flag;
private:
    Mat _src;
    Mat _binary;
    InitParams _params;
    bool _mode;
    vector<RotatedRect> _Rect_led;
    uchar _ArmorLostDelay;

};

#endif // AMMOR_FIND_H
