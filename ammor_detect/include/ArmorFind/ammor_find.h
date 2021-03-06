#ifndef AMMOR_FIND_H
#define AMMOR_FIND_H
#include <include/Header.h>

using namespace std;
using namespace cv;


static inline bool RotateRectSort(RotatedRect a1,RotatedRect a2){
    return a1.center.x < a2.center.x;
}

static inline bool CenterSort(const Armordata a1,const Armordata a2){
    return a1.armor_center.x < a2.armor_center.x;
}


class Ammor_find
{
public:
    Ammor_find();
    void Color_process(Mat &src);
    void Find_lightbar();
    void GetArmors();
    void GetArmors_far();
    void GetArmor_img_cut();
    double GetK(Point2f L1,Point2f L2);
    void sort_Rotated_Point(Point2f _pt[4],Point2f pt[4]);
    void img_cut();
    void detect(const Mat &image,const bool mode,const uchar Fordebug);
    void clear();

public:
    vector<Armordata> _Armordatas;
    vector<Point2f> _ArmorPoints;
    Armordata _LastArmor;
private:
    Mat _src;
    Mat _binary;
    InitParams _params;
    bool _mode;
    vector<RotatedRect> _Rect_led;
    vector<vector<RotatedRect> > Armorlists;
    vector<int> CellMaxs;
    vector<RotatedRect> RectfirstResult;
    int _ArmorLostDelay;
    uchar _ForDebug;
};

#endif // AMMOR_FIND_H
