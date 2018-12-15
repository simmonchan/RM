#include "include/ArmorFind/ammor_find.h"

Ammor_find::Ammor_find(){
    _mode = GET_NO;
    _ArmorLostDelay = -1;
}

/**
  * @brief find the color binary
  * @param src: the image of frame
  * @return none
  */
void Ammor_find::Color_process(Mat &src)
{
    Mat thres_whole;
    vector<Mat> splited(3);
    split(src,splited);
    cvtColor(src,thres_whole,CV_BGR2GRAY);

    threshold(thres_whole,thres_whole,_params.armor_thres_whole,255,THRESH_BINARY);
    if( _mode == RED_DETECT){
        subtract(splited[2],splited[0],_binary);
        threshold(_binary,_binary,_params.armor_thres_red,255,THRESH_BINARY);// red
    }else if(_mode == BLUE_DETECT){
        subtract(splited[0],splited[2],_binary);
        threshold(_binary,_binary,_params.armor_thres_blue,255,THRESH_BINARY);// blue
    }
    Mat contourkernel = getStructuringElement(MORPH_ELLIPSE,Size(9,9));
    dilate(_binary,_binary,contourkernel);
    _binary = _binary & thres_whole;
//    Mat s = getStructuringElement(MORPH_ELLIPSE,Size(3,3));
//    dilate(_binary,_binary,s);
#ifdef IMAGE_DEBUG
    if(_ForDebug == 1){
        imshow("left_binary",_binary);
    }else if(_ForDebug == 2){
        imshow("right_binary",_binary);
    }
#endif
}

/**
  * @brief find the light_bar
  * @param  none
  * @return none
  */
void Ammor_find::Find_lightbar()
{
    // to find the light bars and sort
    vector<vector<Point>> contours;
    findContours(_binary,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,_LastArmor.armor_points[0]);

    size_t contours_size = contours.size();
    for (size_t i=0;i<contours_size;i++)
    {
       RotatedRect rRect =  minAreaRect(contours[i]);
       // remove the crosswide
       if(rRect.size.area() > 50)
       {
         if(((rRect.size.width > rRect.size.height) && (rRect.angle < -55)) ||
                   ((rRect.size.width < rRect.size.height) && (rRect.angle > -35)))
         {
#ifdef IMAGE_DEBUG
            Point2f points[4];
            rRect.points(points);
            for (int i=0;i<4;i++)
            {
                int j = (i+1)%4;
                line (_src,points[i],points[j],Scalar(255,0,255),2);
            }
#endif
            RectfirstResult.push_back(rRect);
         }
       }
    }

    // remove the closer
    size_t size = RectfirstResult.size();
    if(size >= 2){
        vector<RotatedRect> Groups;
        int cellmaxsize;
        Groups.push_back(RectfirstResult[0]);
        cellmaxsize = RectfirstResult[0].size.height * RectfirstResult[0].size.width;
        if(cellmaxsize > 2500) cellmaxsize = 0;
        int maxsize;

        for(size_t i=1;i<size;i++){
            if(RectfirstResult[i].center.x - RectfirstResult[i-1].center.x <10){
                maxsize = RectfirstResult[i].size.height * RectfirstResult[i].size.width;
                if(maxsize > 2500) continue;
                if(maxsize > cellmaxsize) cellmaxsize = maxsize;
                Groups.push_back(RectfirstResult[i]);
            }else{
                Armorlists.push_back(Groups);
                CellMaxs.push_back(cellmaxsize);
                cellmaxsize = 0;
                maxsize = 0;
                Groups.clear();
                Groups.push_back(RectfirstResult[i]);
                cellmaxsize = RectfirstResult[i].size.height * RectfirstResult[i].size.width;
            }
        }
        Armorlists.push_back(Groups);
        CellMaxs.push_back(cellmaxsize);
        size = Armorlists.size();
        for(size_t i=0;i<size;i++){
            int Gsize = Armorlists[i].size();
            int GroupMax = CellMaxs[i];
            if(GroupMax > 5){
                for(int j=0;j<Gsize;j++){
                    maxsize = Armorlists[i][j].size.height * Armorlists[i][j].size.width;
                    if(maxsize == GroupMax){
                        _Rect_led.push_back(Armorlists[i][j]);
                    }
                }
            }
        }
    }
}

/**
  * @brief get the useful armor data
  * @param  none
  * @return none
  */
void Ammor_find::GetArmors()
{
    if(_Rect_led.size() >= 2)
    {
        sort(_Rect_led.begin(),_Rect_led.end(),RotateRectSort);
        Point2f L1,L2;
        float K=0.0,angleabs = 0.0,angleL1=0.0,angleL2=0.0; // angle data
        float areascale=0.0,areaL1=0.0,areaL2=0.0; // area data
        float ydis = 0.0;
        float maxangle=0.0,xdis=0.0,heightmax=0.0,hwdiv=0.0;// x or y dis
        Point2f _pt[4],pt_L1[4],pt_L2[4];
        auto ptangle = [](const Point2f &p1,const Point2f &p2){
            return fabs(atan2(p2.y-p1.y,p2.x-p1.x)*180.0/CV_PI);
        };

        size_t size = _Rect_led.size();
        for(size_t i=0;i<size-1;i++)
        {
            for(size_t j=1;j<size;j++)
            {
                // left params
                angleL1 = fabs(_Rect_led[i].angle);
                L1 = _Rect_led[i].center;
                areaL1 = _Rect_led[i].size.height * _Rect_led[i].size.width;
                _Rect_led[i].points(_pt);
                sort_Rotated_Point(_pt,pt_L1);
                /*pt
                 * 0 2
                 * 1 3
                 * */

                // right params
                L2 = _Rect_led[j].center;
                areaL2 = _Rect_led[j].size.width * _Rect_led[j].size.height;
                angleL2 = fabs(_Rect_led[j].angle);
                _Rect_led[j].points(_pt);
                sort_Rotated_Point(_pt,pt_L2);

                // the dis
                K = GetK(L1,L2);
                ydis = abs(L1.y - L2.y);
                xdis = abs(L1.x - L2.x);
                areascale = areaL1 / areaL2;
                heightmax =MAX(MAX(_Rect_led[i].size.width,_Rect_led[j].size.width),
                               MAX(_Rect_led[i].size.height,_Rect_led[j].size.height));
                hwdiv = xdis/heightmax;
                if(angleL1 > 45.0 && angleL2 < 45.0){
                    angleabs = 90.0 - angleL1 + angleL2;
                }else if(angleL1 <= 45.0 && angleL2 >= 45.0){
                    angleabs = 90.0 - angleL2 + angleL1;
                }else{
                    if(angleL1 > angleL2) angleabs = angleL1 - angleL2;
                    else angleabs = angleL2 - angleL1;
                }
                maxangle = MAX(ptangle(pt_L1[0],pt_L2[2]),ptangle(pt_L1[1],pt_L2[3]));
#ifdef SHOW_DEBUG
            cout << "K:" << K << endl;
            cout << "ydis:" << ydis << endl;
            cout << "xdis:" << xdis << endl;
            cout << "divscale:" << areascale << endl;
            cout << "heightmax:" << heightmax << endl;
            cout << "hwdiv:" << hwdiv << endl;
            cout << "angleabs:" << angleabs << endl;
            cout << "maxangle:" << maxangle << endl;
#endif
                if(fabs(K) < 0.5 && areascale < 3.0 && maxangle < 20.0 && hwdiv < 10.0 && ydis < 0.4*heightmax){
                    if(angleabs < 7){
                        Armordata pushdata;
                        if(hwdiv > 4.0){
                           pushdata.armor = pushdata.big_armor;
                        }
                        else{
                           pushdata.armor = pushdata.small_armor;
                        }
                        Point2f armor_center = Point2f(0.5*(L1.x+L2.x),0.5*(L1.y+L2.y));
                        pushdata.armor_center = armor_center;
                        pushdata.armor_points[0] = pt_L1[0];
                        pushdata.armor_points[1] = pt_L1[1];
                        pushdata.armor_points[2] = pt_L2[2];
                        pushdata.armor_points[3] = pt_L2[3];
                        _Armordatas.push_back(pushdata);
                        _ArmorPoints.push_back(armor_center);
#ifdef IMAGE_DEBUG
                    double radius = sqrt((pow(ydis,2) + pow(xdis,2)))/2;
                    circle(_src,Point(0.5*(L1.x+L2.x),0.5*(L1.y+L2.y)),1,Scalar(255),1);
                    circle(_src,Point(0.5*(L1.x+L2.x),0.5*(L1.y+L2.y)),radius,Scalar(255),2);
#endif
                    }
                }
            }
        }
    }
    if(_ArmorPoints.size() != 0 ){
        _ArmorLostDelay = 0;
    }
    else{
        _ArmorLostDelay += 0.5;
        if(_ArmorLostDelay > 5)
        {
            _ArmorLostDelay = -1;
        }
    }
}

/**
  * @brief get the K of two points
  * @param  L1:the first point
  * @param  L2:the second point
  * @return the K
  */
double Ammor_find::GetK(Point2f L1,Point2f L2){
    return (L1.y - L2.y) / (L1.x - L2.x);
}

/**
  * @brief to sort the Rotated points
  * @param  point2f _pt[4]:the original points
  * @param  point2f pt[4]: the output points
  * @return none
  */
void Ammor_find::sort_Rotated_Point(Point2f _pt[4],Point2f  pt[4])
{
    sort(_pt,_pt+4,[](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; });
    if(_pt[0].y < _pt[1].y)
    {
        pt[0] = _pt[0];
        pt[1] = _pt[1];
    }
    else
    {
        pt[0] = _pt[1];
        pt[1] = _pt[0];
    }
    if(_pt[2].y < _pt[3].y)
    {
        pt[2] = _pt[2];
        pt[3] = _pt[3];
    }
    else
    {
        pt[2] = _pt[3];
        pt[3] = _pt[2];
    }
}

/**
  * @brief cut the image
  * @param  none
  * @return none
  */
void Ammor_find::img_cut()
{
    if(_ArmorLostDelay >= 0){
        Point lu = _LastArmor.armor_points[0];
        Point rd = _LastArmor.armor_points[3];

        int width = (rd.x - lu.x) > 0 ? (rd.x - lu.x) : 0;
        int height = (rd.y - lu.y) > 0 ? (rd.y - lu.y) : 0;

        int top = (lu.y - height*(2 + _ArmorLostDelay))  > 0 ? (lu.y - height*(2 + _ArmorLostDelay)) : 1;
        int down = (rd.y + height*(2 + _ArmorLostDelay)) < 720 ? (rd.y + height*(2 + _ArmorLostDelay)) : 719;
        int left = (lu.x - width*(0.8 + _ArmorLostDelay)) > 0 ? (lu.x - width*(0.8 + _ArmorLostDelay)) : 1;
        int right = (rd.x + width *(0.8 + _ArmorLostDelay)) < 1280 ? (rd.x + width *(0.8 + _ArmorLostDelay)) : 1279;

        if (top < down && left < right){
            _src = _src(Range(top,down),Range(left,right));
            _LastArmor.armor_points[0] = Point2f(left,top);
        }
        else{
            _LastArmor.armor_points[0] = Point2f(0,0);
        }
    }
    else{
         _LastArmor.armor_points[0] = Point2f(0,0);
    }
}

/**
  * @brief detect the armor
  * @param  image:the src
  * @param  mode:color mode
  * @return none
  */
void Ammor_find::detect(const Mat &image,const bool mode,const uchar Fordebug)
{
    clear();
    _ForDebug = Fordebug;
    _mode = mode;
    _src = image.clone();
    img_cut();
    Color_process(_src);
    Find_lightbar();
    GetArmors();
#ifdef IMAGE_DEBUG
    if(_ForDebug == 1){
        imshow("left_src",_src);
    }
    else if (_ForDebug == 2){
       imshow("right_src",_src);
    }
#endif
}

/**
  * @brief clear the varible
  * @param  none
  * @return none
  */
void Ammor_find::clear()
{
    RectfirstResult.clear();
    CellMaxs.clear();
    Armorlists.clear();
    _Rect_led.clear();
    _Armordatas.clear();
    _ArmorPoints.clear();
}
