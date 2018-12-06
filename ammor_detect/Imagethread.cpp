#include "Imagethread.h"
#include "ammor_find.h"
#include "QTime"
#include "time.h"

using namespace cv;
using namespace std;

#define BUFFER_SIZE 3

unsigned int prdIdx = 0;
unsigned int csmIdx = 0;

struct ImageData
{
    Mat img;
    unsigned int frame;
};

Mat data[BUFFER_SIZE];


ImageThread::ImageThread()
{

}

void ImageThread::ImageProducer(RMVideoCapture &cap)
{
    while(1){
        while(prdIdx - csmIdx >= BUFFER_SIZE);
        cap >> data[prdIdx % BUFFER_SIZE];
        int frame = cap.getFrameCount();
        ++prdIdx;
        cout << frame << endl;
    }
}

void ImageThread::ImageConsunmer(){
    Mat src;
    Ammor_find armor_detect;
    while(1){
        while(prdIdx - csmIdx == 0);
        data[csmIdx % BUFFER_SIZE].copyTo(src);
        ++csmIdx;
        if(!src.empty()){
            armor_detect.detect(src,BLUE_DETECT);
            armor_detect.clear();
        }
    }
}
