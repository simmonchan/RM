#ifndef IMAGETHREAD_H
#define IMAGETHREAD_H
#include "RMVideoCapture.h"


class ImageThread
{
public:
    ImageThread();
    void ImageProducer(RMVideoCapture &cap);
    void ImageConsunmer();
};

#endif // IMAGETHREAD_H
