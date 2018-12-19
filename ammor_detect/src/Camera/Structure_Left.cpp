//
// Created by parallels on 4/27/17.
//

#include "include/Camera/Structure_Left.h"
FILE *flog;
struct shared_package_left *shared_package_ptr = NULL;
using namespace std;
using namespace cv;
long long frame_count_last = 0;
uint8_t image_buffer[921600];
int image_buffer_len;
struct shared_package_left *get_shared_package()
{
    int shmid;
    shmid = shmget(960827, sizeof(struct shared_package_left), 0666 | IPC_CREAT);
    struct shared_package_left *s = (struct shared_package_left *) shmat(shmid, NULL, 0);
    printf("shmid:%d, p:%lx\n", shmid, s);
    return s;
}
uint8_t getImageFromMemory_left(Mat &image)
{
    if (shared_package_ptr == NULL)shared_package_ptr = get_shared_package();

    //pthread_rwlock_rdlock(&shared_package_ptr->image_lock);
    int nowsize = shared_package_ptr->image_size;
    vector<char> img_data(shared_package_ptr->image_data, shared_package_ptr->image_data + nowsize);
    long long frame_count_now = shared_package_ptr->count;

    //pthread_rwlock_unlock(&shared_package_ptr->image_lock);
    if (nowsize == 0)return -1;

    if (frame_count_now == frame_count_last)return -2;

    try
    {

        image = imdecode(Mat(img_data), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);

        if (image.size().width <= 0 || image.size().height <= 0)return -1;

        frame_count_last = frame_count_now;

    }
    catch (cv::Exception &e)
    {
        return -1;
    }

    return 0;


}
//uint8_t getImageFromMemory()
//{
//    if (shared_package_ptr == NULL)shared_package_ptr = get_shared_package();

//    image_buffer_len = shared_package_ptr->image_size;
//    int frame_count_now = shared_package_ptr->count;
//    memcpy(image_buffer, shared_package_ptr->image_data, image_buffer_len);

//    if (image_buffer_len == 0)return 1;

//    if (frame_count_now == frame_count_last)return 2;

//    frame_count_last = frame_count_now;
//    return 0;
//}
void printlog(const char *format, ...)
{
    va_list args;

    va_start(args , format);
    vprintf(format , args);
    va_end(args);

    va_start(args, format);
    vfprintf(flog, format, args);
    va_end(args);
    fflush(flog);
}
