#ifndef SUDOKU_H
#define SUDOKU_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;
using namespace cv::dnn;

class sudoku
{
public:
    sudoku();
    void pre_process(Mat &image);
    void find_num_Rect(const Mat &image,vector<Rect> &Rects_num);
    Point getCenterPoint(Rect rect);
    void find_five_num_Rect(vector<Rect> &Rects_num,vector<Rect> &num_five_Rects);
    void detect_num_Rect(Mat &image,vector<Rect> &num_five_Rects);
    void detect_sudoku_num(Mat &image, vector<Rect> &num_five_Rects);
    void getMaxClass(const Mat &probBlob, int *classId, double *classProb);
    std::vector<String> readClassNames(const char *filename);
    int yuche(Mat &src);
    void detect(Mat &image);


private:
    vector<Rect> five_num_Rects;
    int sudoku_width;
    int sudoku_height;
    Mat src;
};

#endif // SUDOKU_H
