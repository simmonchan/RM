#include "sudoku.h"

sudoku::sudoku()
{

}

void sudoku::pre_process(Mat &image)
{
    // 转为灰色
    cvtColor(image,image,CV_BGR2GRAY);
    // 自适应阈值
    threshold(image,image,0,255,CV_THRESH_OTSU);
}

// 得到直立矩形的中心
Point sudoku::getCenterPoint(Rect rect)
{
    Point cpt;
    cpt.x = rect.x + cvRound(rect.width/2.0);
    cpt.y = rect.y + cvRound(rect.height/2.0);
    return cpt;
}

void sudoku::find_num_Rect(const Mat &image,vector<Rect> &Rects_num)
{
    // 找轮廓
    vector<vector<Point>> contours;
    findContours(image,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    // 定义存储多边形的数据结构
    vector<vector<Point>> contours_poly( contours.size());
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // 利用点集的面积
        if (contourArea(contours[i]) > 300 && contourArea(contours[i]) < 10000)
        {
            // 多边形拟合
            approxPolyDP(contours[i],contours_poly[i],arcLength(contours[i], true)*0.06,true);
            int size_num = contours_poly[i].size();
            // 多边形的边数等于4，而且点集在凸包中
            if (size_num == 4 && isContourConvex(Mat(contours_poly[i])) )
            {
                Rect min_rect = boundingRect(contours_poly[i]);
                // 去掉横矩形
                if (min_rect.height > min_rect.width)
                {
                    Rects_num.push_back(min_rect);
                    //rectangle(src,min_rect,Scalar(255,0,0),2);
                }
            }
        }
    }
}

void sudoku::find_five_num_Rect(vector<Rect> &Rects_num,vector<Rect> &num_five_Rects)
{
    // 计算每个矩形到中心点的距离
    vector<double> distance;
    vector<int> index0;
    // 计算每一个矩形中心到图像中点的距离
    for(size_t i=0;i<Rects_num.size();i++)
    {
        Point rRect_center = getCenterPoint(Rects_num[i]);
        double distance_temp = sqrt(pow(rRect_center.x-320,2)+pow(rRect_center.y-200,2));
        distance.push_back(distance_temp);
        index0.push_back(i);
    }
    // 排序，求最小的值
    sort(index0.begin(),index0.end(),[&](const int &a, const int &b)
        {

            return (distance[a] < distance[b]);
        }
    );
    // 再计算到最近的那个矩形的距离
    distance.clear();
    vector<int> index1;
    for(size_t i=0;i<Rects_num.size();i++)
    {
        Point rRect_center = getCenterPoint(Rects_num[index0[0]]);
        double distance_temp = sqrt(pow((rRect_center.x-Rects_num[i].x),2)+pow((rRect_center.y-Rects_num[i].y),2));
        distance.push_back(distance_temp);
        index1.push_back(i);
    }
    // 排序
    sort(index1.begin(),index1.end(),[&](const int &a, const int &b)
        {

            return (distance[a] < distance[b]);
        }
    );
    // 取出前五个
    if (index0.size() >= 5)
    {
        for (int i=0;i<5;i++)
        {
            int indexs = index1[i];
            //cout << distance[indexs] << endl;
            //rectangle(src,rRect[indexs],Scalar(255,255,0),2);
            num_five_Rects.push_back(Rects_num[indexs]);
        }
    }
}

void sudoku::detect_num_Rect(Mat &image, vector<Rect> &num_five_Rects)
{
    if(num_five_Rects.size() > 0)
    {
        // 对每一个进行处理
        for (int i=0;i<5;i++)
        {
            // 预处理
            Rect roi = num_five_Rects[i];
            Mat src_roi = image(roi).clone();
            pre_process(src_roi);
            medianBlur(src_roi,src_roi,3);
            if(!src_roi.empty())
            {
//                    stringstream num1;
//                    num1 << "/home/chan/图片/" << filename++ << ".jpg";
//                    string number = num1.str();
                //imshow(number,src_roi);
                //imwrite(number,src_roi);
                Mat traindata, trainlabel, tmp;
                // 提取样本数据
                for (int i = 1; i < 10 ; i++)
                {
                    for (int j=0;j<20;j++)
                    {
                        // 图片的路径
                        stringstream src_path;
                        src_path << "/home/chan/图片/" << i << "/" << j << ".jpg";
                        string filename = src_path.str();
                        tmp = imread(filename);   // 读取数据集图像信息
                        cvtColor(tmp,tmp,CV_BGR2GRAY);
                        threshold(tmp,tmp,0,255,CV_THRESH_OTSU);
                        resize(tmp, tmp, Size(30,30),CV_INTER_LINEAR);
                        traindata.push_back(tmp.reshape(0, 1));
                        trainlabel.push_back(i);  // 附件标签信息
                    }
                }
                traindata.convertTo(traindata, CV_32F);

                // 最终分类的个数
                int K = 1;
                // 创建数据
                Ptr<ml::TrainData> tData = ml::TrainData::create(traindata,0,trainlabel);
                // 创建KNN训练器
                Ptr<ml::KNearest> knn = ml::KNearest::create();
                // 进行训练
                knn->setDefaultK(K);
                knn->setIsClassifier(true);
                knn->train(tData);
                //测试数据进行
                resize(src_roi, src_roi, Size(30,30),CV_INTER_LINEAR);
                src_roi = src_roi.reshape(0, 1);
                src_roi.convertTo(src_roi, CV_32F);
                int r = knn->predict(src_roi);   //对所有行进行预测
                //cout << r << endl;

                string num = to_string(r);
                Point cpt = num_five_Rects[i].tl();
                putText(image,num,cpt,FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(255,255,0),2);
                //imshow("num",src_roi);
            }

        }
    }
}

void sudoku::detect_sudoku_num(Mat &image, vector<Rect> &num_five_Rects)
{
    int index2[5] = {0,1,2,3,4};
    if(num_five_Rects.size() > 0)
    {
        // 给5个矩形排序
        sort(index2,index2+5,[&](const int &a, const int &b)
            {

                return (num_five_Rects[a].tl().x < num_five_Rects[b].tl().x);
            }
        );
         //截出五个数字的ROI
        Rect roi1 = num_five_Rects[index2[0]];
        Point tl1 = roi1.tl();
        Rect roi2 = num_five_Rects[index2[4]];
        Point tl2 = roi2.br();
        double x = abs(tl1.x - tl2.x);

        tl1.x -= (x*0.7);
        if(tl1.x<0)
        {
            tl1.x = 1;
        }

        tl1.y += (x*0.25);
        if(tl1.y <0)
        {
            tl1.y = 1;
        }

        tl2.x += (x*0.6);
        if (tl2.x>640)
        {
            tl2.x = 639;
        }

        tl2.y += (x*1.5);
        if(tl2.y>480)
        {
            tl2.y = 479;
        }

        if (tl1.x > tl2.x || tl1.y > tl2.y)
        {
            return;
        }
        Rect ROI1(tl1,tl2);

        Mat src_roi1 = image(ROI1).clone();
        pre_process(src_roi1);
        threshold(src_roi1,src_roi1,0,255,CV_THRESH_BINARY_INV);

        if(!src_roi1.empty())
        {
            imshow("sssss",src_roi1);
            struct ROI_localtion
            {
                Rect ROI[9];
                int index[9];
            }area;
            int ROI_count = 0;
            vector<vector<Point> > contours; // 轮廓的点集
            findContours(src_roi1,contours,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
            for (int i=0;i<(int)contours.size();i++)
            {
                //对每个轮廓进行拟合
                Rect rRect =  boundingRect(contours[i]);
                // 判断大小和长宽信息
                if (rRect.area() > 4000 && (rRect.width > rRect.height) && rRect.area() < 10000)
                {
                    ROI_count++;
                    //rectangle(src,rRect,Scalar(255,0,0),2);
                    area.ROI[ROI_count-1] = rRect;
                    area.index[ROI_count-1] = ROI_count - 1;
                    //cout << "第" << ROI_count-1 << "个ROI的左上坐标为" << rRect.x << "," << rRect.y << endl;
                    //cout << "第" << ROI_count-1 << "个ROI的长和宽为" << rRect.height << "," << rRect.width << endl;
                }
            }
            // 如果大于或者小于9个的话，说明检测失败
            if (ROI_count!=9)
            {
                return;
            }

            // 对x进行排序
            sort(area.index,area.index+9,[&](const int &a, const int &b)
                {

                    return (area.ROI[a].x < area.ROI[b].x);
                }
            );

            // 对y进行排序
            for (int i=0;i<3;i++)
            {
                sort(area.index+i*3,area.index+i*3+3,[&](const int &a, const int &b)
                    {
                        return (area.ROI[a].y < area.ROI[b].y);
                    }
                );
            }
            // 测试输出
            for (int i=0;i<9;i++)
            {
               int index = area.index[i];

               Mat src_num = src_roi1(area.ROI[index]).clone();
               resize(src_num,src_num,Size(28,28),CV_INTER_CUBIC);
               Size size = src_num.size();
               int height = size.height;
               int width = size.width;
               Point center = Point(height/2,width/2);
               Mat M = getRotationMatrix2D(center,2,1);
               warpAffine(src_num,src_num,M,size);



               //threshold(src_num,src_num,0,255,CV_THRESH_BINARY_INV);
               //cvtColor(src_num,src_num,CV_BGR2GRAY);
               cout << "第" << i+1 <<  "个" ;
               int number;
               number = yuche(src_num);
               string num = to_string(number);
               Point cpt;
               cpt.x = area.ROI[index].x + cvRound(area.ROI[index].width/2.0)+tl1.x;
               cpt.y = area.ROI[index].y + cvRound(area.ROI[index].height/2.0)+tl1.y;
               putText(image,num,cpt,FONT_HERSHEY_COMPLEX,2,Scalar(255,255,0),3);
            }
        }

  }
}

void sudoku::getMaxClass(const Mat &probBlob, int *classId, double *classProb)
{
    Mat probMat = probBlob.reshape(1, 1);
    Point classNumber;

    minMaxLoc(probMat, NULL, classProb, NULL, &classNumber);
    *classId = classNumber.x;
}

std::vector<String> sudoku::readClassNames(const char *filename = "/home/chan/caffe-cnn/mnist/lenet_labels.txt")
{
    std::vector<String> classNames;

    std::ifstream fp(filename);
    if (!fp.is_open())
    {
        std::cerr << "File with classes labels not found: " << filename << std::endl;
        exit(-1);
    }

    std::string name;
    while (!fp.eof())
    {
        std::getline(fp, name);
        if (name.length())
            classNames.push_back(name.substr(name.find(' ') + 1));
    }
    fp.close();
    return classNames;
}

int sudoku::yuche(Mat &src)
{
//    //初始化
    CV_TRACE_FUNCTION();
    //读取模型测试参数和模型结构文件
    String modelTxt = "/home/chan/caffe-cnn/mnist/lenet_deploy.prototxt";
    String modelBin = "/home/chan/caffe-cnn/mnist/finally_model/train_mnist_iter_10000.caffemodel";
    //读取图片
    String imageFile = "/home/chan/caffe-cnn/mnist/00.png";

    //合成网络
    Net net = dnn::readNetFromCaffe(modelTxt, modelBin);
    //判断网络是否生成成功
    if (net.empty())
    {
        std::cerr << "Can't load network by using the following files: " << std::endl;
        exit(-1);
    }
    cerr << "net read successfully" << endl;

    //读取图片
    Mat img = src.clone();
    imshow("image", img);
    if (img.empty())
    {
        std::cerr << "Can't read image from the file: " << imageFile << std::endl;
        exit(-1);
    }
    cerr << "image read sucessfully" << endl;

    Mat inputBlob = blobFromImage(img, 0.00390625, Size(28,28),
                                    Scalar(33.3184),false);

    //构造blob，为传入网络做准备，图片不能直接进入网络
    // 对应train_prototxt文件的缩放因子 数据变化缩放因子
   // Mat inputBlob = blobFromImage(img, 1, Size(28,28));

    Mat prob;
    cv::TickMeter t;
    for (int i = 0; i < 1; i++)
    {
        CV_TRACE_REGION("forward");
        //将构建的blob传入网络data层
        net.setInput(inputBlob,"data");
        //计时
        //t.start();
        //前向预测
        prob = net.forward("prob");
        //停止计时
        //t.stop();
    }

    int classId;
    double classProb;
    //找出最高的概率ID存储在classId，对应的标签在classProb中
    getMaxClass(prob, &classId, &classProb);

    //打印出结果
    char *filename = "/home/chan/caffe-cnn/mnist/lenet_labels.txt";
    std::vector<String> classNames = readClassNames(filename);
    //std::cout << "Best class: #" << classId << " '" << classNames.at(classId) << "'" << std::endl;
    //std::cout << "Probability: " << classProb * 100 << "%" << std::endl;
    //打印出花费时间
    //std::cout << "Time: " << (double)t.getTimeMilli() / t.getCounter() << " ms (average from " << t.getCounter() << " iterations)" << std::endl;

    //便于观察结果
//    waitKey(1);
    //return 0;
//    shenfu_yuchuli();
    return classId;
}

void sudoku::detect(Mat &image)
{
    src = image.clone();
    Mat imgae = src.clone();
    pre_process(src);
    imshow("gray",src);
    vector<Rect> Rects_num;
    find_num_Rect(src,Rects_num);

    find_five_num_Rect(Rects_num,five_num_Rects);
    detect_num_Rect(imgae,five_num_Rects);
    detect_sudoku_num(imgae,five_num_Rects);
    imshow("image",imgae);
}
