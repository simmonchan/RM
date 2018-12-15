#include "include/Camera/camera_calibration.h"

void camera_calibration(int image_num,Size chessboard_one)
{
    /*首先需要拍摄一组由同一个相机拍摄的10-20张棋盘图片*/
    /* 保存标定结果的文件*/
    ofstream out ("caliberation_result.txt");

    /*读取每一幅图像，从中提取角点，对角点进行亚像素精确化*/
    cout << "开始提取角点" << endl;
    int image_count=0;  /* 图像数量 */
    Size image_size;  /* 图像的尺寸 */
    Size board_size = Size(6,9);    /* 标定板上每行、列的角点数 */
    vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
    vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
    string filename; /*图像的文件名*/

    // 提取图片
    for(int i=0;i<image_num;i++)
    {
        image_count++;
        cout << "图片的数量" << image_count << endl;
        /* 转换文件的名字*/
        stringstream src_path;
        src_path << "/home/chan/桌面/bubing1/pic" << i << ".jpg";
        filename = src_path.str();

        Mat image_input = imread(filename);
        //imshow ("image",image_input);
        //waitKey(1000);
        /* 获取图像信息*/
        image_size.width = image_input.cols;
        image_size.height = image_input.rows;
        //cout << image_size.width << endl;
        //cout << image_size.height << endl;
        /* 提取角点 */
       if(findChessboardCorners(image_input,board_size,image_points_buf) == 0)
       {
           cout<<"找不到角点"; /*找不到角点*/
           exit(1);
       }
       else
       {
           Mat view_gray;
           cvtColor(image_input,view_gray,CV_RGB2GRAY);
           /* 亚像素精确化 */
           find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化
           //cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1)); 另外一种方法
           image_points_seq.push_back(image_points_buf);  //保存亚像素角点
           /* 在图像上显示角点位置 */
           drawChessboardCorners(view_gray,board_size,image_points_buf,false); //用于在图片中标记角点
           //imshow("Camera Calibration",view_gray);
           //waitKey(500);//暂停0.5S
       }
    }
    cout<<"角点提取完成！\n";
    /*以下是摄像机标定*/
    cout<<"开始标定" << endl;
    /*棋盘三维信息*/
    Size square_size = chessboard_one;  /* 实际测量得到的标定板上每个棋盘格的大小 */
    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
    /*内外参数*/
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
    vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
    vector<int> point_counts;  // 每幅图像中角点的数量
    /* 初始化标定板上角点的三维坐标 */
    int i,j,t;
    for (t=0;t<image_count;t++)
    {
        vector<Point3f> tempPointSet;
        for (i=0;i<board_size.height;i++)
        {
            for (j=0;j<board_size.width;j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
    }
    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i=0;i<image_count;i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    /* 开始标定 */
    calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    out << "相机的内参矩阵\n" << cameraMatrix << endl;
    out << "相机的畸变系数\n" << distCoeffs << endl;
    for(int i = 0;i != image_count;i++)
    {
        out << "第" << i << "幅图像的旋转向量和平移向量" << endl;
        out << rvecsMat[i] << endl;
        out << tvecsMat[i] << endl;
    }
    cout<<"标定完成！\n";
    /*对标定结果进行评价*/
//    cout<<"开始评价标定结果………………\n";
//    double total_err = 0.0; /* 所有图像的平均误差的总和 */
//    double err = 0.0; /* 每幅图像的平均误差 */
//    vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
//    cout<<"\t每幅图像的标定误差：\n";
//    fout<<"每幅图像的标定误差：\n";
//    for (i=0;i<image_count;i++)
//    {
//        vector<Point3f> tempPointSet=object_points[i];
//        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
//        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points2);
//        /* 计算新的投影点和旧的投影点之间的误差*/
//        vector<Point2f> tempImagePoint = image_points_seq[i];
//        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
//        Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
//        for (int j = 0 ; j < tempImagePoint.size(); j++)
//        {
//            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
//            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
//        }
//        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
//        total_err += err/=  point_counts[i];
//        cout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;
//    }
//    cout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl;
//    cout<<"评价完成！"<<endl;
    /*对图像进行矫正*/
//    Mat mapx = Mat(image_size,CV_32FC1);
//    Mat mapy = Mat(image_size,CV_32FC1);
//    Mat R = Mat::eye(3,3,CV_32F);
//    cout<<"保存矫正图像"<<endl;
//    for (int i = 0 ; i != image_count ; i++)
//    {
//        initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,image_size,CV_32FC1,mapx,mapy);
//        stringstream src_file_path;
//        src_file_path << "/home/chan/githud/picture/" << i << ".jpg";
//        filename = src_file_path.str();
//        Mat imageSource = imread(filename);
//        Mat newimage = imageSource.clone();
//        //另一种不需要转换矩阵的方式
//        //undistort(imageSource,newimage,cameraMatrix,distCoeffs);
//        remap(imageSource,newimage,mapx, mapy, INTER_LINEAR);
//        stringstream dst_file_path;
//        dst_file_path.clear();
//        dst_file_path << "/home/chan/githud/picture/change_" << i << ".jpg";
//        filename = dst_file_path.str();
//        imwrite(filename,newimage);
//    }
//    cout<<"保存结束"<<endl;
}

void camera_two_calibration()
{
    /**************************************标定参数********************************************/
    Mat cameraMatrixL = (Mat_<double>(3, 3) <<
                         1328.7,   0 ,   708.4,
                                  0 ,   1329.6,    510.1,
                                  0 ,        0  ,  1);
    Mat distCoeffL = (Mat_<double>(5, 1) << -0.4391 ,0.2226,-0.0020 ,2.3817e-04,0.0);

    Mat cameraMatrixR = (Mat_<double>(3, 3) <<
                         1331.8,         0 ,   707.4,
                              0  ,  1334.7 ,   415.1,
                              0,         0 ,   1);
    Mat distCoeffR = (Mat_<double>(5, 1) << -0.4453, 0.2328, -0.0022 -0.0017 , 0.0);


    Mat T = (Mat_<double>(3, 1) << -197.9936 ,-2.6815 ,-8.8757);//T平移向量
    Mat R = (Mat_<double>(3, 3) <<
             0.9967,   -0.0090,   -0.0804,
             0.0166 ,   0.9954 ,   0.0945,
             0.0791,   -0.0955 ,   0.9923);//R旋转向量


    /****************************************标定函数需要的参数*************************************/
    int imageWidth = 1280;                             //摄像头的分辨率
    int imageHeight = 720;
    Size imageSize = Size(imageWidth, imageHeight);

    Rect validROIL,validROIR;          //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
    Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
    cout << "开始校准" << endl;

    /*立体校正*/
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
           0, imageSize, &validROIL, &validROIR);

    FileStorage fs("camera_calibrate.yaml",FileStorage::WRITE);
    fs << "cameraMatrixL" << cameraMatrixL << "distCoeffL" << distCoeffL << "Rl" << Rl << "Pl" << Pl;
    fs << "cameraMatrixR" << cameraMatrixR << "distCoeffR" << distCoeffR << "Rr" << Rr << "Pr" << Pr;
    fs << "Q" << Q;

    fs.release();
}
