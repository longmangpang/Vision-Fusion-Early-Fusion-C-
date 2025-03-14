#ifndef LIDARTOCAMERA_H
#define LIDARTOCAMERA_H

#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>


using namespace std;
using namespace cv;
using namespace Eigen;



class LiDARtoCamera {
public:
    LiDARtoCamera();
    std::vector<cv::Point3f> projectPCDtoImage(const vector<Vector3d>& pcd_points,const string& calibfile);
    void showPCDOnImage(Mat& image, const vector<Vector3d>& pcd_points,const string& calibfile);

private:
    unordered_map<string, MatrixXd> calib_data;
    Matrix<double, 3, 4> P;
    Matrix<double, 3, 3> R0;
    Matrix<double, 3, 4> V2C;

    unordered_map<string, MatrixXd> readCalibFile(const string& filepath);
    MatrixXd R0To4x4(MatrixXd R0) const;
    MatrixXd V2CTo4x4(MatrixXd V2C) const;
};

#endif // LIDARTOCAMERA_H
