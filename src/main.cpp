
#include <regex>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <ctime>
#include "utils_my.h"
#include "yolo8_seg_inference.h"
#include "LiDARtoCamera.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Dense>



std::string extractFileNameWithoutExtension(const std::string& filePath) {
    // 查找文件路径中最后一个"/"的位置
    size_t lastSlash = filePath.find_last_of("/\\");
    if (lastSlash == std::string::npos) {
        lastSlash = 0; // 如果没有找到"/"，从字符串开头开始
    } else {
        lastSlash += 1; // 移动到"/"之后的位置
    }

    // 查找文件名中最后一个"."的位置
    size_t lastDot = filePath.find_last_of('.');
    if (lastDot == std::string::npos) {
        lastDot = filePath.length(); // 如果没有找到"."，取整个文件名
    }

    // 提取并返回文件名（不包括扩展名）
    return filePath.substr(lastSlash, lastDot - lastSlash);
}

int main(int argc, char* argv[])
{
	srand(time(0));
    
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 创建一个Eigen向量来存储点云数据
    std::vector<Eigen::Vector3d> pcd_points;

    LiDARtoCamera liDARtoCamera = LiDARtoCamera();


    // 分割模型
	std::string model_path = "/data/PCL_test/Vision-Fusion-Early-Fusion_C++/data/onnx/yolov8n-seg.onnx";
    float confThreshold = 0.4f;
    float iouThreshold = 0.4f;
    float maskThreshold = 0.5f;

    YOLOPredictor  predictor = YOLOPredictor(model_path, true,
                                  confThreshold,
                                  iouThreshold,
                                  maskThreshold);
    const std::vector<std::string> classNames = utils_my::loadNames("/data/PCL_test/Vision-Fusion-Early-Fusion_C++/resources/coco.names");

    std::string imagePath = "/data/PCL_test/Vision-Fusion-Early-Fusion_C++/data/img";
    const std::string suffixName = "yolov8";
    const std::string savePath = "/data/PCL_test/Vision-Fusion-Early-Fusion_C++/output";
	std::regex pattern(".+\\.(jpg|jpeg|png|gif)$");
    clock_t startTime, endTime;
    startTime = clock();
    int picNums = 0;
    
    for (const auto &entry : std::filesystem::directory_iterator(imagePath))
    {
        if (std::filesystem::is_regular_file(entry.path()) && std::regex_match(entry.path().filename().string(), pattern))
        {
            picNums += 1;
            std::string Filename = entry.path().string();
            std::string baseName = std::filesystem::path(Filename).filename().string();
            std::cout << Filename << " predicting..." << std::endl;

            cv::Mat image = cv::imread(Filename);
            std::vector<Yolov8Result> result = predictor.predict(image);
            // 获取pcd文件
            std::string fileName = extractFileNameWithoutExtension(Filename);
            std::string input_pcd = "/data/PCL_test/Vision-Fusion-Early-Fusion_C++/data/velodyne/"+fileName+".pcd";
            
            // 读取.pcd文件
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *cloud) < 0)
            {
                PCL_ERROR("Couldn't read file input.pcd \n");
                return (-1);
            }

                // 将点云数据从PCL转换到Eigen向量
            for (const auto& point : *cloud) {
                pcd_points.push_back(Eigen::Vector3d(point.x, point.y, point.z));
            }

            // 获取标定文件
            const std::string calibfile_fileName = "/data/PCL_test/Vision-Fusion-Early-Fusion_C++/data/calib/"+fileName+".txt";

            std::vector<cv::Point3f> img_points  = liDARtoCamera.projectPCDtoImage(pcd_points,calibfile_fileName);

            utils_my::visualizeDetection(image, result, classNames,img_points);
            // utils_my::visualizeDetection(image, result, classNames);

            std::string newFilename = baseName.substr(0, baseName.find_last_of('.')) + "_" + suffixName + baseName.substr(baseName.find_last_of('.'));
            std::string outputFilename = savePath + "/" + newFilename;
            cv::imwrite(outputFilename, image);
            std::cout << outputFilename << " Saved !!!" << std::endl;
        }
    }
    endTime = clock();
    std::cout << "The total run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "seconds" << std::endl;
    std::cout << "The average run time is: " << (double)(endTime - startTime) / picNums / CLOCKS_PER_SEC << "seconds" << std::endl;

    std::cout << "##########DONE################" << std::endl;

    return 0;
   
}
