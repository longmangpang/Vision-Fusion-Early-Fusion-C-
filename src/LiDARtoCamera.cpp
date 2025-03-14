#include "LiDARtoCamera.h"

LiDARtoCamera::LiDARtoCamera() {


}

// vector<Point2f> LiDARtoCamera::projectPCDtoImage(const vector<Vector3d>& pcd_points,const string& calibfile) {

//     unordered_map<string, MatrixXd> calib_data = readCalibFile(calibfile);
//     // 加载校准参数
   
//     P = calib_data["P2"];

//     R0 = calib_data["R0_rect"];

//     V2C = calib_data["Tr_velo_to_cam"];
//     MatrixXd points_homogeneous(4, pcd_points.size());
//     for (int i = 0; i < pcd_points.size(); ++i) {
//         points_homogeneous.col(i) << pcd_points[i](0), pcd_points[i](1), pcd_points[i](2), 1.0;
//     }

//     MatrixXd img_points_homogeneous = P * R0To4x4() * V2CTo4x4() * points_homogeneous;

//     vector<Point2f> img_points;
//     img_points.reserve(pcd_points.size());  // 预分配空间

//     // vector<Point2f> img_points(pcd_points.size());
//     for (int i = 0; i < pcd_points.size(); ++i) {
//         double x = img_points_homogeneous(0, i) / img_points_homogeneous(2, i);
//         double y = img_points_homogeneous(1, i) / img_points_homogeneous(2, i);
//         // img_points[i] = Point2f(x, y);
//         double depth = img_points_homogeneous(2, i);  // 获取深度值
//         // 过滤深度小于 2 的点
//         if (depth >= 2.0) {
//             img_points.push_back(Point2f(x, y));
//         }
//     }
//     return img_points;
    
// }

std::vector<cv::Point3f> LiDARtoCamera::projectPCDtoImage(const vector<Vector3d>& pcd_points, const string& calibfile) {
    unordered_map<string, MatrixXd> calib_data = readCalibFile(calibfile);
    MatrixXd P = calib_data["P2"];
    MatrixXd R0 = calib_data["R0_rect"];
    MatrixXd V2C = calib_data["Tr_velo_to_cam"];

    MatrixXd points_homogeneous(4, pcd_points.size());
    for (int i = 0; i < pcd_points.size(); ++i) {
        points_homogeneous.col(i) << pcd_points[i](0), pcd_points[i](1), pcd_points[i](2), 1.0;
    }

    MatrixXd img_points_homogeneous = P * R0To4x4(R0) * V2CTo4x4(V2C) * points_homogeneous;

    std::vector<cv::Point3f> img_points_with_depth;

    img_points_with_depth.reserve(pcd_points.size());

    for (int i = 0; i < pcd_points.size(); ++i) {
        double x = img_points_homogeneous(0, i) / img_points_homogeneous(2, i);
        double y = img_points_homogeneous(1, i) / img_points_homogeneous(2, i);
        double depth = img_points_homogeneous(2, i);  // 获取深度值
       
        // 过滤深度小于 2 的点
        if (depth >= 2.0) {
            img_points_with_depth.push_back(cv::Point3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(depth)));
        }
    }
    
    return img_points_with_depth;
}


void LiDARtoCamera::showPCDOnImage(Mat& image, const vector<Vector3d>& pcd_points,const string& calibfile) {
    std::vector<cv::Point3f> img_points = projectPCDtoImage(pcd_points,calibfile);

     // 获取图像的边界
    int img_width = image.cols;
    int img_height = image.rows;

       // 遍历投影后的点，检查是否在图像视场内
    for (const auto& pt : img_points) {
        // 检查点是否在图像边界内
        if (pt.x >= 0 && pt.x < img_width && pt.y >= 0 && pt.y < img_height) {
            // 如果在图像边界内，则绘制
            cv::circle(image, cv::Point(pt.x, pt.y), 3, cv::Scalar(0, 255, 0), cv::FILLED);
        }
    }
 
}

unordered_map<string, MatrixXd> LiDARtoCamera::readCalibFile(const string& filepath) {
    std::unordered_map<std::string, Eigen::MatrixXd> data;
    std::ifstream file(filepath);
    std::string line;

    while (std::getline(file, line)) {
        // Remove any leading/trailing whitespace characters
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line.empty()) {
            continue;
        }

        size_t colonPos = line.find(':');
        if (colonPos == std::string::npos) {
            continue; // Skip lines without a colon
        }

        std::string key = line.substr(0, colonPos);
        std::string valueStr = line.substr(colonPos + 1);

        // Use string stream to parse the values
        std::istringstream valueStream(valueStr);
        double value;
        std::vector<double> values;
        while (valueStream >> value) {
            values.push_back(value);
        }

        // Determine matrix dimensions based on the number of elements and common sizes.
        // For simplicity, we assume that matrices are either 3x3 or 3x4.
        Eigen::MatrixXd matrix;
        if (values.size() == 9) {
            matrix = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(values.data()).cast<double>();
        } else if (values.size() == 12) {
            matrix = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(values.data()).cast<double>();
        } else {
            // Handle other cases or throw an error if the size is unexpected
            throw std::runtime_error("Unexpected matrix size in calibration file.");
        }

        data[key] = matrix;
    }
    return data;
    
}

MatrixXd LiDARtoCamera::R0To4x4(MatrixXd R0) const {
    MatrixXd R0_4x4 = MatrixXd::Identity(4, 4);
    R0_4x4.block<3, 3>(0, 0) = R0;
    return R0_4x4;
}

MatrixXd LiDARtoCamera::V2CTo4x4(MatrixXd V2C) const {
    MatrixXd V2C_4x4 = MatrixXd::Identity(4, 4);
    V2C_4x4.block<3, 4>(0, 0) = V2C;
    return V2C_4x4;
}
