#include "utils_my.h"

size_t utils_my::vectorProduct(const std::vector<int64_t> &vector)
{
    if (vector.empty())
        return 0;

    size_t product = 1;
    for (const auto &element : vector)
        product *= element;

    return product;
}

std::wstring utils_my::charToWstring(const char *str)
{
    typedef std::codecvt_utf8<wchar_t> convert_type;
    std::wstring_convert<convert_type, wchar_t> converter;

    return converter.from_bytes(str);
}

std::vector<std::string> utils_my::loadNames(const std::string &path)
{
    // load class names
    std::vector<std::string> classNames;
    std::ifstream infile(path);
    if (infile.good())
    {
        std::string line;
        while (getline(infile, line))
        {
            if (line.back() == '\r')
                line.pop_back();
            classNames.emplace_back(line);
        }
        infile.close();
    }
    else
    {
        std::cerr << "ERROR: Failed to access class name path: " << path << std::endl;
    }
    // set color
    srand(time(0));

    for (int i = 0; i < 2 * classNames.size(); i++)
    {
        int b = rand() % 256;
        int g = rand() % 256;
        int r = rand() % 256;
        colors.push_back(cv::Scalar(b, g, r));
    }
    return classNames;
}

// void utils_my::visualizeDetection(cv::Mat &im, std::vector<Yolov8Result> &results,
//                                const std::vector<std::string> &classNames)
// {
//     cv::Mat image = im.clone();
//     for (const Yolov8Result &result : results)
//     {

//         int x = result.box.x;
//         int y = result.box.y;

//         int conf = (int)std::round(result.conf * 100);
//         int classId = result.classId;
//         std::string label = classNames[classId] + " 0." + std::to_string(conf);

//         int baseline = 0;
//         cv::Size size = cv::getTextSize(label, cv::FONT_ITALIC, 0.4, 1, &baseline);
//         image(result.box).setTo(colors[classId + classNames.size()], result.boxMask);

//         cv::rectangle(image, result.box, colors[classId], 2);
//         cv::rectangle(image,
//                       cv::Point(x, y), cv::Point(x + size.width, y + 12),
//                       colors[classId], -1);
//         cv::putText(image, label,
//                     cv::Point(x, y - 3 + 12), cv::FONT_ITALIC,
//                     0.4, cv::Scalar(0, 0, 0), 1);
//     }
//     cv::addWeighted(im, 0.4, image, 0.6, 0, im);
// }

cv::Scalar getDepthColor(float depth) {
    // 假设深度范围是 [minDepth, maxDepth]
    const float minDepth = 0.0f;
    const float maxDepth = 10.0f;

    // 将深度值归一化到 [0, 1]
    float normalizedDepth = (depth - minDepth) / (maxDepth - minDepth);

    // 将归一化深度值映射到颜色范围内
    // 这里我们使用红色到蓝色的渐变
    int red = static_cast<int>(255 * (1 - normalizedDepth));
    int blue = static_cast<int>(255 * normalizedDepth);
    int green = 0; // 绿色分量保持为0

    return cv::Scalar(blue, green, red);
}


// void saveMaskToTxt(const cv::Mat& mask, const std::string& filename) {
//     std::ofstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "无法打开文件: " << filename << std::endl;
//         return;
//     }

//     for (int i = 0; i < mask.rows; ++i) {
//         for (int j = 0; j < mask.cols; ++j) {
//             // 假设 mask 是 CV_8U 类型
//             unsigned char pixelValue = mask.at<unsigned char>(i, j);
//             file << static_cast<int>(pixelValue) << " ";
//         }
//         file << std::endl;
//     }

//     file.close();
//     std::cout << "掩码已成功保存到: " << filename << std::endl;
// }

void utils_my::visualizeDetection(cv::Mat &im, std::vector<Yolov8Result> &results,
    const std::vector<std::string> &classNames,
    const std::vector<cv::Point3f> &img_points_with_depth)
{
    cv::Mat image = im.clone();

    // 获取图像的宽度、高度和通道数
    int image_width = image.cols;
    int image_height = image.rows;

    
    for (const Yolov8Result &result : results)
    {   
        int x = result.box.x;
        int y = result.box.y;
        int width = result.box.width;
        int height = result.box.height;

        int conf = (int)std::round(result.conf * 100);
        int classId = result.classId;
        std::string label = classNames[classId] + " " + std::to_string(conf) + "%";

        int baseline = 0;
        cv::Size size = cv::getTextSize(label, cv::FONT_HERSHEY_COMPLEX, 0.5, 1, &baseline);

        cv::rectangle(image, result.box, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(image,
        cv::Point(x, y - size.height - 3), cv::Point(x + size.width, y + baseline),
        cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(image, label,
        cv::Point(x, y - 3), cv::FONT_HERSHEY_COMPLEX,
        0.5, cv::Scalar(0, 0, 0), 1);
        // image(result.box).setTo(colors[classId + classNames.size()], result.boxMask);
      
        // 绘制img_points中的点
        for (const cv::Point3f &point : img_points_with_depth) {
            cv::Point intPoint(static_cast<int>(point.x), static_cast<int>(point.y));
            // 判断点是否在 result.box 中
            if (intPoint.x >= result.box.x && intPoint.x < result.box.x + result.box.width &&
                intPoint.y >= result.box.y && intPoint.y < result.box.y + result.box.height) {
                // 计算在 result.boxMask 中的相对坐标
                int maskX = intPoint.x - result.box.x;
                int maskY = intPoint.y - result.box.y;
                // 检查该点在 result.boxMask 中的值是否为 255
                if (maskX >= 0 && maskX < result.boxMask.cols && maskY >= 0 && maskY < result.boxMask.rows) {
                    if (result.boxMask.at<uchar>(maskY, maskX) == 255) {
                        // 根据深度值获取颜色
                        cv::Scalar color = getDepthColor(point.z);
                        cv::circle(image, intPoint, 1, color, -1);
                    }
                }
            }
        }
       
    }
    cv::addWeighted(im, 0.4, image, 0.6, 0, im);
}


// void utils_my::visualizeDetection(cv::Mat &im, std::vector<Yolov8Result> &results,
//     const std::vector<std::string> &classNames,const std::vector<cv::Point2f> &img_points)
// {
//     cv::Mat image = im.clone();
//     std::vector<cv::Scalar> colors(classNames.size(), cv::Scalar(0, 255, 0)); // 假设使用绿色绘制矩形框

//     for (const Yolov8Result &result : results)
//     {
//         int x = result.box.x;
//         int y = result.box.y;
//         int width = result.box.width;
//         int height = result.box.height;

//         int conf = (int)std::round(result.conf * 100);
//         int classId = result.classId;
//         std::string label = classNames[classId] + " " + std::to_string(conf) + "%";

//         int baseline = 0;
//         cv::Size size = cv::getTextSize(label, cv::FONT_HERSHEY_COMPLEX, 0.5, 1, &baseline);
        
//         // 在result.boxMask指定的区域内绘制img_points中的点
//         for (const auto& point : img_points) {
//             // 检查点是否在boxMask的矩形区域内
//             if (point.x >= x && point.x <= x + width && point.y >= y && point.y <= y + height) {
//                 // 检查点是否在boxMask的非零区域内
//                 cv::Point maskPoint;
//                 maskPoint.x = static_cast<int>(point.x) - x;
//                 maskPoint.y = static_cast<int>(point.y) - y;
//                 if (maskPoint.x >= 0 && maskPoint.x < result.boxMask.cols &&
//                     maskPoint.y >= 0 && maskPoint.y < result.boxMask.rows &&
//                     result.boxMask.at<uchar>(maskPoint.y, maskPoint.x) != 0) {
//                     cv::circle(image, cv::Point(point), 2, colors[classId], -1);
//                 }
//             }
//         }

//         cv::rectangle(image, result.box, colors[classId], 2);
//         cv::rectangle(image,
//                       cv::Point(x, y - size.height - 3), cv::Point(x + size.width, y + baseline),
//                       colors[classId], cv::FILLED);
//         cv::putText(image, label,
//                     cv::Point(x, y - 3), cv::FONT_HERSHEY_COMPLEX,
//                     0.5, cv::Scalar(0, 0, 0), 1);
//     }
//     cv::addWeighted(im, 0.4, image, 0.6, 0, im);
// }



void utils_my::letterbox(const cv::Mat &image, cv::Mat &outImage,
                      const cv::Size &newShape = cv::Size(640, 640),
                      const cv::Scalar &color = cv::Scalar(114, 114, 114),
                      bool auto_ = true,
                      bool scaleFill = false,
                      bool scaleUp = true,
                      int stride = 32)
{
    cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);

    float ratio[2]{r, r};
    int newUnpad[2]{(int)std::round((float)shape.width * r),
                    (int)std::round((float)shape.height * r)};

    auto dw = (float)(newShape.width - newUnpad[0]);
    auto dh = (float)(newShape.height - newUnpad[1]);

    if (auto_)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        newUnpad[0] = newShape.width;
        newUnpad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f;
    dh /= 2.0f;

    if (shape.width != newUnpad[0] && shape.height != newUnpad[1])
    {
        cv::resize(image, outImage, cv::Size(newUnpad[0], newUnpad[1]));
    }

    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

void utils_my::scaleCoords(cv::Rect &coords,
                        cv::Mat &mask,
                        const float maskThreshold,
                        const cv::Size &imageShape,
                        const cv::Size &imageOriginalShape)
{
    float gain = std::min((float)imageShape.height / (float)imageOriginalShape.height,
                          (float)imageShape.width / (float)imageOriginalShape.width);

    int pad[2] = {(int)(((float)imageShape.width - (float)imageOriginalShape.width * gain) / 2.0f),
                  (int)(((float)imageShape.height - (float)imageOriginalShape.height * gain) / 2.0f)};

    coords.x = (int)std::round(((float)(coords.x - pad[0]) / gain));
    coords.x = std::max(0, coords.x);
    coords.y = (int)std::round(((float)(coords.y - pad[1]) / gain));
    coords.y = std::max(0, coords.y);

    coords.width = (int)std::round(((float)coords.width / gain));
    coords.width = std::min(coords.width, imageOriginalShape.width - coords.x);
    coords.height = (int)std::round(((float)coords.height / gain));
    coords.height = std::min(coords.height, imageOriginalShape.height - coords.y);
    mask = mask(cv::Rect(pad[0], pad[1], imageShape.width - 2 * pad[0], imageShape.height - 2 * pad[1]));

    cv::resize(mask, mask, imageOriginalShape, cv::INTER_LINEAR);

    mask = mask(coords) > maskThreshold;
}
template <typename T>
T utils_my::clip(const T &n, const T &lower, const T &upper)
{
    return std::max(lower, std::min(n, upper));
}
