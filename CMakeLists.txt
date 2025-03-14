cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(YOLOv8SegWithPCL)



# 设置 ONNXRUNTIME_DIR 变量
set(ONNXRUNTIME_DIR "/data/onnxruntime-linux-x64-gpu-1.15.1" CACHE PATH "Path to ONNX Runtime installation")
message(STATUS "ONNXRUNTIME_DIR: ${ONNXRUNTIME_DIR}")


# 查找PCL和OpenCV库
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# 添加头文件和源文件目录
include_directories(
    ${PCL_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} 
    ${ONNXRUNTIME_INCLUDE_DIR}  # 添加 ONNX Runtime 的头文件路径
    ${CMAKE_SOURCE_DIR}/include
)

# 添加库文件路径
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${ONNXRUNTIME_ROOT}/lib)  # 添加 ONNX Runtime 的库文件路径

# 添加可执行文件
add_executable(${PROJECT_NAME} 
    src/utils_my.cpp
    src/main.cpp 
    src/yolo8_seg_inference.cpp 
    src/LiDARtoCamera.cpp
)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 链接PCL、OpenCV和ONNX Runtime库
target_link_libraries(${PROJECT_NAME} 
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${ONNXRUNTIME_LIB}  # 添加 ONNX Runtime 的库
)


# 添加 ONNX Runtime 头文件搜索路径
target_include_directories(YOLOv8SegWithPCL PRIVATE "${ONNXRUNTIME_DIR}/include")

if (UNIX)
    target_link_libraries(YOLOv8SegWithPCL "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
endif(UNIX)