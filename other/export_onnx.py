#使用 Python 脚本导出模型
from ultralytics import YOLO
model = YOLO("yolov8n-seg.pt")
model.export(format="onnx", opset=12, imgsz=640)
