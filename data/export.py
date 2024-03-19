from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n-pose.pt')  # load an official model
#model = YOLO('yolov8n.pt')  # load an official model

# Export the model
# Export the model
model.export(format='onnx', imgsz=[480,640], opset=12)  # 640 x 480
#model.export(format='onnx',imgsz=[640,640], opset=12)
