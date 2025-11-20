from ultralytics import YOLO

model = YOLO("/home/deepet/VSCode/xyz_ws/src/doosan-robot2/dsr_example2/dsr_example/dsr_example/weights/new_best_model.pt")
model.export(format="onnx", opset=12, dynamic=False)
