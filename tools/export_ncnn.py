"""Run once on the RPi5 to export YOLOv8n to NCNN format.

Output: yolov8n_ncnn_model/ in the current directory.
Verify with: ls yolov8n_ncnn_model/  (should show .param and .bin files)
"""
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
model.export(format='ncnn')
print("Export complete. Verify: ls yolov8n_ncnn_model/")
