
from TagDetector import AprilTagDetector
from PIL import Image
import numpy as np
import cv2

tag_detector = AprilTagDetector()
image_path = 'frame_16.png'  # Укажите путь к вашему изображению
image = cv2.imread(image_path)

# Преобразуем изображение в массив NumPy
image_array = np.array(image)
print(tag_detector.detect_tags(image))
