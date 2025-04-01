import cv2
import apriltag

class AprilTagDetector:
    def __init__(self):
        options = apriltag.DetectorOptions(families="tag36h10")
        self.detector = apriltag.Detector(options)

    def detect_tags(self, image):
        # Преобразование изображения в градации серого
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray_image)
        return tags

    def highlight_tags(self, image, tags):
        # Выделяем обнаруженные теги на изображении зеленым квадратом
        for tag in tags:
            # Получаем координаты углов тега
            corners = tag.corners.astype(int)
            # Рисуем квадрат вокруг тега
            cv2.rectangle(image, (corners[0][0], corners[0][1]), 
                          (corners[2][0], corners[2][1]), (0, 255, 0, 255), thickness=2)

            # Отображаем ID тега
            cv2.putText(image, str(tag.tag_id), (corners[0][0], corners[0][1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 255), 2)

        return image

