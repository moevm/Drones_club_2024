
import numpy as np


class Point:
    def __init__(self, coordinates, orientation):
        self.coordinates = np.array(coordinates)
        self.orientation = np.array(orientation)  # Добавляем ориентацию

class Route:
    def __init__(self):
        self.points = []
        self.current_index = 0

    def add_point(self, coordinates, orientation):
        self.points.append(Point(coordinates, orientation))

    def get_current_point(self):
        return self.points[self.current_index]

    def next_point(self):
        if self.current_index < len(self.points) - 1:
            print("___________________________________")
            self.current_index += 1

    def is_last_point(self):
        return self.current_index == len(self.points) - 1