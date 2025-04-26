
import numpy as np


class Point:
    def __init__(self, coordinates, orientation, expecting_tag):
        self.coordinates = np.array(coordinates)
        self.orientation = np.array(orientation)  # Добавляем ориентацию
        self.expecting_tag = expecting_tag

class Route:
    def __init__(self):
        self.points = []
        self.current_index = 0

    def add_point(self, coordinates, orientation, expecting_tag = False):
        self.points.append(Point(coordinates, orientation, expecting_tag))

    def get_current_point(self):
        return self.points[self.current_index]

    def next_point(self):
        if self.current_index < len(self.points) - 1:
            print("___________________________________")
            self.current_index += 1

    def is_last_point(self):
        return self.current_index == len(self.points) - 1