import numpy as np
from math import sqrt, ceil

class Drone():
    __position : np.array
    __rotation : np.array
    speed : float # m/fr

    __trajectory : np.array

    def __init__(self, start_position : np.array, speed : float, start_rotation : np.array = np.zeros(3)):
        self.__speed = speed
        self.__position = start_position
        self.__rotation = start_rotation
        self.__trajectory = np.array([start_position])
    

    def get_trajectory(self):
        return self.__trajectory
    

    def fly_forward(self, move_vector : np.array):
        yaw, pitch, roll = np.radians(self.__rotation)
        
        rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        r = rz @ ry @ rx
        
        rotated_vector = r @ move_vector
        
        frame_count = ceil(sqrt(sum(move_vector ** 2)) / self.__speed)

        trajectory = np.zeros((frame_count, 3))
        trajectory[-1] = self.__position + rotated_vector

        rotated_vector /= frame_count

        for i in range(1, frame_count):
            trajectory[i - 1] = self.__position + rotated_vector / frame_count * i
        
        self.__trajectory = np.vstack((self.__trajectory, trajectory))
        self.__position = self.__trajectory[-1]


    def rotate(self, rotate_vector : np.array):
        for i in range(3):
            self.__rotation[i] += rotate_vector[i]
            self.__rotation[i] %= 360


    def fly_line(self, target_pos : np.array):
        move_vector = target_pos - self.__position
        frame_count = ceil(sqrt(sum(move_vector ** 2)) / self.__speed)
        trajectory = np.zeros((frame_count, 3))

        trajectory[-1] = self.__position + move_vector

        for i in range(1, frame_count):
            trajectory[i - 1] = self.__position + move_vector / frame_count * i

        self.__trajectory = np.vstack((self.__trajectory, trajectory))
        self.__position = self.__trajectory[-1]
