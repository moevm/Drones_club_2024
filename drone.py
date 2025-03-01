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
        print(self.__trajectory)
        return self.__trajectory
    

    def fly_forward(self, move_vector : np.array):
        pass


    def fly_line(self, target_pos : np.array):
        move_vector = target_pos - self.__position
        frame_count = ceil(sqrt(sum(move_vector ** 2)) / self.__speed)
        trajectory = np.zeros((frame_count, 3))

        for i in range(1, frame_count + 1):
            trajectory[i - 1] = self.__position + move_vector / frame_count * i

        self.__trajectory = np.vstack((self.__trajectory, trajectory))
