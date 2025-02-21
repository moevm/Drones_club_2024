import pybullet as p
import os

class GameObject:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    models_path = os.path.dirname(SCRIPT_DIR) + '/our_env/3d_models/'

    __position : list[3]
    __rotation : list[3]

    def __init__(self, model_name, position : list[3] = [0, 0, 0], rotation : list[3] = [0, 0, 0]):
        self.objectId = p.loadURDF(self.models_path + model_name,
                                   position,
                                   p.getQuaternionFromEuler(rotation))

        self.__position = position
        self.__rotation = rotation
    

    def SetPosition(self, position : list[3]):
        self.__position = position

