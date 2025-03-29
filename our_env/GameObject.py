import pybullet as p
import os

class GameObject:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    models_path = SCRIPT_DIR + '/3d_models/'

    position : list[3]
    rotation : list[3]
    scale : list[3] | float
    __objectId : any
    __model_name : str

    def __init__(self, model_name, position : list[3] = [0, 0, 0], rotation : list[3] = [0, 0, 0], scale : list[3] | float = 1) -> None:
   
        self.__objectId = p.loadURDF(self.models_path + model_name,
                                   position,
                                   p.getQuaternionFromEuler(rotation),
                                   globalScaling = scale)

        self.position = position
        self.rotation = rotation
        self.scale = scale
        self.__model_name = model_name
    

    def SetPosition(self, position : list[3]) -> None:
        self.position = position

        p.resetBasePositionAndOrientation(self.__objectId, position, p.getQuaternionFromEuler(self.rotation))

    
    def SetRotation(self, rotation : list[3]) -> None:
        self.rotation = rotation

        p.resetBasePositionAndOrientation(self.__objectId, self.position, p.getQuaternionFromEuler(rotation))

    
    def SetScale(self, scale : list[3] | float) -> None:
        self.scale = scale

        p.removeBody(self.__objectId)

        self.__objectId = p.loadURDF(self.models_path + self.__model_name,
                                   self.position,
                                   p.getQuaternionFromEuler(self.rotation),
                                   globalScaling = scale)


    def SetPositionAndRotation(self, position : list[3], rotation : list[3]) -> None:
        self.position = position
        self.rotation = rotation

        p.resetBasePositionAndOrientation(self.__objectId, position, p.getQuaternionFromEuler(rotation))


    def Move(self, moveVector : list[3]) -> None:
        for i in range(3):
            self.position[i] += moveVector[i]

        self.SetPosition(self.position)


    def Rotate(self, rotateVector : list[3]) -> None:
        for i in range(3):
            self.rotation[i] += rotateVector[i]
            if self.rotation[i] < 0:
                self.rotation[i] += 360
            elif self.rotation[i] >= 360:
                self.rotation[i] -= 360
        
        self.SetRotation(self.rotation)


    def Destroy(self) -> None:
        p.removeBody(self.__objectId)

