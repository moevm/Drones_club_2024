import time
import argparse
import numpy as np
from our_env.TagDetector import AprilTagDetector
from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from our_env.AutoAviary import AutoAviary
from our_env.Map import Route
from our_env.AutoAviary import *

DEFAULT_DRONE = DroneModel('cf2x')
DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = True
DEFAULT_SIMULATION_FREQ_HZ = 120
DEFAULT_CONTROL_FREQ_HZ = 24

DEFAULT_OUTPUT_FOLDER = 'my_results'
DEFAULT_COLAB = True
DEFAULT_VISION_ATTR = True

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def run(
        drone=DEFAULT_DRONE,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VIDEO,
        vision_attributes=DEFAULT_VISION_ATTR,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        plot=True,
        colab=DEFAULT_COLAB
):
    
    x0, y0, z0 = 0, 0, .1
    INIT_XYZS = np.array([[x0, y0, z0]])

    env = AutoAviary(drone_model=drone,
                     num_drones=1,
                     initial_xyzs=INIT_XYZS,
                     physics=Physics.PYB_GND,
                     neighbourhood_radius=10,
                     pyb_freq=simulation_freq_hz,
                     ctrl_freq=control_freq_hz,
                     gui=gui,
                     record=record_video,
                     obstacles=True,
                     vision_attributes=vision_attributes)

    my_route = Route()
    
    # Добавляем три точки с координатами и углами RPY
    my_route.add_point([0.0, 0.0, 0.1], [0.0, 0.0, 0.0])   # Стартовая точка

    my_route.add_point([0.0, 0.0, 2.0], [0.0, 0.0, 0.0])
    my_route.add_point([0.0, 4, 2.0], [0.0, 0.0, 1.57], True)       
    
    my_route.add_point([7, 6.0, 2.0], [0.0, 0.0, 1.57])
    my_route.add_point([6, 10, 2.0], [0.0, 0.0, 3.14], True) ## 2_cube

    my_route.add_point([7, 15, 2.0], [0.0, 0.0, 3.14])
    my_route.add_point([0.0, 16, 2.0], [0.0, 0.0, 4.71], True) ## 3_cube
    
    my_route.add_point([-7, 15.0, 2.0], [0.0, 0.0, 4.71])
    my_route.add_point([-6, 10, 2.0], [0.0, 0.0, 6.28], True) 
    my_route.add_point([-7, 6.0, 2.0], [0.0, 0.0, 6.28])
    
    
    my_route.add_point([0.0, 0.0, 0.1], [0.0, 0.0, 0]) ## go_home
    

    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=1,
                    output_folder=output_folder,
                    colab=colab)

    ctrl = [DSLPIDControl(drone_model=drone)]
   
    action = np.zeros((1, 4))
    
    START = time.time()
    i = 0
    current_orientation = [0.0, 0.0, 0.0]
    # Добавьте эти параметры в функцию run
    MOVE_STEP = 1.0  # Шаг перемещения (метры)
    ROTATE_STEP = 0.05# Шаг поворота (радианы)
    new_orientation = [0.0, 0.0, 0.0]
    # Состояние дрона
    state = "moving_to_target"  # Начальное состояние
    while True:
        
        obs, reward, terminated, truncated, info = env.step(action)
        current_position = obs[0][:3]
        if i: 
            current_orientation = new_orientation[:]  

        target_pos = my_route.get_current_point().coordinates.copy()
        target_orientation = my_route.get_current_point().orientation.copy()

        new_orientation = np.copy(current_orientation)  # Инициализация new_orientation по умолчанию
        print(state)
        if state == "moving_to_target":
            # Проверяем достижение текущей точки маршрута.
            if np.linalg.norm(current_position - target_pos) < .2:
                state = "rotating_to_orientation"  # Переход к следующему состоянию

            # Вычисляем направление к целевой позиции
            direction = target_pos - current_position
            distance_to_target = np.linalg.norm(direction)

            if distance_to_target > MOVE_STEP:
                direction_normalized = direction / distance_to_target
                new_position = current_position + direction_normalized * MOVE_STEP
            else:
                new_position = target_pos  # Достигли цели

        elif state == "rotating_to_orientation":
            print(state)
            # Вычисляем необходимый угол поворота
            desired_orientation = target_orientation
            angle_diff = normalize_angle(desired_orientation - current_orientation)

            # Обновляем ориентацию, приближаясь к нужному углу
            for j in range(len(current_orientation)):
                if abs(angle_diff[j]) > ROTATE_STEP:
                    new_orientation[j] = current_orientation[j] + np.sign(angle_diff[j]) * ROTATE_STEP
                else:
                    new_orientation[j] = desired_orientation[j]

            # Проверяем достигли ли нужного угла
            if all(abs(angle_diff) <= ROTATE_STEP):
                if my_route.get_current_point().expecting_tag:
                    state = "examining_tag" # изучаем тег
                else:
                    state = "moving_to_next_point"  # Переход к следующему состоянию


        elif state == "moving_to_next_point":
            my_route.next_point()  # Переход к следующей точке маршрута.
            if my_route.ended:
                break

            state = "moving_to_target"  # Возвращаемся в начальное состояние

        elif state == "examining_tag":
            if (current_orientation[2] >= -np.pi/4 and current_orientation[2] < np.pi/4) or \
            (current_orientation[2] >= 7*np.pi/4 or current_orientation[2] < -7*np.pi/4):
                # Дрон смотрит вдоль оси X
                backward_direction = np.array([-1, 0, 0])  # В противоположную сторону по X
            elif (current_orientation[2] >= np.pi/4 and current_orientation[2] < 3*np.pi/4):
                # Дрон смотрит вдоль оси Y
                backward_direction = np.array([0, -1, 0])  # В противоположную сторону по Y
            elif (current_orientation[2] >= 3*np.pi/4 and current_orientation[2] < 5*np.pi/4):
                # Дрон смотрит обратно вдоль оси X
                backward_direction = np.array([1, 0, 0])  # В противоположную сторону по X
            else:
                # Дрон смотрит обратно вдоль оси Y
                backward_direction = np.array([0, 1, 0])  # В противоположную сторону по Y
            #backward_direction = np.array([-np.sin(current_orientation[2]), np.cos(current_orientation[2]), 0])
            #side_direction = np.array([np.cos(current_orientation[2]), np.sin(current_orientation[2]), 0])
            print("="*20)
            if env.tag_of_cube is None:
                print("-"*20)
                print("Backward direction:", backward_direction)
                #print("Side direction:", side_direction)
                new_position = current_position + backward_direction * MOVE_STEP
            else:
                # здесь можно добавить логику если мы полностью увидели тег
                state = "moving_to_next_point"

        current_orientation = new_orientation[:]

        print(current_orientation, new_orientation)
        print(env.tag_of_cube is not None, '*' * 30)
        action[0], _, _ = ctrl[0].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                            state=obs[0],
                                                            target_pos=new_position,
                                                            target_rpy=new_orientation)

        for j in range(1):
            logger.log(drone=j,
                        timestamp=i/env.CTRL_FREQ,
                        state=obs[j])

        env.render()

        if gui:
            sync(i % env.CTRL_FREQ , START , env.CTRL_TIMESTEP)

        i += 1

        if time.time() - START >= 30000:
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='TBD')
    parser.add_argument('--drone',              default=DEFAULT_DRONE,type=DroneModel , help='Drone model', metavar='', choices=list(DroneModel))
    parser.add_argument('--gui',                default=DEFAULT_GUI,type=str2bool , help='Whether to use PyBullet GUI', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,type=str2bool , help='Whether to record a video', metavar='')
    parser.add_argument('--vision_attributes',   default=DEFAULT_VISION_ATTR,type=str2bool , help='Whether to record a video from drone', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,type=int , help='Simulation frequency in Hz', metavar='')
    parser.add_argument('--control_freq_hz',     default=DEFAULT_CONTROL_FREQ_HZ,type=int , help='Control frequency in Hz', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER,type=str , help='Folder where to save logs', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB,type=str , help='Whether example is being run by a notebook', metavar='')
   
    ARGS = parser.parse_args()
   
    run(**vars(ARGS))
