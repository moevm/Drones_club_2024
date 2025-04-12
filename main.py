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

DEFAULT_DRONE = DroneModel('cf2x')
DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48

DEFAULT_OUTPUT_FOLDER = 'my_results'
DEFAULT_COLAB = True
DEF_VISION_ATTR = True

def run(
        drone=DEFAULT_DRONE,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VIDEO,
        vision_attributes=DEF_VISION_ATTR,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        plot=True,
        colab=DEFAULT_COLAB
):
    
    # Начальные параметры для дрона.
    x0, y0, z0 = 0, 0, .1
    INIT_XYZS = np.array([[x0, y0, z0]])

    env = AutoAviary(drone_model=drone,
                     num_drones=1,
                     initial_xyzs=INIT_XYZS,
                     neighbourhood_radius=10,
                     pyb_freq=simulation_freq_hz,
                     ctrl_freq=control_freq_hz,
                     gui=gui,
                     record=record_video,
                     obstacles=True,
                     vision_attributes=vision_attributes)

    
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=1,
                    output_folder=output_folder,
                    colab=colab)

    
    ctrl = [DSLPIDControl(drone_model=drone)]
   
   # Начальные значения для управления движением.
    action = np.zeros((1, 4))
    current_position = INIT_XYZS[0].copy()
    current_orientation = np.array([0.0, 0.0, 0.0])  # roll-pitch-yaw (r-p-y)
    height_target_reached = False
   
    START = time.time()

    i = 0  # Инициализируем счетчик времени

    while True:  # Бесконечный цикл симуляции.
        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        # Получаем текущее состояние дрона.
        current_position = obs[0][:3]
        current_orientation = obs[0][3:6]

        if not height_target_reached:
            # Двигаемся вверх до высоты 3.
            if current_position[2] < 3:
                target_pos = [current_position[0], current_position[1], 3]  # Целевая позиция по Z - 3 метра
            else:
                height_target_reached = True

        elif height_target_reached and current_position[2] >= 3:
            # Двигаемся вперед на 5 метров.
            if current_position[0] < 5: 
                target_pos = [5, current_position[1], current_position[2]]  # Целевая позиция по X - 5 метров вперед
            else:
                target_pos = [current_position[0], current_position[1], current_position[2]] 

        if env.tag_of_cube:
            print("&&&&&&&&&&&&&&&&&&&&")
            # Если дрон видит тег - возвращаемся на координаты (0, 0, 0).
            target_pos[:] = [x0, y0, z0]
            height_target_reached = False

        # Вычисляем действия с помощью контроллера.
        action[0], _, _ = ctrl[0].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                           state=obs[0],
                                                           target_pos=target_pos,
                                                           target_rpy=current_orientation)  

        for j in range(1):
            logger.log(drone=j,
                        timestamp=i/env.CTRL_FREQ,
                        state=obs[j])

        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i % env.CTRL_FREQ , START , env.CTRL_TIMESTEP)

        i += 1

   #### Close the environment #################################
    env.close()

    if plot:
        logger.plot()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='TBD')
    parser.add_argument('--drone',              default=DEFAULT_DRONE,type=DroneModel , help='Drone model', metavar='', choices=list(DroneModel))
    parser.add_argument('--gui',                default=DEFAULT_GUI,type=str2bool , help='Whether to use PyBullet GUI', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,type=str2bool , help='Whether to record a video', metavar='')
    parser.add_argument('--vision_attributes',   default=DEF_VISION_ATTR,type=str2bool , help='Whether to record a video from drone', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,type=int , help='Simulation frequency in Hz', metavar='')
    parser.add_argument('--control_freq_hz',     default=DEFAULT_CONTROL_FREQ_HZ,type=int , help='Control frequency in Hz', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER,type=str , help='Folder where to save logs', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB,type=str , help='Whether example is being run by a notebook', metavar='')
   
    ARGS = parser.parse_args()
   
    run(**vars(ARGS))