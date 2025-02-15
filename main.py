import time
import argparse
import numpy as np

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

DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'my_results'
DEFAULT_COLAB = True
DEF_VISION_ATTR = True

def run(
        drone=DEFAULT_DRONE, 
        gui=DEFAULT_GUI, 
        record_video=DEFAULT_RECORD_VIDEO, 
        vision_attributes = DEF_VISION_ATTR,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ, 
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ, 
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        plot=True,
        colab=DEFAULT_COLAB
    ):
    #### Initialize the simulation #############################

    PERIOD = duration_sec
    NUM_WP = control_freq_hz*PERIOD 


    '''
    Место, которое нелюходимо поменять
    Начало

    x0, y0, z0 - точка спавна дрона
    x1, y1, z1 - точка окончания полета

    INIT_XYZS - вектор начального положения дрона

    TARGET_POS - вся траектория дрона

    NUM_WP - количество шагов симуляции
    '''
   
    x0 = 0
    y0 = 0
    z0 = .1

    INIT_XYZS = np.array([[x0, y0, z0]])

    x1 = 2
    y1 = 1
    z1 = 4

    next_POS = np.zeros((int(NUM_WP/2), 3)) 

    for i in range(int(NUM_WP/2)):
        j = (i+1)/(NUM_WP/2)
        next_POS[i, :] = (INIT_XYZS[0, 0] + (x1-x0)*j, INIT_XYZS[0, 1] + (y1-y0)*j, INIT_XYZS[0, 2] + (z1-z0)*j)
        print(next_POS[i, :])
    
    x2 = -3
    y2 = -2
    z2 = 2 
 
    newt2_POS = np.zeros((int(NUM_WP/2), 3)) 

    for i in range(int(NUM_WP/2)):
        k = (i+1)/(NUM_WP/2)
        newt2_POS [i, :] = (next_POS[int(NUM_WP/2)-1, 0] + (x2-x1)*k, next_POS[int(NUM_WP/2)-1, 1] + (y2-y1)*k, next_POS[int(NUM_WP/2)-1, 2] + (z2-z1)*k)

    TARGET_POS = np.vstack((next_POS, newt2_POS,))

    print('\n'*10, TARGET_POS, '\n'*10)
    


    # TODO env!!!!
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

    wp_counters = [0]
    wp_counter = 0


    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=1,
                    duration_sec=duration_sec,
                    output_folder=output_folder,
                    colab=colab
                    )


    ctrl = [DSLPIDControl(drone_model=drone)]
 

    action = np.zeros((1,4))
    START = time.time()
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)
      
        action[0], _, _ = ctrl[0].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                             state=obs[0],
                                                             target_pos=TARGET_POS[wp_counter, :],
                                                             )
       
        if wp_counter < NUM_WP - 1:
            wp_counter = wp_counter + 1         
        else:
            wp_counter = 0 

        for j in range(1):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j]
                    #    control=np.hstack([TARGET_POS[wp_counter, :], INIT_XYZS[j ,2], np.zeros(9)])
                       )

        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    if plot:
        logger.plot()


if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='TBD')
    parser.add_argument('--drone',              default=DEFAULT_DRONE,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--vision_attributes',  default=DEF_VISION_ATTR,      type=str2bool,      help='Whether to record a video frome drone (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 10)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))

