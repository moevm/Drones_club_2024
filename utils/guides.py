import numpy as np

def go_back(tmp_pos, tmp_rpy, delta):
    target_pos = tmp_pos.copy()
    target_pos[0] += delta * np.cos(tmp_rpy[2])
    target_pos[1] -= delta * np.sin(tmp_rpy[2])
    return target_pos

def go_forward(tmp_pos, tmp_rpy, delta):
    target_pos = tmp_pos.copy()
    target_pos[0] -= delta * np.cos(tmp_rpy[2])
    target_pos[1] += delta * np.sin(tmp_rpy[2])
    return target_pos

def go_left(tmp_pos, tmp_rpy, delta):
    target_pos = tmp_pos.copy()
    target_pos[1] += delta * np.cos(tmp_rpy[2])
    target_pos[0] -= delta * np.sin(tmp_rpy[2])
    return target_pos

def go_right(tmp_pos, tmp_rpy, delta):
    target_pos = tmp_pos.copy()
    target_pos[1] -= delta * np.cos(tmp_rpy[2])
    target_pos[0] += delta * np.sin(tmp_rpy[2])
    return target_pos

def clockwise(tmp_rpy, delta):
    target_rpy = tmp_rpy.copy()
    target_rpy[2] -= delta
    return target_rpy

def counterclockwise(tmp_rpy, delta):
    target_rpy = tmp_rpy.copy()
    target_rpy[2] += delta
    return target_rpy

def go_down(tmp_pos, delta):
    target_pos = tmp_pos.copy()
    if target_pos[2] > 0.01:
        target_pos[2] -= delta
    return target_pos

def go_up(tmp_pos, delta):
    target_pos = tmp_pos.copy()
    target_pos[2] += delta
    return target_pos
