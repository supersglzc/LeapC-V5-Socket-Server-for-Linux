#!/usr/bin/env python3
# ipc_client.py

import socket
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from utils import *


HOST = 'localhost'  # The server's hostname or IP address
PORT = 8080        # The port used by the server

fps_list = np.zeros(1000)


def parse_info(data):
    info = {}
    
    info['palm_direction'] = data[:3]
    info['palm_normal'] = data[3:6]
    info['palm_position'] = data[6:9]

    finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
    bone_names = ['distal', 'metacarpal', 'proximal', 'intermediate']

    for i in range(5):
        info[finger_names[i]] = {}
        for j in range(4):
            info[finger_names[i]][bone_names[j]] = data[9+i*16+j*4:13+i*16+j*4]
    
    return info


idx = {
    'thumb': [0, 0, 0, 0],
    'index': [0, 0, 0, 0],
    'middle': [0, 0, 0, 0],
    'ring': [0, 0, 0, 0],
    'pinky': [0, 0, 0, 0],
}


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    fps = 0.0
    step = 0
    while True:
        t1 = time.time()
        
        data = s.recv(2048)
        if not data:
            break
        try:
            received_arr = np.frombuffer(data, dtype=np.float32)
            # print('Received', repr(received_arr), received_arr.shape)
            t2 = time.time()
            fps = 1/(t2-t1)

            info = parse_info(received_arr)
            angles = {}
            finger_order = ['thumb', 'index', 'middle', 'ring', 'pinky']
            bone_order = ['metacarpal', 'proximal', 'intermediate', 'distal']
            for i in range(5):
                angles[finger_order[i]] = []
                for j in range(4):
                    if j == 0:
                        # last_bone = hand_info
                        last_bone_r = R.from_quat([[0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]])
                        last_bone_mat = last_bone_r.as_matrix()
                    else:
                        last_bone = bone_order[j-1]
                        last_bone_r = R.from_quat(info[finger_order[i]][last_bone])
                        last_bone_mat = last_bone_r.as_matrix()

                    bone = bone_order[j]
                    bone_r = R.from_quat(info[finger_order[i]][bone])
                    bone_mat = bone_r.as_matrix()

                    # Get rotation matrix between bones, change of basis
                    rot_mat = np.matmul(
                        bone_mat, last_bone_mat.transpose()
                        )

                    # Generate euler angles in degrees from rotation matrix
                    angles[finger_order[i]].append(get_angles_from_rot(rot_mat))

            print(np.rad2deg(angles['ring']))

        except Exception as e:
            print(e)
        fps_list = np.roll(fps_list, -1)
        fps_list[-1] = fps

        #print('FPS:%f(%.2f/%.2f)'%(np.mean(fps_list),fps,np.cov(fps_list)))
        if len(np.where(fps_list==0.0)[0])>10:
            fps_print = fps
        else:
            fps_print = np.mean(fps_list)
        print('FPS:%.0f'%(fps_print))
        
        step += 1
