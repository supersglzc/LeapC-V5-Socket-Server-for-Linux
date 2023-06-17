#!/usr/bin/env python3
# ipc_client.py

import socket
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from utils import *
import pybullet
import imageio
import os
np.set_printoptions(precision=3, suppress=True)

milliseconds = lambda: int(time.time() * 1000)
t_ = milliseconds()
images = []
# camera_view_matrix = pybullet.computeViewMatrix(cameraEyePosition=[0.25, -0.75, 0.5],
#                                                                  cameraTargetPosition=[0., 0., 0.2],
#                                                                  cameraUpVector=[0., 0., 1.])
# camera_projection_matrix = pybullet.computeProjectionMatrixFOV(fov=45., aspect=1., nearVal=0.1,
#                                                                                 farVal=1.1)
physics_client = pybullet.connect(pybullet.GUI)
shadow_hand_right_urdf = os.path.join("assets", "urdf", "shadow_hand_right.urdf")
shadow_hand_right_id = pybullet.loadURDF(fileName=shadow_hand_right_urdf,
                                                      basePosition=[0., 0., 0.],
                                                      baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                                                      flags=pybullet.URDF_USE_SELF_COLLISION)
shadow_hand_right_num_joints = pybullet.getNumJoints(shadow_hand_right_id)
shadow_hand_right_target_pos = [0.] * shadow_hand_right_num_joints
# Set sub-step-parameter and start Real-Time-Simulation
pybullet.setPhysicsEngineParameter(numSubSteps=0)
pybullet.setRealTimeSimulation(1)
t = 0


HOST = 'localhost'  # The server's hostname or IP address
PORT = 8080        # The port used by the server

fps_list = np.zeros(1000)


def parse_info(data):
    info = {}
    print(data.shape)
    info['palm_direction'] = data[:3]
    info['palm_normal'] = data[3:6]
    info['palm_position'] = data[6:9]
    info['palm_orientation'] = data[9:13]

    finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
    bone_names = ['distal', 'metacarpal', 'proximal', 'intermediate']

    for i in range(5):
        info[finger_names[i]] = {}
        start = 13+i*40
        for j in range(4):
            info[finger_names[i]][bone_names[j]] = {}
            info[finger_names[i]][bone_names[j]]['rotation'] = data[start+j*4:start+j*4+4]
        for j in range(4):
            info[finger_names[i]][bone_names[j]]['next_joint'] = data[start+16+j*3:start+19+j*3]
        for j in range(4):
            info[finger_names[i]][bone_names[j]]['prev_joint'] = data[start+28+j*3:start+31+j*3]

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
        
        data = s.recv(4096)
        if not data:
            break

        received_arr = np.frombuffer(data, dtype=np.float32)
        # print(received_arr.shape)
        if received_arr.shape[0] != 213:
            print("Invalid frame!")
            continue
        t2 = time.time()
        fps = 1/(t2-t1)

        info = parse_info(received_arr)
        # angles = {}
        # finger_order = ['thumb', 'index', 'middle', 'ring', 'pinky']
        # bone_order = ['metacarpal', 'proximal', 'intermediate', 'distal']
        # for i in range(5):
        #     angles[finger_order[i]] = []
        #     for j in range(4):
        #         if j == 0:
        #             # last_bone = hand_info
        #             last_bone_r = R.from_quat([[0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]])
        #             last_bone_mat = last_bone_r.as_matrix()
        #         else:
        #             last_bone = bone_order[j-1]
        #             last_bone_r = R.from_quat(info[finger_order[i]][last_bone]['rotation'])
        #             last_bone_mat = last_bone_r.as_matrix()

        #         bone = bone_order[j]
        #         bone_r = R.from_quat(info[finger_order[i]][bone]['rotation'])
        #         bone_mat = bone_r.as_matrix()

        #         # Get rotation matrix between bones, change of basis
        #         rot_mat = np.matmul(
        #             bone_mat, last_bone_mat.transpose()
        #             )

        #         # Generate euler angles in degrees from rotation matrix
        #         angles[finger_order[i]].append(get_angles_from_rot(rot_mat))

        # index
        theta1, theta2, theta3, theta4 = finger_joint_rotations(info['index'], ifprint=True)
        # print(np.rad2deg([theta1, theta2, theta3, theta4]))
        if not np.isnan(theta1) and not np.isnan(theta2) and not np.isnan(theta3) and not np.isnan(theta4):
            shadow_hand_right_target_pos[3:7] = [
                np.clip(theta1, -0.349, 0.349),
                np.clip(theta2, 0., 1.571),
                np.clip(theta3, 0., 1.571),
                np.clip(theta4, 0., 1.571)]
                        
        # middle
        theta1, theta2, theta3, theta4 = finger_joint_rotations(info['middle'])
        if not np.isnan(theta1) and not np.isnan(theta2) and not np.isnan(theta3) and not np.isnan(theta4):
            shadow_hand_right_target_pos[8:12] = [
                np.clip(theta1, -0.349, 0.349),
                np.clip(theta2, 0., 1.571),
                np.clip(theta3, 0., 1.571),
                np.clip(theta4, 0., 1.571)]
            
        # pinky
        theta1, theta2, theta3, theta4 = finger_joint_rotations(info['pinky'])
        if not np.isnan(theta1) and not np.isnan(theta2) and not np.isnan(theta3) and not np.isnan(theta4):
            shadow_hand_right_target_pos[19:23] = [
                -np.clip(theta1, -0.349, 0.349),  # "-" = BUG ?
                np.clip(theta2, 0., 1.571),
                np.clip(theta3, 0., 1.571),
                np.clip(theta4, 0., 1.571)]
            
        # ring
        theta1, theta2, theta3, theta4 = finger_joint_rotations(info['ring'])
        if not np.isnan(theta1) and not np.isnan(theta2) and not np.isnan(theta3) and not np.isnan(theta4):
            shadow_hand_right_target_pos[13:17] = [
                -np.clip(theta1, -0.349, 0.349),  # "-" = BUG ?
                np.clip(theta2, 0., 1.571),
                np.clip(theta3, 0., 1.571),
                np.clip(theta4, 0., 1.571)]
        
        theta1, theta2, theta3, theta4, theta5 = get_thumb(info['thumb'], info['palm_orientation'])
        # print(theta1, theta2, theta3, theta4, theta5)
        if not np.isnan(theta1) and not np.isnan(theta2) and not np.isnan(theta3) and not np.isnan(
                theta4) and not np.isnan(theta5):
            shadow_hand_right_target_pos[24:29] = [
                np.clip(theta1, -1.047, 1.047),
                np.clip(theta2, 0., 1.222),
                np.clip(theta3, -0.209, 0.209),
                np.clip(theta4, -0.524, 0.524),
                np.clip(theta5, 0., 1.571)]

        pybullet.setJointMotorControlArray(bodyUniqueId=shadow_hand_right_id,
                                                    jointIndices=range(shadow_hand_right_num_joints),
                                                    controlMode=pybullet.POSITION_CONTROL,
                                                    targetPositions=shadow_hand_right_target_pos, 
                                                    )

        fps_list = np.roll(fps_list, -1)
        fps_list[-1] = fps

        #print('FPS:%f(%.2f/%.2f)'%(np.mean(fps_list),fps,np.cov(fps_list)))
        if len(np.where(fps_list==0.0)[0])>10:
            fps_print = fps
        else:
            fps_print = np.mean(fps_list)
        print('FPS:%.0f'%(fps_print))
        
        step += 1
