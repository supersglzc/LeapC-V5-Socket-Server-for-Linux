import numpy as np
import math
from scipy.spatial.transform import Rotation as R


def get_angles_from_rot(rot_mat):
    sy = math.sqrt(rot_mat[0, 0] * rot_mat[0, 0] + rot_mat[1, 0] * rot_mat[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(rot_mat[2, 1], rot_mat[2, 2])
        y = math.atan2(-rot_mat[2, 0], sy)
        z = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
    else:
        x = math.atan2(-rot_mat[1, 2], rot_mat[1, 1])
        y = math.atan2(-rot_mat[2, 0], sy)
        z = 0

    return [x,y,z]


def get_rot_from_angles(theta):
    x = np.array([
		[1,         0,                  0                   ],
		[0,         math.cos(theta[0]), -math.sin(theta[0]) ],
		[0,         math.sin(theta[0]), math.cos(theta[0])  ]
	])
    y = np.array([
		[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
		[0,                     1,      0                   ],
		[-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
	])
    z = np.array([
		[math.cos(theta[2]),    -math.sin(theta[2]),    0],
		[math.sin(theta[2]),    math.cos(theta[2]),     0],
		[0,                     0,                      1]
	])
    
    R = np.dot(z, np.dot(y, x))
    return R


def finger_joint_rotations(info, ifprint=False):
    """
    Compute the joint rotations (angles) of the given finger.
    This works for the index, middle, ring and little finger.

    Parameters
    ----------
    finger : Leap.Finger
        Leap.Finger object. Index, Middle, Ring or Little finger.

    Returns
    -------
    theta1 : float
        Joint rotation (angle) of theta1 in radiant.
    theta2 : float
        Joint rotation (angle) of theta2 in radiant.
    theta3 : float
        Joint rotation (angle) of theta3 in radiant.
    theta4 : float
        Joint rotation (angle) of theta4 in radiant.
    """

    # Bases
    # metacarpal_basis = finger.bone(Leap.Bone.TYPE_METACARPAL).basis
    # proximal_basis = finger.bone(Leap.Bone.TYPE_PROXIMAL).basis
    # intermediate_basis = finger.bone(Leap.Bone.TYPE_INTERMEDIATE).basis
    metacarpal_r = R.from_quat(info['metacarpal']['rotation'])
    metacarpal_mat = metacarpal_r.as_matrix()
    proximal_r = R.from_quat(info['proximal']['rotation'])
    proximal_mat = proximal_r.as_matrix()
    intermediate_r = R.from_quat(info['intermediate']['rotation'])
    intermediate_mat = intermediate_r.as_matrix()
    if ifprint:
        # print(metacarpal_mat)
        # print(proximal_mat)
        print(metacarpal_mat @ np.linalg.pinv(proximal_mat))
    # Transformations
    # metacarpal_transform = Leap.Matrix(metacarpal_basis.x_basis,
    #                                    metacarpal_basis.y_basis,
    #                                    metacarpal_basis.z_basis,
    #                                    finger.bone(Leap.Bone.TYPE_METACARPAL).next_joint)
    # metacarpal_transform = metacarpal_transform.rigid_inverse()
    metacarpal_transform = np.asarray([
        [metacarpal_mat[0][0], metacarpal_mat[0][1], metacarpal_mat[0][2], 0],
        [metacarpal_mat[1][0], metacarpal_mat[1][1], metacarpal_mat[1][2], 0],
        [metacarpal_mat[2][0], metacarpal_mat[2][1], metacarpal_mat[2][2], 0],
        [info['metacarpal']['next_joint'][0], info['metacarpal']['next_joint'][1], info['metacarpal']['next_joint'][2], 1.0],
    ])
    metacarpal_transform = np.linalg.pinv(metacarpal_transform)
    # proximal_transform = Leap.Matrix(proximal_basis.x_basis,
    #                                  proximal_basis.y_basis,
    #                                  proximal_basis.z_basis,
    #                                  finger.bone(Leap.Bone.TYPE_PROXIMAL).next_joint)
    # proximal_transform = proximal_transform.rigid_inverse()
    proximal_transform = np.asarray([
        [proximal_mat[0][0], proximal_mat[0][1], proximal_mat[0][2], 0],
        [proximal_mat[1][0], proximal_mat[1][1], proximal_mat[1][2], 0],
        [proximal_mat[2][0], proximal_mat[2][1], proximal_mat[2][2], 0],
        [info['proximal']['next_joint'][0], info['proximal']['next_joint'][1], info['proximal']['next_joint'][2], 1.0],
    ])
    proximal_transform = np.linalg.pinv(proximal_transform)

    # intermediate_transform = Leap.Matrix(intermediate_basis.x_basis,
    #                                      intermediate_basis.y_basis,
    #                                      intermediate_basis.z_basis,
    #                                      finger.bone(Leap.Bone.TYPE_INTERMEDIATE).next_joint)
    # intermediate_transform = intermediate_transform.rigid_inverse()
    intermediate_transform = np.asarray([
        [intermediate_mat[0][0], intermediate_mat[0][1], intermediate_mat[0][2], 0],
        [intermediate_mat[1][0], intermediate_mat[1][1], intermediate_mat[1][2], 0],
        [intermediate_mat[2][0], intermediate_mat[2][1], intermediate_mat[2][2], 0],
        [info['intermediate']['next_joint'][0], info['intermediate']['next_joint'][1], info['intermediate']['next_joint'][2], 1.0],
    ])
    intermediate_transform = np.linalg.pinv(intermediate_transform)

    # End position - "endeffector"
    # end_pos_metacarpal = finger.bone(Leap.Bone.TYPE_PROXIMAL).next_joint

    # Transform end position in current system
    # vec = metacarpal_transform.transform_point(end_pos_metacarpal)
    vec = np.asarray([info['proximal']['next_joint'][0], 
                       info['proximal']['next_joint'][1],
                       info['proximal']['next_joint'][2],
                       1]).dot(metacarpal_transform)
    # from IPython import embed
    # embed()
    if ifprint:
        print(vec)
    # l2 = finger.bone(Leap.Bone.TYPE_PROXIMAL).length
    l2 = np.linalg.norm(
        info['proximal']['next_joint'] - info['proximal']['prev_joint']
    )

    # From inverse kinematics (DH-convention)
    # theta1 = -np.arctan(vec.x / vec.z)
    # theta2 = -np.arcsin(vec.y / l2)
    theta1 = -np.arctan(vec[0] / vec[2])
    theta2 = -np.arcsin(vec[1] / l2)

    # End position - "endeffector"
    # end_pos_proximal = finger.bone(Leap.Bone.TYPE_INTERMEDIATE).next_joint

    # Transform end position in current system
    # vec = proximal_transform.transform_point(end_pos_proximal)
    vec = np.asarray([info['intermediate']['next_joint'][0], 
                       info['intermediate']['next_joint'][1],
                       info['intermediate']['next_joint'][2],
                       1]).dot(proximal_transform)
    # l3 = finger.bone(Leap.Bone.TYPE_INTERMEDIATE).length
    l3 = np.linalg.norm(
        info['intermediate']['next_joint'] - info['intermediate']['prev_joint']
    )

    # From inverse kinematics (DH-convention)
    # theta3 = -np.arcsin(vec.y / l3)
    theta3 = -np.arcsin(vec[1] / l3)

    # Transform end position in current system
    # end_pos_intermediate = finger.bone(Leap.Bone.TYPE_DISTAL).next_joint

    # Transform end position in current system
    # vec = intermediate_transform.transform_point(end_pos_intermediate)
    vec = np.asarray([info['distal']['next_joint'][0], 
                       info['distal']['next_joint'][1],
                       info['distal']['next_joint'][2],
                       1]).dot(intermediate_transform)
    # l4 = finger.bone(Leap.Bone.TYPE_DISTAL).length
    l4 = np.linalg.norm(
        info['distal']['next_joint'] - info['distal']['prev_joint']
    )

    # From inverse kinematics (DH-convention)
    # theta4 = -np.arcsin(vec.y / l4)
    theta4 = -np.arcsin(vec[1] / l4)

    # Coupled joints
    if theta4 > theta3:
        theta4 = theta3

    return theta1, theta2, theta3, theta4


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def as_numpy(vector):
    return np.asarray([vector.x, vector.y, vector.z])


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


def joint_rotation_y(prev_basis, next_basis, rot_angle_min=None, rot_angle_max=None):
    """
    Compute the joint rotation (angle) around the y-axis between the given bases.

    Parameters
    ----------
    prev_basis : Leap.Matrix
        Basis (x, y, z, origin) of the previous basis.
    next_basis : Leap.Matrix
        Basis (x, y, z, origin) of the next basis.
    rot_angle_min : float, optional
        Minimum joint rotation (angle) limit.
    rot_angle_max : float, optional
        Maximum joint rotation (angle) limit.

    Returns
    -------
    float
        Joint rotation (angle) around y-axis.
    """

    # Compute the rotation angle and axis between the y-bases
    # rot_angle = prev_basis.y_basis.angle_to(next_basis.y_basis)
    # rot_angle2 = angle_between(as_numpy(prev_basis.y_basis),
    #                            as_numpy(next_basis.y_basis))
    rot_angle = angle_between(prev_basis[1],
                              next_basis[1])
    
    # rot_axis = prev_basis.y_basis.cross(next_basis.y_basis)
    # rot_axis2 = np.cross(
    #     as_numpy(prev_basis.y_basis),
    #     as_numpy(next_basis.y_basis)
    # )
    rot_axis = np.cross(
        prev_basis[1],
        next_basis[1]
    )
    # print("y-axis")
    # print(rot_angle, rot_angle2)
    # print(rot_axis, rot_axis2)
    
    # Set up corresponding rotation matrix
    # rot_mat = Leap.Matrix()
    # rot_mat.set_rotation(rot_axis, rot_angle)
    # print(rot_mat.x_basis, rot_mat.y_basis, rot_mat.z_basis)
    # rot_mat2 = rotation_matrix(rot_axis2, rot_angle2)
    # print(rot_mat2)
    rot_mat = rotation_matrix(rot_axis, rot_angle)

    # Rotate next_basis in prev_basis
    # rot_dir = rot_mat.transform_direction(next_basis.x_basis)
    # rot_dir2 = as_numpy(next_basis.x_basis).dot(rot_mat2)
    # print(rot_dir, rot_dir2)
    rot_dir = next_basis[0].dot(rot_mat)

    # x-bases are in one plane, ready to measure the joint rotation (angle)
    # rot_joint = prev_basis.x_basis.angle_to(rot_dir)
    # cross = rot_dir.cross(prev_basis.x_basis)
    
    # rot_joint2 = angle_between(as_numpy(prev_basis.x_basis),
    #                            rot_dir2)
    # cross2 = np.cross(
    #     rot_dir2,
    #     as_numpy(prev_basis.x_basis)
    # )
    rot_joint = angle_between(prev_basis[0],
                               rot_dir)
    cross = np.cross(
        rot_dir,
        prev_basis[0]
    )
    # print("x-axis")
    # print(rot_joint, rot_joint2)
    # print(cross, cross2)

    # Determine sign
    # if cross.dot(prev_basis.y_basis) < 0:
    #     rot_joint *= -1

    if cross.dot(prev_basis[1]) < 0:
        rot_joint *= -1
        
    # Minimum limit
    if rot_angle_min is not None and rot_joint < rot_angle_min:
        rot_joint = rot_angle_min
    # if rot_angle_min is not None and rot_joint2 < rot_angle_min:
    #     rot_joint2 = rot_angle_min

    # Maximum limit
    if rot_angle_max is not None and rot_joint > rot_angle_max:
        rot_joint = rot_angle_max
    # if rot_angle_max is not None and rot_joint2 > rot_angle_max:
    #     rot_joint2 = rot_angle_max

    # print(rot_joint, rot_joint2)
    return rot_joint


def get_thumb(thumb, hand_orientation):
    hand_r = R.from_quat(hand_orientation)
    hand_basis = hand_r.as_matrix()
    # thumb = hand.fingers.finger_type(Leap.Finger.TYPE_THUMB)[0]
    # metacarpal_basis = thumb.bone(Leap.Bone.TYPE_METACARPAL).basis
    metacarpal_r = R.from_quat(thumb['metacarpal']['rotation'])
    metacarpal_basis = metacarpal_r.as_matrix()

    # rot_angle = joint_rotation_y(hand_basis, metacarpal_basis) - 45 * Leap.DEG_TO_RAD
    # rot_m = Leap.Matrix()
    # rot_m.set_rotation(hand.basis.y_basis, rot_angle)
    # rot_m2 = rotation_matrix(as_numpy(hand.basis.y_basis), rot_angle)
    rot_angle = joint_rotation_y(hand_basis, metacarpal_basis) - np.deg2rad(45)
    rot_m = rotation_matrix(hand_basis[1], rot_angle)
    
    # x_basis = rot_m.transform_direction(hand.basis.x_basis)
    # x_basis2 = as_numpy(hand.basis.x_basis).dot(rot_m2)
    x_basis = hand_basis[0].dot(rot_m)
    # z_basis = rot_m.transform_direction(hand.basis.z_basis)
    # z_basis2 = as_numpy(hand.basis.z_basis).dot(rot_m2)
    z_basis = hand_basis[2].dot(rot_m)
    
    # print(x_basis, x_basis2)
    # print(z_basis, z_basis2)

    # metacarpal_basis = Leap.Matrix(x_basis,
    #                                hand.basis.y_basis,
    #                                z_basis,
    #                                thumb.bone(Leap.Bone.TYPE_METACARPAL).next_joint)
    # print(metacarpal_basis.to_array_4x4())
    # metacarpal_transform = metacarpal_basis.rigid_inverse()
    
    metacarpal_basis = np.asarray([
        [x_basis[0], x_basis[1], x_basis[2], 0],
        [hand_basis[1][0], hand_basis[1][1], hand_basis[1][2], 0],
        [z_basis[0], z_basis[1], z_basis[2], 0],
        [thumb['metacarpal']['next_joint'][0], thumb['metacarpal']['next_joint'][1], thumb['metacarpal']['next_joint'][2], 1.0],
    ])
    # print(metacarpal_basis2)
    metacarpal_transform = np.linalg.pinv(metacarpal_basis)
    
    # End position - "endeffector"
    # end_pos_metacarpal = thumb.bone(Leap.Bone.TYPE_PROXIMAL).next_joint

    # Transform end position in current system
    # vec = metacarpal_transform.transform_point(end_pos_metacarpal)
    vec = np.asarray([thumb['proximal']['next_joint'][0], 
                       thumb['proximal']['next_joint'][1],
                       thumb['proximal']['next_joint'][2],
                       1]).dot(metacarpal_transform)
    # l2 = thumb.bone(Leap.Bone.TYPE_PROXIMAL).length
    l2 = np.linalg.norm(
        thumb['proximal']['next_joint'] - thumb['proximal']['prev_joint']
    )

    # From inverse kinematics (DH-convention)
    # theta1 = -np.arctan(vec.x / vec.y)
    # theta2 = np.pi - np.arccos(vec.z / l2)
    theta1 = -np.arctan(vec[0] / vec[1])
    theta2 = np.pi - np.arccos(vec[2] / l2)

    # Compute new proximal basis
    # rot_m.set_rotation(z_basis, -theta1)
    rot_m = rotation_matrix(z_basis, -theta1)
    # x_basis = rot_m.transform_direction(x_basis)
    x_basis = x_basis.dot(rot_m)
    # y_basis = rot_m.transform_direction(hand.basis.y_basis)
    y_basis = hand_basis[1].dot(rot_m)

    # rot_m.set_rotation(x_basis, theta2)
    rot_m = rotation_matrix(x_basis, theta2)
    # y_basis = rot_m.transform_direction(y_basis)
    y_basis = y_basis.dot(rot_m)
    # z_basis = rot_m.transform_direction(z_basis)
    z_basis = z_basis.dot(rot_m)

    # proximal_basis = Leap.Matrix(x_basis,
    #                              y_basis,
    #                              z_basis,
    #                              thumb.bone(Leap.Bone.TYPE_PROXIMAL).next_joint)
    # proximal_transform = proximal_basis.rigid_inverse()
    proximal_basis = np.asarray([
        [x_basis[0], x_basis[1], x_basis[2], 0],
        [y_basis[0], y_basis[1], y_basis[2], 0],
        [z_basis[0], z_basis[1], z_basis[2], 0],
        [thumb['proximal']['next_joint'][0], thumb['proximal']['next_joint'][1], thumb['proximal']['next_joint'][2], 1.0],
    ])
    proximal_transform = np.linalg.pinv(proximal_basis)

    # End position - "endeffector"
    # end_pos_proximal = thumb.bone(Leap.Bone.TYPE_INTERMEDIATE).next_joint

    # Transform end position in current system
    # vec = proximal_transform.transform_point(end_pos_proximal)
    vec = np.asarray([thumb['intermediate']['next_joint'][0], 
                       thumb['intermediate']['next_joint'][1],
                       thumb['intermediate']['next_joint'][2],
                       1]).dot(proximal_transform)
    # l4 = thumb.bone(Leap.Bone.TYPE_INTERMEDIATE).length
    l4 = np.linalg.norm(
        thumb['intermediate']['next_joint'] - thumb['intermediate']['prev_joint']
    )

    # From inverse kinematics (DH-convention)
    # theta3 = np.arctan(vec.y / vec.z)
    # theta4 = np.arcsin(vec.x / l4)
    theta3 = np.arctan(vec[1] / vec[2])
    theta4 = np.arcsin(vec[0] / l4)
    # print(proximal_transform)
    # print(end_pos_proximal)
    # print(theta4)

    # Compute new intermediate basis
    # rot_m.set_rotation(x_basis, theta3)
    rot_m = rotation_matrix(x_basis, theta3)
    # y_basis = rot_m.transform_direction(y_basis)
    y_basis = y_basis.dot(rot_m)
    # z_basis = rot_m.transform_direction(z_basis)
    z_basis = z_basis.dot(rot_m)

    # rot_m.set_rotation(y_basis, theta4)
    rot_m = rotation_matrix(y_basis, theta4)
    # x_basis = rot_m.transform_direction(x_basis)
    x_basis = x_basis.dot(rot_m)
    # z_basis = rot_m.transform_direction(z_basis)
    z_basis = z_basis.dot(rot_m)

    # intermediate_basis = Leap.Matrix(x_basis,
    #                                  y_basis,
    #                                  z_basis,
    #                                  thumb.bone(Leap.Bone.TYPE_INTERMEDIATE).next_joint)
    # intermediate_transform = intermediate_basis.rigid_inverse()
    intermediate_basis = np.asarray([
        [x_basis[0], x_basis[1], x_basis[2], 0],
        [y_basis[0], y_basis[1], y_basis[2], 0],
        [z_basis[0], z_basis[1], z_basis[2], 0],
        [thumb['intermediate']['next_joint'][0], thumb['intermediate']['next_joint'][1], thumb['intermediate']['next_joint'][2], 1.0],
    ])
    intermediate_transform = np.linalg.pinv(intermediate_basis)

    # End position - "endeffector"
    # end_pos_intermediate = thumb.bone(Leap.Bone.TYPE_DISTAL).next_joint

    # Transform end position in current system
    # vec = intermediate_transform.transform_point(end_pos_intermediate)
    vec = np.asarray([thumb['distal']['next_joint'][0], 
                       thumb['distal']['next_joint'][1],
                       thumb['distal']['next_joint'][2],
                       1]).dot(intermediate_transform)
    # l5 = thumb.bone(Leap.Bone.TYPE_DISTAL).length
    l5 = np.linalg.norm(
        thumb['distal']['next_joint'] - thumb['distal']['prev_joint']
    )

    # From inverse kinematics (DH-convention)
    # theta5 = np.pi - np.arccos(vec.z / l5)
    theta5 = np.pi - np.arccos(vec[2] / l5)

    return theta1, theta2, theta3, theta4, theta5






