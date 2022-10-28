#!/usr/bin/python3

from math import atan, pi, atan2, asin, sin, cos, sqrt
import rclpy
from rclpy import init
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from time import sleep
import numpy as np

def eulToQuat(eul):
    roll, pitch, yaw = eul[0], eul[1], eul[2]
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return np.array([qw, qx, qy, qz])

def quatToEul(quat):
    x = atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
    y = asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
    z = atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))

    return np.array([x, y, z])

def quatInv(quat):
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]])

def quatMultiply(quat1, quat2):
    w = quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
    x = quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
    y = quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
    z = quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]

    return np.array([w, x, y, z])

def quatToMat(quat):
    mat = np.ndarray((3,3))

    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3) 
    r02 = 2 * (q1 * q3 + q0 * q2) 
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    mat[0,0] = r00
    mat[0,1] = r01
    mat[0,2] = r02
    mat[1,0] = r10
    mat[1,1] = r11
    mat[1,2] = r12
    mat[2,0] = r20
    mat[2,1] = r21
    mat[2,2] = r22

    return mat

def eulToR(eul):
    cos_yaw = cos(eul[2])
    cos_pitch = cos(eul[1])
    cos_roll = cos(eul[0])
    sin_yaw = sin(eul[2])
    sin_pitch = sin(eul[1])
    sin_roll = sin(eul[0])

    mat = np.ndarray((3,3))

    mat[0,0] = cos_pitch*cos_yaw
    mat[0,1] = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw
    mat[0,2] = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw
    mat[1,0] = cos_pitch*sin_yaw
    mat[1,1] = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw
    mat[1,2] = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_pitch
    mat[2,0] = -sin_pitch
    mat[2,1] = sin_roll*cos_pitch
    mat[2,2] = cos_roll*cos_pitch

    return mat

def matToQuat(R):
    tr = R[0,0] + R[1,1] + R[2,2]

    qw, qx, qy, qz = 0,0,0,0

    if (tr > 0):

        S = sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S

    elif ((R[0,0] > R[1,1]) and (R[0,0] > R[2,2])):
        S = sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S 
        qz = (R[0,2] + R[2,0]) / S 

    elif (R(1,1) > R(2,2)):
        S = sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S 
    else:
        S = sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S

    quat = [qw, qx, qy, qz]

    return quat

W_Q_D = [0.623134, 0.049436, 0.114871, 0.772052]

# a_v = np.array([[20], [20], [0]])
# a_psi_v = atan2(a_v[1], a_v[0])
W_v = [1, 4, 0]
W_psi_v = atan2(W_v[1], W_v[0])

# print("a_psi_v:", a_psi_v)
print("W_psi_P:", W_psi_v)

# b_q_w = quatInv(w_q_b)
# b_R_w = quatToMat(b_q_w)

D_Q_W = quatInv(W_Q_D)
print("D_Q_W:", D_Q_W)
D_R_W = quatToMat(D_Q_W)
print("D_R_W:", D_R_W)

# b_v = np.matmul(b_R_w, a_v)
# b_psi_v = atan2(b_v[1], b_v[0])

D_v = np.matmul(D_R_W, W_v)
print("D_v:", D_v)
D_psi_v_expected = atan2(D_v[1], D_v[0])
print("D_psi_v_expected:", D_psi_v_expected)
D_psi_v = 2.984513
print("D_psi_v:", D_psi_v)

# b_v = np.matmul(eulToR([0, 0, b_psi_v]), np.array([[1],[0],[0]]))
# print("b_v:", b_v)

D_v = np.matmul(eulToR([0, 0, D_psi_v]), np.array([[1],[0],[0]]))
print("D_v:", D_v)

# a_v = np.array([
#     -(b_v[1]*b_R_w[0,1] - b_v[0]*b_R_w[1,1])/(b_R_w[0,0]*b_R_w[1,1] - b_R_w[0,1]*b_R_w[1,0]),
#     (b_v[1]*b_R_w[0,0] - b_v[0]*b_R_w[1,0])/(b_R_w[0,0]*b_R_w[1,1] - b_R_w[0,1]*b_R_w[1,0]),
#     [0]
# ])
# print("a_V:",a_v)

W_v = np.array([
    -(D_v[1]*D_R_W[0,1] - D_v[0]*D_R_W[1,1])/(D_R_W[0,0]*D_R_W[1,1] - D_R_W[0,1]*D_R_W[1,0]),
    (D_v[1]*D_R_W[0,0] - D_v[0]*D_R_W[1,0])/(D_R_W[0,0]*D_R_W[1,1] - D_R_W[0,1]*D_R_W[1,0]),
    [0]
])
print("W_V:",W_v)


# a_psi_v = atan2(a_v[1], a_v[0])
# print("a_psi_v:", a_psi_v)

W_psi_v_comp = atan2(W_v[1], W_v[0])
print("W_psi_v_comp:", W_psi_v_comp)

# a_v_norm = a_v / sqrt(a_v[0]**2 + a_v[1]**2 + a_v[2]**2)
W_v_norm = W_v / sqrt(W_v[0]**2 + W_v[1]**2 + W_v[2]**2)


W_P_x = W_v_norm
W_P_y = np.matmul(eulToR([0, 0, pi/2]),W_P_x)
W_P_z = [0, 0, 1]

# print(W_P_x)
# print(W_P_y)
# print(W_P_z)

W_R_P = np.ndarray((3,3))

for i in range(3):
    W_R_P[i,0] = W_P_x[i,0]
    W_R_P[i,1] = W_P_y[i,0]
    W_R_P[i,2] = W_P_z[i]

W_Q_P = matToQuat(W_R_P)

print("W_R_P:", W_R_P)
print("W_Q_P:", W_Q_P)

exit()



init()

node = Node("orientation_tester")

pub_a = node.create_publisher(PoseStamped, "pose_a", 10)
pub_b = node.create_publisher(PoseStamped, "pose_b", 10)
pub_P = node.create_publisher(PoseStamped, "pose_P", 10)

while True:
    pose_a = PoseStamped()
    pose_a.header.frame_id = "world"
    # pose_a.header.stamp = node.get_clock().now()
    pose_a.pose.orientation.w, pose_a.pose.orientation.x, pose_a.pose.orientation.y, pose_a.pose.orientation.z = w_q_a[0], w_q_a[1], w_q_a[2], w_q_a[3]
    pose_a.pose.position.x = 0.
    pose_a.pose.position.y = 0.
    pose_a.pose.position.z = 0.

    pose_b = PoseStamped()
    pose_b.header.frame_id = "world"
    # pose_b.header.stamp = node.get_clock().now()
    pose_b.pose.orientation.w, pose_b.pose.orientation.x, pose_b.pose.orientation.y, pose_b.pose.orientation.z = w_q_b[0], w_q_b[1], w_q_b[2], w_q_b[3]
    pose_b.pose.position.x = 0.
    pose_b.pose.position.y = 0.
    pose_b.pose.position.z = 2.

    pose_P = PoseStamped()
    pose_P.header.frame_id = "world"
    # pose_b.header.stamp = node.get_clock().now()
    pose_P.pose.orientation.w, pose_P.pose.orientation.x, pose_P.pose.orientation.y, pose_P.pose.orientation.z = quat_P[0], quat_P[1], quat_P[2], quat_P[3]
    pose_P.pose.position.x = 0.
    pose_P.pose.position.y = 0.
    pose_P.pose.position.z = 2.

    pub_a.publish(pose_a)
    pub_b.publish(pose_b)
    pub_P.publish(pose_P)




