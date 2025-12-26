"""
作者：青潮智科技
日期：2025-11-06
"""

import numpy as np
import math

def euler2transf(x, y, z, roll, pitch, yaw):

    # 平移向量
    trans = np.transpose(np.array([[x,y,z]]))

    roll_transf = np.identity(3)
    roll_transf[1,1] = math.cos(roll)
    roll_transf[1,2] = -math.sin(roll)
    roll_transf[2,1] = math.sin(roll)
    roll_transf[2,2] = math.cos(roll)
    
    # 绕Y轴旋转（pitch）
    pitch_transf = np.identity(3)
    roll_transf[0,0] = math.cos(pitch)
    roll_transf[0,2] = math.sin(pitch)
    roll_transf[2,0] = -math.sin(pitch)
    roll_transf[2,2] = math.cos(pitch)

    # 绕Z轴旋转（yaw）
    yaw_transf = np.identity(3)
    yaw_transf[0,0] = math.cos(yaw)
    yaw_transf[0,1] = -math.sin(yaw)
    yaw_transf[1,0] = math.sin(yaw)
    yaw_transf[1,1] = math.cos(yaw)

    # 旋转矩阵（按roll-pitch-yaw顺序组合）
    rotational = np.matmul(np.matmul(roll_transf,pitch_transf),yaw_transf)

    # 齐次变换矩阵 [R | t]
    #              [0 | 1]
    result = np.block([[rotational,trans],
                       [np.zeros([1,3]),1]])

    return result

def invtransf(transf):

    # 旋转矩阵的转置（等于旋转矩阵的逆）
    rotational_t = np.transpose(transf[0:3,0:3])
    trans = np.transpose([transf[0:3,3]])
    
    # 逆变换矩阵 [R^T | -R^T*t]
    #           [0   |    1    ]
    result = np.block([[rotational_t,np.matmul(-rotational_t,trans)],
                       [np.zeros([1,3]),1]])
    
    return result

def global2local_legpos(legpos_global, x_global, y_global, z_global, roll, pitch, yaw):

    T_global_body = euler2transf(x_global, y_global, z_global, roll, pitch, yaw)

    T_global_front_left_origin = np.matmul(T_global_body, euler2transf(body_width/2, -body_front, 0, 0, 0, 0))

    T_global_front_right_origin = np.matmul(T_global_body, euler2transf(-body_width/2, -body_front, 0, 0, 0, 0))

    T_global_back_left_origin = np.matmul(T_global_body, euler2transf(body_width/2, body_back, 0, 0, 0, 0))

    T_global_back_right_origin = np.matmul(T_global_body, euler2transf(-body_width/2, body_back, 0, 0, 0, 0))

    legpos_global_matix = np.concatenate((legpos_global, np.ones([1,4])), axis=0)

    legpos_local = np.zeros([4,4])
    legpos_local[:,0] =  np.matmul(invtransf(T_global_front_left_origin), legpos_global_matix[:,0])
    legpos_local[:,1] =  np.matmul(invtransf(T_global_front_right_origin), legpos_global_matix[:,1])
    legpos_local[:,2] =  np.matmul(invtransf(T_global_back_left_origin), legpos_global_matix[:,2])
    legpos_local[:,3] =  np.matmul(invtransf(T_global_back_right_origin), legpos_global_matix[:,3])

    return legpos_local[0:3,:]

def inv_kine(legpos_local): 

    result = np.zeros([3,4])
    
    # 对每条腿进行逆运动学求解
    for i in range(4):
        x = legpos_local[0,i]
        y = legpos_local[1,i]
        z = legpos_local[2,i]

        # 计算投影到xz平面后，扣除髋部长度的距离
        F = math.sqrt(x*x + z*z - hip_length*hip_length)

        # 计算髋部外展关节角度（不同腿需要不同的符号）
        if (i==1 or i==2):
            hip_rolling_angle = math.atan2(z, -x*pow((-1),i)) + math.atan2(F, -hip_length)
        else:
            hip_rolling_angle = -math.atan2(z, -x*pow((-1),i)) - math.atan2(F, -hip_length)
        
        # 使用余弦定理计算膝关节角度
        D = (F*F + y*y - upperleg_length*upperleg_length - lowerleg_length*lowerleg_length) / (2*upperleg_length*lowerleg_length)
        
        knee_angle = -math.atan2((math.sqrt(1-D*D)), D)

        # 计算髋部俯仰关节角度
        hip_pitching_angle = pow((-1),i)*(math.atan2(y,F) - math.atan2(lowerleg_length * math.sin(knee_angle), upperleg_length + lowerleg_length * math.cos(knee_angle)))

        result[0,i] = hip_rolling_angle
        result[1,i] = hip_pitching_angle
        result[2,i] = knee_angle
    
    return result

# testudog机器人的几何参数（单位：米）
hip_length = 0.0623          
upperleg_length = 0.118     
lowerleg_length = 0.118     
body_front = 0.102         
body_back = 0.252          
body_width = 0.150        
