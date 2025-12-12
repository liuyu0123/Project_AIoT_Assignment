"""
机械臂操作辅助函数库
提供常用的机械臂操作封装函数
作者：青潮智科技
日期：2025-11-04
"""

import pybullet as p
import numpy as np
import math

def get_robot_info(robot_id):
    """
    获取机器人完整信息
    
    Args:
        robot_id: 机器人ID
    
    Returns:
        dict: 包含机器人信息的字典
    """
    num_joints = p.getNumJoints(robot_id)
    
    info = {
        'num_joints': num_joints,
        'joints': []
    }
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_data = {
            'index': i,
            'name': joint_info[1].decode('utf-8'),
            'type': joint_info[2],
            'damping': joint_info[6],
            'friction': joint_info[7],
            'lower_limit': joint_info[8],
            'upper_limit': joint_info[9],
            'max_force': joint_info[10],
            'max_velocity': joint_info[11],
        }
        info['joints'].append(joint_data)
    
    return info

def get_controllable_joints(robot_id):
    """
    获取可控制的关节列表
    
    Args:
        robot_id: 机器人ID
    
    Returns:
        list: 可控制关节的索引列表
    """
    num_joints = p.getNumJoints(robot_id)
    controllable = []
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_type = joint_info[2]
        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            controllable.append(i)
    
    return controllable

def reset_robot(robot_id, joint_angles=None):
    """
    重置机器人到指定姿态
    
    Args:
        robot_id: 机器人ID
        joint_angles: 关节角度列表，如果为None则重置到零位
    """
    controllable_joints = get_controllable_joints(robot_id)
    
    if joint_angles is None:
        joint_angles = [0] * len(controllable_joints)
    
    for i, joint_index in enumerate(controllable_joints):
        if i < len(joint_angles):
            p.resetJointState(robot_id, joint_index, joint_angles[i])

def move_joints_smooth(robot_id, target_angles, steps=100, max_force=500):
    """
    平滑地移动关节到目标角度
    
    Args:
        robot_id: 机器人ID
        target_angles: 目标关节角度列表
        steps: 插值步数
        max_force: 最大力矩
    
    Yields:
        当前步数
    """
    controllable_joints = get_controllable_joints(robot_id)
    
    # 获取当前角度
    current_angles = []
    for joint_index in controllable_joints:
        joint_state = p.getJointState(robot_id, joint_index)
        current_angles.append(joint_state[0])
    
    # 线性插值
    for step in range(steps):
        alpha = (step + 1) / steps
        interpolated_angles = [
            current + alpha * (target - current)
            for current, target in zip(current_angles, target_angles)
        ]
        
        # 设置关节位置
        for i, joint_index in enumerate(controllable_joints):
            if i < len(interpolated_angles):
                p.setJointMotorControl2(
                    robot_id,
                    joint_index,
                    p.POSITION_CONTROL,
                    targetPosition=interpolated_angles[i],
                    force=max_force
                )
        
        yield step

def get_end_effector_state(robot_id, end_effector_index):
    """
    获取末端执行器状态
    
    Args:
        robot_id: 机器人ID
        end_effector_index: 末端执行器索引
    
    Returns:
        tuple: (位置, 姿态, 速度, 角速度)
    """
    link_state = p.getLinkState(
        robot_id,
        end_effector_index,
        computeLinkVelocity=1
    )
    
    position = link_state[0]
    orientation = link_state[1]
    linear_velocity = link_state[6]
    angular_velocity = link_state[7]
    
    return position, orientation, linear_velocity, angular_velocity

def calculate_ik(robot_id, end_effector_index, target_pos, target_orn=None):
    """
    计算逆运动学
    
    Args:
        robot_id: 机器人ID
        end_effector_index: 末端执行器索引
        target_pos: 目标位置
        target_orn: 目标姿态（可选）
    
    Returns:
        list: 关节角度
    """
    if target_orn is None:
        joint_poses = p.calculateInverseKinematics(
            robot_id,
            end_effector_index,
            target_pos
        )
    else:
        joint_poses = p.calculateInverseKinematics(
            robot_id,
            end_effector_index,
            target_pos,
            target_orn
        )
    
    return list(joint_poses)

def draw_debug_line(start_pos, end_pos, color=[1, 0, 0], width=2, lifetime=0):
    """
    绘制调试线
    
    Args:
        start_pos: 起点位置
        end_pos: 终点位置
        color: RGB颜色
        width: 线宽
        lifetime: 生命周期（秒），0表示永久
    
    Returns:
        线的ID
    """
    return p.addUserDebugLine(
        start_pos,
        end_pos,
        lineColorRGB=color,
        lineWidth=width,
        lifeTime=lifetime
    )

def draw_coordinate_system(position, orientation, length=0.1, lifetime=0):
    """
    绘制坐标系（XYZ轴）
    
    Args:
        position: 位置
        orientation: 姿态（四元数）
        length: 轴长度
        lifetime: 生命周期
    """
    # 获取旋转矩阵
    rotation_matrix = p.getMatrixFromQuaternion(orientation)
    rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
    
    # X轴（红色）
    x_end = [
        position[0] + rotation_matrix[0, 0] * length,
        position[1] + rotation_matrix[1, 0] * length,
        position[2] + rotation_matrix[2, 0] * length
    ]
    draw_debug_line(position, x_end, color=[1, 0, 0], lifetime=lifetime)
    
    # Y轴（绿色）
    y_end = [
        position[0] + rotation_matrix[0, 1] * length,
        position[1] + rotation_matrix[1, 1] * length,
        position[2] + rotation_matrix[2, 1] * length
    ]
    draw_debug_line(position, y_end, color=[0, 1, 0], lifetime=lifetime)
    
    # Z轴（蓝色）
    z_end = [
        position[0] + rotation_matrix[0, 2] * length,
        position[1] + rotation_matrix[1, 2] * length,
        position[2] + rotation_matrix[2, 2] * length
    ]
    draw_debug_line(position, z_end, color=[0, 0, 1], lifetime=lifetime)

def calculate_trajectory_linear(start_pos, end_pos, num_points):
    """
    计算直线轨迹
    
    Args:
        start_pos: 起点
        end_pos: 终点
        num_points: 轨迹点数量
    
    Returns:
        list: 轨迹点列表
    """
    trajectory = []
    for i in range(num_points):
        alpha = i / (num_points - 1)
        point = [
            start_pos[j] + alpha * (end_pos[j] - start_pos[j])
            for j in range(3)
        ]
        trajectory.append(point)
    return trajectory

def calculate_trajectory_circular(center, radius, height, num_points, start_angle=0, end_angle=2*math.pi):
    """
    计算圆形轨迹
    
    Args:
        center: 圆心 [x, y]
        radius: 半径
        height: 高度（z坐标）
        num_points: 轨迹点数量
        start_angle: 起始角度（弧度）
        end_angle: 结束角度（弧度）
    
    Returns:
        list: 轨迹点列表
    """
    trajectory = []
    for i in range(num_points):
        angle = start_angle + (end_angle - start_angle) * i / (num_points - 1)
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        z = height
        trajectory.append([x, y, z])
    return trajectory

def apply_joint_control(robot_id, joint_indices, target_positions, max_forces=None):
    """
    批量控制关节
    
    Args:
        robot_id: 机器人ID
        joint_indices: 关节索引列表
        target_positions: 目标位置列表
        max_forces: 最大力矩列表（可选）
    """
    if max_forces is None:
        max_forces = [500] * len(joint_indices)
    
    for i, joint_index in enumerate(joint_indices):
        if i < len(target_positions):
            p.setJointMotorControl2(
                robot_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=target_positions[i],
                force=max_forces[i] if i < len(max_forces) else 500
            )

def get_joint_states(robot_id, joint_indices):
    """
    获取多个关节的状态
    
    Args:
        robot_id: 机器人ID
        joint_indices: 关节索引列表
    
    Returns:
        dict: 包含位置、速度、力矩的字典
    """
    positions = []
    velocities = []
    forces = []
    
    for joint_index in joint_indices:
        state = p.getJointState(robot_id, joint_index)
        positions.append(state[0])
        velocities.append(state[1])
        forces.append(state[3])
    
    return {
        'positions': positions,
        'velocities': velocities,
        'forces': forces
    }

def quaternion_to_euler(quaternion):
    """
    四元数转欧拉角
    
    Args:
        quaternion: 四元数 [x, y, z, w]
    
    Returns:
        list: 欧拉角 [roll, pitch, yaw]（弧度）
    """
    return p.getEulerFromQuaternion(quaternion)

def euler_to_quaternion(euler):
    """
    欧拉角转四元数
    
    Args:
        euler: 欧拉角 [roll, pitch, yaw]（弧度）
    
    Returns:
        list: 四元数 [x, y, z, w]
    """
    return p.getQuaternionFromEuler(euler)

def distance_3d(pos1, pos2):
    """
    计算两点间的三维距离
    
    Args:
        pos1: 点1
        pos2: 点2
    
    Returns:
        float: 距离
    """
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(pos1, pos2)))
