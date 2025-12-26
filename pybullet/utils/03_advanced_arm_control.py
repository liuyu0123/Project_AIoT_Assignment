"""
PyBullet高级机械臂控制示例
功能：使用逆运动学控制机械臂末端执行器位置
作者：青潮智科技
日期：2025-11-04
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import math

class RobotArmController:
    """机械臂控制器类"""
    
    def __init__(self, robot_id, end_effector_index):
        """
        初始化机械臂控制器
        
        Args:
            robot_id: 机器人ID
            end_effector_index: 末端执行器关节索引
        """
        self.robot_id = robot_id
        self.end_effector_index = end_effector_index
        self.num_joints = p.getNumJoints(robot_id)
        
        # 获取可控制的关节
        self.controllable_joints = []
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            joint_type = joint_info[2]
            if joint_type != p.JOINT_FIXED:
                self.controllable_joints.append(i)
        
        print(f"控制器初始化完成：{len(self.controllable_joints)}个可控关节")
    
    def get_end_effector_pos(self):
        """获取末端执行器当前位置"""
        link_state = p.getLinkState(self.robot_id, self.end_effector_index)
        return link_state[0]  # 世界坐标系中的位置
    
    def get_end_effector_orientation(self):
        """获取末端执行器当前姿态"""
        link_state = p.getLinkState(self.robot_id, self.end_effector_index)
        return link_state[1]  # 四元数表示的姿态
    
    def move_to_position(self, target_pos, target_orn=None, max_force=500):
        """
        使用逆运动学将末端执行器移动到目标位置
        
        Args:
            target_pos: 目标位置 [x, y, z]
            target_orn: 目标姿态（四元数），如果为None则不约束姿态
            max_force: 关节最大力矩
        """
        # 计算逆运动学
        if target_orn is None:
            joint_poses = p.calculateInverseKinematics(
                self.robot_id,
                self.end_effector_index,
                target_pos
            )
        else:
            joint_poses = p.calculateInverseKinematics(
                self.robot_id,
                self.end_effector_index,
                target_pos,
                target_orn
            )
        
        # 设置关节目标位置
        for i, joint_index in enumerate(self.controllable_joints):
            if i < len(joint_poses):
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_index,
                    p.POSITION_CONTROL,
                    targetPosition=joint_poses[i],
                    force=max_force
                )
    
    def get_joint_angles(self):
        """获取所有可控关节的当前角度"""
        angles = []
        for joint_index in self.controllable_joints:
            joint_state = p.getJointState(self.robot_id, joint_index)
            angles.append(joint_state[0])
        return angles
    
    def set_joint_angles(self, angles, max_force=500):
        """设置关节角度"""
        for i, joint_index in enumerate(self.controllable_joints):
            if i < len(angles):
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_index,
                    p.POSITION_CONTROL,
                    targetPosition=angles[i],
                    force=max_force
                )

def draw_coordinate_frame(position, orientation, length=0.1):
    """
    在指定位置绘制坐标系
    
    Args:
        position: 位置 [x, y, z]
        orientation: 姿态（四元数）
        length: 坐标轴长度
    """
    # 计算旋转矩阵
    rotation_matrix = p.getMatrixFromQuaternion(orientation)
    rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
    
    # X轴（红色）
    x_axis = rotation_matrix[:, 0] * length
    p.addUserDebugLine(
        position,
        [position[0] + x_axis[0], position[1] + x_axis[1], position[2] + x_axis[2]],
        lineColorRGB=[1, 0, 0],
        lineWidth=2,
        lifeTime=0.1
    )
    
    # Y轴（绿色）
    y_axis = rotation_matrix[:, 1] * length
    p.addUserDebugLine(
        position,
        [position[0] + y_axis[0], position[1] + y_axis[1], position[2] + y_axis[2]],
        lineColorRGB=[0, 1, 0],
        lineWidth=2,
        lifeTime=0.1
    )
    
    # Z轴（蓝色）
    z_axis = rotation_matrix[:, 2] * length
    p.addUserDebugLine(
        position,
        [position[0] + z_axis[0], position[1] + z_axis[1], position[2] + z_axis[2]],
        lineColorRGB=[0, 0, 1],
        lineWidth=2,
        lifeTime=0.1
    )

def circular_trajectory(center, radius, height, angle):
    """
    生成圆形轨迹上的点
    
    Args:
        center: 圆心 [x, y]
        radius: 半径
        height: 高度（z坐标）
        angle: 当前角度（弧度）
    
    Returns:
        [x, y, z] 位置
    """
    x = center[0] + radius * math.cos(angle)
    y = center[1] + radius * math.sin(angle)
    z = height
    return [x, y, z]

def main():
    """主函数"""
    
    print("=" * 100)
    print("PyBullet高级机械臂控制示例 - 逆运动学控制")
    print("=" * 100)
    
    # 1. 连接到仿真器
    print("\n[步骤1] 连接到仿真器...")
    physics_client = p.connect(p.GUI)
    if physics_client < 0:
        print("❌ 连接失败！")
        return
    print("✓ 连接成功！")
    
    # 2. 设置环境
    print("\n[步骤2] 设置仿真环境...")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    print("✓ 环境设置完成")
    
    # 3. 加载地面
    print("\n[步骤3] 加载地面...")
    plane_id = p.loadURDF("plane.urdf")
    print(f"✓ 地面ID：{plane_id}")
    
    # 4. 加载机械臂
    print("\n[步骤4] 加载机械臂...")
    robot_id = p.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True
    )
    print(f"✓ 机械臂ID：{robot_id}")
    
    # 5. 配置视图
    print("\n[步骤5] 配置视图...")
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5]
    )
    print("✓ 视图配置完成")
    
    # 6. 确定末端执行器索引
    num_joints = p.getNumJoints(robot_id)
    end_effector_index = num_joints - 1  # 通常是最后一个关节
    print(f"\n[步骤6] 末端执行器索引：{end_effector_index}")
    
    # 7. 创建控制器
    print("\n[步骤7] 创建机械臂控制器...")
    controller = RobotArmController(robot_id, end_effector_index)
    
    # 8. 添加调试控制参数
    print("\n[步骤8] 添加调试控制参数...")
    x_slider = p.addUserDebugParameter("目标X", -0.5, 0.5, 0.3)
    y_slider = p.addUserDebugParameter("目标Y", -0.5, 0.5, 0.0)
    z_slider = p.addUserDebugParameter("目标Z", 0.2, 1.0, 0.5)
    mode_slider = p.addUserDebugParameter("模式(0=手动,1=圆形)", 0, 1, 0)
    print("✓ 控制参数添加完成")
    
    # 9. 添加一个目标球体用于可视化
    print("\n[步骤9] 添加目标标记...")
    target_visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.03,
        rgbaColor=[1, 0, 0, 0.5]
    )
    target_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=target_visual,
        basePosition=[0.3, 0, 0.5]
    )
    print("✓ 目标标记创建完成（红色半透明球）")
    
    # 10. 运行仿真
    print("\n[步骤10] 开始仿真...")
    print("-" * 100)
    print("控制模式：")
    print("  - 手动模式(0)：使用滑块控制目标位置")
    print("  - 圆形轨迹(1)：机械臂末端沿圆形轨迹运动")
    print("按Ctrl+C停止仿真")
    print("-" * 100)
    
    try:
        step_count = 0
        angle = 0
        
        while True:
            # 读取控制模式
            mode = p.readUserDebugParameter(mode_slider)
            
            if mode < 0.5:  # 手动模式
                # 读取目标位置
                target_x = p.readUserDebugParameter(x_slider)
                target_y = p.readUserDebugParameter(y_slider)
                target_z = p.readUserDebugParameter(z_slider)
                target_pos = [target_x, target_y, target_z]
            else:  # 圆形轨迹模式
                # 生成圆形轨迹
                angle += 0.01
                target_pos = circular_trajectory(
                    center=[0.3, 0],
                    radius=0.15,
                    height=0.5,
                    angle=angle
                )
            
            # 更新目标标记位置
            p.resetBasePositionAndOrientation(
                target_id,
                target_pos,
                [0, 0, 0, 1]
            )
            
            # 控制机械臂移动到目标位置
            controller.move_to_position(target_pos)
            
            # 获取并绘制末端执行器坐标系
            ee_pos = controller.get_end_effector_pos()
            ee_orn = controller.get_end_effector_orientation()
            draw_coordinate_frame(ee_pos, ee_orn, length=0.1)
            
            # 执行仿真步
            p.stepSimulation()
            time.sleep(1./240.)
            
            # 每隔240步打印一次信息
            if step_count % 240 == 0:
                distance = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
                print(f"\n时间: {step_count/240:.1f}秒")
                print(f"  目标位置: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                print(f"  末端位置: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
                print(f"  位置误差: {distance:.4f}m")
            
            step_count += 1
    
    except KeyboardInterrupt:
        print("\n\n用户中断仿真")
    
    # 11. 断开连接
    print("\n[步骤11] 断开连接...")
    p.disconnect()
    print("✓ 已断开连接")
    print("\n" + "=" * 100)
    print("仿真结束")
    print("=" * 100)

if __name__ == "__main__":
    main()
