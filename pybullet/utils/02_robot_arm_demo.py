"""
PyBullet机械臂基础示例
功能：加载机械臂模型，查看关节信息，进行简单的关节控制
作者：青潮智科技
日期：2025-11-04
"""

import pybullet as p
import pybullet_data
import time
import numpy as np

def print_robot_info(robot_id):
    """打印机器人的详细信息"""
    num_joints = p.getNumJoints(robot_id)
    print(f"\n机器人关节数量: {num_joints}")
    print("-" * 100)
    print(f"{'索引':<6} {'名称':<25} {'类型':<15} {'阻尼':<10} {'摩擦':<10} {'范围':<20}")
    print("-" * 100)
    
    joint_types = {
        p.JOINT_REVOLUTE: "旋转关节",
        p.JOINT_PRISMATIC: "移动关节",
        p.JOINT_SPHERICAL: "球形关节",
        p.JOINT_PLANAR: "平面关节",
        p.JOINT_FIXED: "固定关节"
    }
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_types.get(joint_info[2], "未知类型")
        joint_damping = joint_info[6]
        joint_friction = joint_info[7]
        joint_lower_limit = joint_info[8]
        joint_upper_limit = joint_info[9]
        
        limit_str = f"[{joint_lower_limit:.2f}, {joint_upper_limit:.2f}]"
        
        print(f"{i:<6} {joint_name:<25} {joint_type:<15} {joint_damping:<10.3f} {joint_friction:<10.3f} {limit_str:<20}")
    
    print("-" * 100)

def get_controllable_joints(robot_id):
    """获取可控制的关节索引列表（排除固定关节）"""
    num_joints = p.getNumJoints(robot_id)
    controllable_joints = []
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:  # 不是固定关节
            controllable_joints.append(i)
    
    return controllable_joints

def main():
    """主函数"""
    
    print("=" * 100)
    print("PyBullet机械臂基础示例")
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
    
    # 4. 加载机械臂（使用PyBullet内置的Kuka机械臂）
    print("\n[步骤4] 加载机械臂模型...")
    robot_start_pos = [0, 0, 0]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    # 使用Kuka IIWA机械臂（PyBullet内置模型）
    robot_id = p.loadURDF(
        "kuka_iiwa/model.urdf",
        robot_start_pos,
        robot_start_orientation,
        useFixedBase=True  # 固定基座
    )
    print(f"✓ 机械臂加载成功，ID：{robot_id}")
    
    # 5. 打印机械臂信息
    print("\n[步骤5] 查看机械臂信息...")
    print_robot_info(robot_id)
    
    # 6. 获取可控制的关节
    controllable_joints = get_controllable_joints(robot_id)
    print(f"\n可控制的关节索引: {controllable_joints}")
    print(f"可控制关节数量: {len(controllable_joints)}")
    
    # 7. 配置调试视图
    print("\n[步骤6] 配置视图...")
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=50,
        cameraPitch=-35,
        cameraTargetPosition=[0, 0, 0.5]
    )
    print("✓ 视图配置完成")
    
    # 8. 添加调试参数滑块（用于手动控制关节）
    print("\n[步骤7] 添加关节控制滑块...")
    joint_sliders = []
    for joint_index in controllable_joints:
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode('utf-8')
        joint_lower_limit = joint_info[8]
        joint_upper_limit = joint_info[9]
        
        # 创建滑块
        slider = p.addUserDebugParameter(
            joint_name,
            joint_lower_limit,
            joint_upper_limit,
            0  # 初始值
        )
        joint_sliders.append(slider)
    print(f"✓ 已添加{len(joint_sliders)}个控制滑块")
    
    # 9. 运行仿真（手动控制模式）
    print("\n[步骤8] 开始仿真...")
    print("-" * 100)
    print("提示：使用右侧的滑块控制机械臂各关节！")
    print("按Ctrl+C停止仿真")
    print("-" * 100)
    
    try:
        step_count = 0
        while True:
            # 读取滑块值并设置关节目标位置
            for i, joint_index in enumerate(controllable_joints):
                target_position = p.readUserDebugParameter(joint_sliders[i])
                p.setJointMotorControl2(
                    robot_id,
                    joint_index,
                    p.POSITION_CONTROL,
                    targetPosition=target_position,
                    force=500  # 最大力矩
                )
            
            # 执行仿真步
            p.stepSimulation()
            time.sleep(1./240.)
            
            # 每隔240步打印一次关节状态
            if step_count % 240 == 0:
                print(f"\n时间: {step_count/240:.1f}秒")
                print("当前关节角度:")
                for joint_index in controllable_joints:
                    joint_state = p.getJointState(robot_id, joint_index)
                    joint_position = joint_state[0]
                    joint_info = p.getJointInfo(robot_id, joint_index)
                    joint_name = joint_info[1].decode('utf-8')
                    print(f"  {joint_name}: {np.rad2deg(joint_position):.2f}°")
            
            step_count += 1
    
    except KeyboardInterrupt:
        print("\n\n用户中断仿真")
    
    # 10. 断开连接
    print("\n[步骤9] 断开连接...")
    p.disconnect()
    print("✓ 已断开连接")
    print("\n" + "=" * 100)
    print("仿真结束")
    print("=" * 100)

if __name__ == "__main__":
    main()
