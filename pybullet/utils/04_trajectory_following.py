"""
机械臂轨迹跟踪示例
功能：让机械臂末端沿着预定义的轨迹运动
作者：青潮智科技
日期：2025-11-04
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import sys
import os

# 添加utils目录到路径
sys.path.append(os.path.dirname(__file__))
# from utils import robot_helper as rh
import robot_helper as rh

def visualize_trajectory(trajectory, color=[1, 0.5, 0], width=3):
    """
    可视化轨迹
    
    Args:
        trajectory: 轨迹点列表
        color: 线条颜色
        width: 线条宽度
    """
    for i in range(len(trajectory) - 1):
        rh.draw_debug_line(
            trajectory[i],
            trajectory[i + 1],
            color=color,
            width=width,
            lifetime=0  # 永久显示
        )

def main():
    """主函数"""
    
    print("=" * 100)
    print("机械臂轨迹跟踪示例")
    print("=" * 100)
    
    # 1. 初始化仿真环境
    print("\n[初始化] 连接到仿真器...")
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    
    # 2. 加载场景
    print("[初始化] 加载场景...")
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True
    )
    
    # 3. 配置视图
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5]
    )
    
    # 4. 获取机械臂信息
    num_joints = p.getNumJoints(robot_id)
    end_effector_index = num_joints - 1
    controllable_joints = rh.get_controllable_joints(robot_id)
    
    print(f"✓ 机械臂加载完成")
    print(f"  - 可控关节数: {len(controllable_joints)}")
    print(f"  - 末端执行器索引: {end_effector_index}")
    
    # 5. 定义轨迹
    print("\n[轨迹规划] 生成轨迹...")
    
    # 方案1：直线轨迹
    trajectory_1 = rh.calculate_trajectory_linear(
        start_pos=[0.3, 0.3, 0.5],
        end_pos=[0.3, -0.3, 0.5],
        num_points=50
    )
    
    # 方案2：圆形轨迹
    trajectory_2 = rh.calculate_trajectory_circular(
        center=[0.4, 0],
        radius=0.15,
        height=0.5,
        num_points=100
    )
    
    # 方案3：组合轨迹（正方形）
    trajectory_3 = []
    square_corners = [
        [0.4, 0.2, 0.3],
        [0.4, -0.2, 0.3],
        [0.2, -0.2, 0.3],
        [0.2, 0.2, 0.3],
        [0.4, 0.2, 0.3]  # 回到起点
    ]
    for i in range(len(square_corners) - 1):
        segment = rh.calculate_trajectory_linear(
            square_corners[i],
            square_corners[i + 1],
            num_points=25
        )
        trajectory_3.extend(segment)
    
    # 选择要使用的轨迹
    trajectories = {
        1: ("直线轨迹", trajectory_1, [1, 0, 0]),
        2: ("圆形轨迹", trajectory_2, [0, 1, 0]),
        3: ("正方形轨迹", trajectory_3, [0, 0, 1])
    }
    
    # 6. 添加轨迹选择器
    trajectory_selector = p.addUserDebugParameter("轨迹选择(1/2/3)", 1, 3, 2)
    speed_slider = p.addUserDebugParameter("速度倍率", 0.1, 5.0, 1.0)
    
    print("✓ 轨迹生成完成")
    print("  - 轨迹1: 直线（红色）")
    print("  - 轨迹2: 圆形（绿色）")
    print("  - 轨迹3: 正方形（蓝色）")
    
    # 7. 运行仿真
    print("\n[仿真] 开始轨迹跟踪...")
    print("-" * 100)
    print("使用右侧滑块选择轨迹和调整速度")
    print("按Ctrl+C停止")
    print("-" * 100)
    
    current_trajectory_id = 2
    current_trajectory = trajectory_2
    trajectory_index = 0
    visualized = False
    
    try:
        step_count = 0
        
        while True:
            # 检查是否切换轨迹
            selected_trajectory = int(p.readUserDebugParameter(trajectory_selector))
            if selected_trajectory != current_trajectory_id:
                current_trajectory_id = selected_trajectory
                name, traj, color = trajectories[selected_trajectory]
                current_trajectory = traj
                trajectory_index = 0
                
                # 清除之前的可视化
                p.removeAllUserDebugItems()
                
                # 可视化新轨迹
                visualize_trajectory(current_trajectory, color=color, width=3)
                
                print(f"\n切换到: {name}")
                print(f"轨迹点数: {len(current_trajectory)}")
            
            # 首次可视化
            if not visualized:
                name, traj, color = trajectories[current_trajectory_id]
                visualize_trajectory(current_trajectory, color=color, width=3)
                visualized = True
            
            # 获取速度倍率
            speed = p.readUserDebugParameter(speed_slider)
            
            # 更新轨迹索引
            if step_count % max(1, int(5 / speed)) == 0:
                trajectory_index = (trajectory_index + 1) % len(current_trajectory)
            
            # 获取目标位置
            target_pos = current_trajectory[trajectory_index]
            
            # 计算逆运动学
            joint_poses = rh.calculate_ik(robot_id, end_effector_index, target_pos)
            
            # 控制机械臂
            rh.apply_joint_control(
                robot_id,
                controllable_joints,
                joint_poses[:len(controllable_joints)]
            )
            
            # 获取末端执行器位置
            ee_pos, ee_orn, _, _ = rh.get_end_effector_state(robot_id, end_effector_index)
            
            # 绘制末端执行器坐标系
            rh.draw_coordinate_system(ee_pos, ee_orn, length=0.08, lifetime=0.1)
            
            # 绘制目标点
            p.addUserDebugLine(
                target_pos,
                [target_pos[0], target_pos[1], target_pos[2] + 0.05],
                lineColorRGB=[1, 1, 0],
                lineWidth=5,
                lifeTime=0.1
            )
            
            # 执行仿真
            p.stepSimulation()
            time.sleep(1./240.)
            
            # 定期打印信息
            if step_count % 240 == 0:
                distance = rh.distance_3d(ee_pos, target_pos)
                progress = (trajectory_index / len(current_trajectory)) * 100
                print(f"\n时间: {step_count/240:.1f}秒 | "
                      f"进度: {progress:.1f}% | "
                      f"误差: {distance*1000:.2f}mm")
            
            step_count += 1
    
    except KeyboardInterrupt:
        print("\n\n用户中断仿真")
    
    # 断开连接
    print("\n[清理] 断开连接...")
    p.disconnect()
    print("✓ 仿真结束")
    print("=" * 100)

if __name__ == "__main__":
    main()
