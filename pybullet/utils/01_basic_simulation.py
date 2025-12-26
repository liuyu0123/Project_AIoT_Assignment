"""
PyBullet基础仿真示例
功能：创建一个简单的物理仿真场景，包含地面和几个物体
作者：青潮智科技
日期：2025-11-04
"""

import pybullet as p
import pybullet_data
import time
import numpy as np

def main():
    """主函数：运行基础仿真"""
    
    print("=" * 60)
    print("PyBullet基础仿真示例")
    print("=" * 60)
    
    # 1. 连接到PyBullet仿真器（GUI模式）
    print("\n[步骤1] 连接到PyBullet仿真器...")
    physics_client = p.connect(p.GUI)  # 使用GUI模式，会打开可视化窗口
    # p.connect(p.DIRECT)  # DIRECT模式：不显示窗口，用于后台计算
    
    if physics_client < 0:
        print("❌ 连接失败！")
        return
    else:
        print("✓ 连接成功！")
    
    # 2. 设置PyBullet数据路径（包含内置的模型文件）
    print("\n[步骤2] 设置数据路径...")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    print(f"✓ 数据路径：{pybullet_data.getDataPath()}")
    
    # 3. 设置重力
    print("\n[步骤3] 设置重力...")
    p.setGravity(0, 0, -9.81)  # x, y, z方向的重力加速度（m/s²）
    print("✓ 重力设置为：(0, 0, -9.81) m/s²")
    
    # 4. 加载地面
    print("\n[步骤4] 加载地面...")
    plane_id = p.loadURDF("plane.urdf")  # PyBullet内置的平面模型
    print(f"✓ 地面加载成功，ID：{plane_id}")
    
    # 5. 加载一些物体
    print("\n[步骤5] 加载物体...")
    
    # 5.1 加载一个立方体（使用几何形状创建）
    cube_collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[0.5, 0.5, 0.5]  # 半尺寸，实际尺寸为1x1x1
    )
    cube_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[0.5, 0.5, 0.5],
        rgbaColor=[1, 0, 0, 1]  # 红色
    )
    cube_id = p.createMultiBody(
        baseMass=1.0,  # 质量1kg
        baseCollisionShapeIndex=cube_collision_shape,
        baseVisualShapeIndex=cube_visual_shape,
        basePosition=[0, 0, 2]  # 初始位置：x=0, y=0, z=2
    )
    print(f"✓ 红色立方体创建成功，ID：{cube_id}")
    
    # 5.2 加载一个球体
    sphere_collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.3
    )
    sphere_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.3,
        rgbaColor=[0, 0, 1, 1]  # 蓝色
    )
    sphere_id = p.createMultiBody(
        baseMass=0.5,
        baseCollisionShapeIndex=sphere_collision_shape,
        baseVisualShapeIndex=sphere_visual_shape,
        basePosition=[1, 1, 3]
    )
    print(f"✓ 蓝色球体创建成功，ID：{sphere_id}")
    
    # 5.3 加载PyBullet内置的一些模型
    table_id = p.loadURDF("table/table.urdf", basePosition=[2, 0, 0])
    print(f"✓ 桌子模型加载成功，ID：{table_id}")
    
    # 6. 配置仿真参数
    print("\n[步骤6] 配置仿真参数...")
    p.setTimeStep(1./240.)  # 设置仿真步长为1/240秒
    p.setRealTimeSimulation(0)  # 关闭实时仿真，使用步进模式
    print("✓ 仿真步长：1/240秒")
    print("✓ 模式：步进模式")
    
    # 7. 配置调试可视化
    print("\n[步骤7] 配置调试视图...")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # 显示GUI控制面板
    p.resetDebugVisualizerCamera(
        cameraDistance=5,      # 相机距离
        cameraYaw=50,          # 偏航角
        cameraPitch=-35,       # 俯仰角
        cameraTargetPosition=[0, 0, 0]  # 相机目标位置
    )
    print("✓ 调试视图配置完成")
    
    # 8. 运行仿真
    print("\n[步骤8] 开始仿真...")
    print("-" * 60)
    print("仿真运行中... (按Ctrl+C停止)")
    print("-" * 60)
    
    try:
        for i in range(10000):  # 运行10000步
            p.stepSimulation()  # 执行一步仿真
            time.sleep(1./240.)  # 休眠，使仿真速度接近实时
            
            # 每隔240步（约1秒）打印一次物体位置
            if i % 240 == 0:
                cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
                sphere_pos, sphere_orn = p.getBasePositionAndOrientation(sphere_id)
                print(f"\n时间: {i/240:.1f}秒")
                print(f"  立方体位置: [{cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f}]")
                print(f"  球体位置: [{sphere_pos[0]:.2f}, {sphere_pos[1]:.2f}, {sphere_pos[2]:.2f}]")
    
    except KeyboardInterrupt:
        print("\n\n用户中断仿真")
    
    # 9. 断开连接
    print("\n[步骤9] 断开连接...")
    p.disconnect()
    print("✓ 已断开连接")
    print("\n" + "=" * 60)
    print("仿真结束")
    print("=" * 60)

if __name__ == "__main__":
    main()
