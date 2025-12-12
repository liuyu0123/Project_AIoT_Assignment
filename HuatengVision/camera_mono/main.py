#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
单目视觉测距系统
基于 OpenCV 和相似三角形原理的距离测量
原文链接: https://blog.csdn.net/qq_43010752/article/details/122320949
"""

import sys
import os
from monocular_distance import MonocularDistanceMeasurement
from focal_length_calibration import FocalLengthCalibrator
from distance_demo import DistanceDemo


def print_banner():
    """打印程序横幅"""
    print("=" * 60)
    print("        单目视觉测距系统")
    print("   基于 OpenCV 和相似三角形原理的距离测量")
    print("=" * 60)
    print("原理说明：")
    print("· 使用相似三角形法进行距离测量")
    print("· 公式：D = (W × F) / P")
    print("  - D：距离")
    print("  - W：物体实际宽度")
    print("  - F：相机焦距")
    print("  - P：物体像素宽度")
    print("=" * 60)


def print_menu():
    """打印主菜单"""
    print("\n请选择功能：")
    print("1. 实时距离测量（需要摄像头）")
    print("2. 焦距标定工具")
    print("3. 距离测量演示")
    print("4. 查看技术原理")
    print("5. 退出程序")
    print("-" * 40)


def show_theory():
    """显示技术原理"""
    print("\n" + "=" * 50)
    print("单目测距技术原理详解")
    print("=" * 50)

    print("\n1. 相似三角形原理：")
    print("   当物体距离相机不同距离时，在图像中的大小会发生变化")
    print("   根据相似三角形的性质，可以建立以下关系：")
    print("   相机焦距 F = (像素宽度 P × 实际距离 D) / 实际宽度 W")
    print("   测量距离 D = (实际宽度 W × 相机焦距 F) / 像素宽度 P")

    print("\n2. 实现步骤：")
    print("   · 步骤 1：标定相机焦距")
    print("     将已知尺寸的物体（如 A4 纸）放在已知距离处")
    print("     测量物体在图像中的像素宽度")
    print("     计算焦距：F = (P × D) / W")
    print("   · 步骤 2：实时距离测量")
    print("     检测图像中的目标物体")
    print("     测量物体的像素宽度")
    print("     应用公式计算距离：D = (W × F) / P")

    print("\n3. 图像处理流程：")
    print("   · 图像预处理：灰度化、高斯模糊")
    print("   · 二值化处理：阈值分割")
    print("   · 形态学操作：膨胀处理")
    print("   · 轮廓检测：找到目标物体")
    print("   · 边界框计算：获取像素尺寸")
    print("   · 距离计算：应用测距公式")

    print("\n4. 关键参数：")
    print("   · 焦距 (F)：相机的固有参数，需要标定获得")
    print("   · 实际宽度 (W)：目标物体的真实尺寸")
    print("   · 像素宽度 (P)：物体在图像中的像素尺寸")

    print("\n5. 应用场景：")
    print("   · 机器人导航和避障")
    print("   · 工业测量和质量控制")
    print("   · 安防监控中的距离估算")
    print("   · 增强现实应用")

    print("\n6. 优缺点分析：")
    print("   优点：")
    print("     · 成本低，只需单个摄像头")
    print("     · 实现简单，计算量小")
    print("     · 实时性好")
    print("   缺点：")
    print("     · 需要已知目标物体尺寸")
    print("     · 精度受光照和环境影响")
    print("     · 只能测量特定形状的物体")

    input("\n按回车键返回主菜单...")


def main():
    """主程序入口"""
    print_banner()

    while True:
        print_menu()
        choice = input("请输入选择（1-5）：").strip()

        try:
            if choice == "1":
                print("\n启动实时距离测量...")
                distance_system = MonocularDistanceMeasurement()
                distance_system.run()

            elif choice == "2":
                print("\n启动焦距标定工具...")
                calibrator = FocalLengthCalibrator()
                calibrator.run()

            elif choice == "3":
                print("\n启动距离测量演示...")
                demo = DistanceDemo()
                demo.run()

            elif choice == "4":
                show_theory()

            elif choice == "5":
                print("\n感谢使用单目视觉测距系统！")
                print("程序退出。")
                break

            else:
                print("无效选择，请输入 1-5 之间的数字。")

        except KeyboardInterrupt:
            print("\n\n程序被用户中断。")
            break
        except Exception as e:
            print(f"\n程序运行出错：{e}")
            print("请检查摄像头连接或重新启动程序。")


if __name__ == "__main__":
    main()