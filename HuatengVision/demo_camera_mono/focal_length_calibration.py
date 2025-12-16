import cv2
import numpy as np


class FocalLengthCalibrator:
    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.capture = None
        self.init_camera()

    # ---------------------------------------------------------
    # 自动尝试打开摄像头
    # ---------------------------------------------------------
    def init_camera(self):
        """初始化摄像头"""
        try:
            for camera_index in [0, 1, 2]:
                self.capture = cv2.VideoCapture(camera_index)
                if self.capture.isOpened():
                    print(f"成功连接摄像头 {camera_index}")
                    break
            else:
                print("警告：无法连接摄像头")
                self.capture = None
        except Exception as e:
            print(f"摄像头初始化失败：{e}")
            self.capture = None

    # ---------------------------------------------------------
    # 焦距计算公式
    # ---------------------------------------------------------
    def calculate_focal_length(self, pixel_width: float,
                               real_width: float,
                               distance: float) -> float:
        """
        计算焦距
        pixel_width : 像素宽度
        real_width  : 实际宽度（英寸）
        distance    : 距离（英寸）
        """
        return (pixel_width * distance) / real_width

    # ---------------------------------------------------------
    # 手动输入标定
    # ---------------------------------------------------------
    def manual_calibration(self) -> float | None:
        print("\n=== 手动焦距标定 ===")
        print("请按照以下步骤进行标定：")
        print("1. 准备一张 A4 纸（宽度 11 英寸）")
        print("2. 将 A4 纸放在距离相机已知距离的位置")
        print("3. 测量 A4 纸在图像中的像素宽度")

        try:
            real_width = float(input("请输入 A4 纸实际宽度（英寸，默认 11）：") or "11")
            distance = float(input("请输入 A4 纸到相机的距离（英寸）："))
            pixel_width = float(input("请输入 A4 纸在图像中的像素宽度："))

            focal_length = self.calculate_focal_length(pixel_width, real_width, distance)

            print(f"\n计算结果：")
            print(f"焦距 F = {focal_length:.2f}")
            print(f"公式：F = (P × D) / W = ({pixel_width} × {distance}) / {real_width} = {focal_length:.2f}")
            return focal_length
        except ValueError:
            print("输入格式错误，请输入数字")
            return None

    # ---------------------------------------------------------
    # 交互式鼠标拖拽标定
    # ---------------------------------------------------------
    def interactive_calibration(self) -> float | None:
        if not self.capture:
            print("无摄像头，无法进行交互式标定")
            return None

        print("\n=== 交互式焦距标定 ===")
        print("请将 A4 纸放在相机前，用鼠标从左到右拖拽测量像素宽度")
        print("按空格键计算焦距，按 ESC 退出")

        measuring = False
        start_point = None
        end_point = None
        pixel_width = 0

        def mouse_callback(event, x, y, flags, param):
            nonlocal measuring, start_point, end_point, pixel_width
            if event == cv2.EVENT_LBUTTONDOWN:
                measuring = True
                start_point = (x, y)
                end_point = None
            elif event == cv2.EVENT_MOUSEMOVE and measuring:
                end_point = (x, y)
            elif event == cv2.EVENT_LBUTTONUP:
                measuring = False
                if start_point and end_point:
                    pixel_width = abs(end_point[0] - start_point[0])

        cv2.namedWindow("Focal Length Calibration")
        cv2.setMouseCallback("Focal Length Calibration", mouse_callback)

        while True:
            ret, frame = self.capture.read()
            if not ret:
                break

            # 绘制测量线
            if start_point and end_point:
                cv2.line(frame, start_point, end_point, (0, 255, 0), 2)
                cv2.putText(frame, f"Width: {pixel_width} pixels",
                            (10, 30), self.font, 0.8, (0, 255, 0), 2)

            # 提示文字
            cv2.putText(frame, "Drag to measure A4 paper width",
                        (10, frame.shape[0] - 40), self.font, 0.6, (255, 255, 255), 1)
            cv2.putText(frame, "SPACE: Calculate | ESC: Exit",
                        (10, frame.shape[0] - 20), self.font, 0.6, (255, 255, 255), 1)

            cv2.imshow("Focal Length Calibration", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == 32 and pixel_width > 0:  # SPACE
                try:
                    real_width = float(input("\n请输入 A4 纸实际宽度（英寸，默认 11）：") or "11")
                    distance = float(input("请输入 A4 纸到相机的距离（英寸）："))
                    focal_length = self.calculate_focal_length(pixel_width, real_width, distance)
                    print(f"\n计算结果：")
                    print(f"像素宽度：{pixel_width}")
                    print(f"实际宽度：{real_width} 英寸")
                    print(f"距离：{distance} 英寸")
                    print(f"焦距：{focal_length:.2f}")
                    cv2.destroyAllWindows()
                    return focal_length
                except ValueError:
                    print("输入格式错误")

        cv2.destroyAllWindows()
        return None

    # ---------------------------------------------------------
    # 主菜单
    # ---------------------------------------------------------
    def run(self) -> None:
        print("=" * 50)
        print("焦距标定工具")
        print("=" * 50)

        while True:
            print("\n请选择标定方式：")
            print("1. 手动输入标定")
            print("2. 交互式标定")
            print("3. 退出")
            choice = input("请输入选择（1-3）：").strip()

            if choice == "1":
                focal_length = self.manual_calibration()
                if focal_length:
                    print(f"\n建议在主程序中使用焦距值：{focal_length:.2f}")
            elif choice == "2":
                focal_length = self.interactive_calibration()
                if focal_length:
                    print(f"\n建议在主程序中使用焦距值：{focal_length:.2f}")
            elif choice == "3":
                break
            else:
                print("无效选择，请重新输入")

        if self.capture:
            self.capture.release()
        cv2.destroyAllWindows()


# ---------------------------------------------------------
# 入口
# ---------------------------------------------------------
if __name__ == "__main__":
    calibrator = FocalLengthCalibrator()
    calibrator.run()