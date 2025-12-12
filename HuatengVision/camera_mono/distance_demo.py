import cv2
import numpy as np


class DistanceDemo:
    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.foc = 2810.0          # 默认焦距（像素）
        self.real_wid = 11.69      # A4 纸宽度（英寸）

    # ---------------------------------------------------------
    # 根据给定距离生成单张演示画面
    # ---------------------------------------------------------
    def create_demo_scene(self, distance_cm: float) -> np.ndarray:
        width, height = 800, 600
        frame = np.full((height, width, 3), 40, dtype=np.uint8)  # 深灰背景

        # 1. 计算 A4 纸在当前距离下的像素宽度
        distance_inch = distance_cm / 2.54
        pixel_width = (self.real_wid * self.foc) / distance_inch

        # 2. 计算矩形尺寸（A4 比例 1 : √2 ≈ 1 : 1.414）
        rect_width = int(pixel_width)
        rect_height = int(rect_width / 1.414)

        # 限制尺寸，防止过大或过小
        rect_width = max(20, min(rect_width, width - 100))
        rect_height = max(15, min(rect_height, height - 100))

        rect_x = (width - rect_width) // 2
        rect_y = (height - rect_height) // 2

        # 3. 绘制 A4 纸
        cv2.rectangle(frame,
                      (rect_x, rect_y),
                      (rect_x + rect_width, rect_y + rect_height),
                      (255, 255, 255), -1)
        cv2.rectangle(frame,
                      (rect_x, rect_y),
                      (rect_x + rect_width, rect_y + rect_height),
                      (0, 255, 0), 2)

        # 4. 显示信息
        info_text = [
            f"Distance: {distance_cm:.1f} cm",
            f"Pixel Width: {rect_width} px",
            f"Real Width: {self.real_wid:.2f} inch",
            f"Focal Length: {self.foc:.1f}"
        ]
        for i, text in enumerate(info_text):
            cv2.putText(frame, text, (10, 30 + i * 25),
                        self.font, 0.6, (255, 255, 255), 1)

        # 5. 显示公式
        formula = (f"D = (W * F) / P = ({self.real_wid:.2f} * {self.foc:.0f}) "
                   f"/ {rect_width} = {distance_cm:.1f} cm")
        cv2.putText(frame, formula, (10, height - 40),
                    self.font, 0.5, (0, 255, 255), 1)
        return frame

    # ---------------------------------------------------------
    # 交互式距离调整演示
    # ---------------------------------------------------------
    def run_distance_simulation(self) -> None:
        print("距离测量演示")
        print("使用上下箭头键调整距离，Esc 退出")
        current_distance = 50.0  # 初始距离 50 cm

        while True:
            frame = self.create_demo_scene(current_distance)
            cv2.putText(frame,
                        "UP/DOWN: Adjust Distance | Esc: Exit",
                        (10, frame.shape[0] - 10),
                        self.font, 0.5, (255, 255, 255), 1)
            cv2.imshow("Distance Measurement Demo", frame)

            key = cv2.waitKey(30) & 0xFF
            if key == 27:          # Esc
                break
            elif key == 82:        # ↑
                current_distance = min(current_distance + 5.0, 200.0)
            elif key == 84:        # ↓
                current_distance = max(current_distance - 5.0, 10.0)

        cv2.destroyAllWindows()

    # ---------------------------------------------------------
    # 多距离对比视图
    # ---------------------------------------------------------
    def create_comparison_view(self) -> np.ndarray:
        distances = [20, 40, 60, 80, 100, 120]
        canvas_width, canvas_height = 1200, 800
        canvas = np.full((canvas_height, canvas_width, 3), 30, dtype=np.uint8)

        cols, rows = 3, 2
        sub_width = canvas_width // cols
        sub_height = canvas_height // rows

        for i, distance in enumerate(distances):
            row, col = divmod(i, cols)
            start_x, start_y = col * sub_width, row * sub_height

            sub_frame = self.create_demo_scene(distance)
            sub_frame = cv2.resize(sub_frame,
                                   (sub_width - 10, sub_height - 10))

            y_slice = slice(start_y + 5, start_y + sub_height - 5)
            x_slice = slice(start_x + 5, start_x + sub_width - 5)
            canvas[y_slice, x_slice] = sub_frame

            cv2.rectangle(canvas,
                          (start_x, start_y),
                          (start_x + sub_width, start_y + sub_height),
                          (100, 100, 100), 2)
        return canvas

    # ---------------------------------------------------------
    # 对比演示
    # ---------------------------------------------------------
    def run_comparison_demo(self) -> None:
        print("多距离对比演示")
        print("按任意键退出")
        comparison_frame = self.create_comparison_view()
        cv2.imshow("Distance Comparison Demo", comparison_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # ---------------------------------------------------------
    # 主菜单
    # ---------------------------------------------------------
    def run(self) -> None:
        print("=" * 50)
        print("距离测量演示工具")
        print("=" * 50)

        while True:
            print("\n请选择演示模式：")
            print("1. 交互式距离调整")
            print("2. 多距离对比显示")
            print("3. 退出")
            choice = input("请输入选择（1-3）：").strip()
            if choice == "1":
                self.run_distance_simulation()
            elif choice == "2":
                self.run_comparison_demo()
            elif choice == "3":
                break
            else:
                print("无效选择，请重新输入")


# ---------------------------------------------------------
# 入口
# ---------------------------------------------------------
if __name__ == "__main__":
    demo = DistanceDemo()
    demo.run()