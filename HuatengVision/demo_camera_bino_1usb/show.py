import cv2


def list_cameras(max_id=10):
    ids = []
    for i in range(max_id):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            ids.append(i)
        cap.release()
    return ids

def main():
    # 根据你的C++代码设置参数
    camera_index = 0  # 你的代码使用索引1
    resolution_width = 640  # 双目总宽度
    resolution_height = 240  # 高度
    
    # 打开相机
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"错误: 无法打开相机 {camera_index}")
        print("请尝试修改 camera_index (0, 1, 2...)")
        return
    
    # 设置分辨率（部分相机可能不支持设置）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution_height)
    
    # 获取实际分辨率
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"相机实际分辨率: {actual_width}×{actual_height}")
    
    # 如果分辨率不符合预期，尝试其他常见值
    if actual_width != resolution_width or actual_height != resolution_height:
        print("警告: 实际分辨率与预期不符，尝试自动调整...")
        if actual_width > 0 and actual_height > 0:
            resolution_width = actual_width
            resolution_height = actual_height
    
    # 单目画面宽度
    single_width = resolution_width // 2
    
    print("\n操作说明：")
    print("  按 'q' 键退出预览")
    print("  按 's' 键保存当前左右画面到当前目录")
    print("  按 'ESC' 键退出\n")
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("错误: 无法读取帧")
            break
        
        # 检查帧的尺寸
        if frame.shape[1] != resolution_width:
            print(f"警告: 帧宽度{frame.shape[1]}与预期{resolution_width}不符")
        
        # 分割左右画面（水平分割）
        left_frame = frame[:, :single_width]   # 左画面
        right_frame = frame[:, single_width:]  # 右画面
        
        # 检查画面是否有效
        if left_frame.size == 0 or right_frame.size == 0:
            print("错误: 分割后的画面为空，请检查分辨率设置")
            break
        
        # 显示画面
        cv2.imshow('Left Camera (左相机)', left_frame)
        cv2.imshow('Right Camera (右相机)', right_frame)
        
        # 等待按键
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q') or key == 27:  # 'q' 或 ESC
            print("退出预览")
            break
        elif key == ord('s'):
            # 保存画面
            cv2.imwrite('left_image.png', left_frame)
            cv2.imwrite('right_image.png', right_frame)
            print("已保存 left_image.png 和 right_image.png")
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print('枚举相机 …')
    cams = list_cameras()
    print('可用序号：', cams)
    main()