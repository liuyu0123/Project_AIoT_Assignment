from PIL import Image
from ultralytics import YOLO
model = YOLO("yolo11n.pt")
results = model(["https://ultralytics.com/images/bus.jpg", "https://ultralytics.com/images/zidane.jpg"])
for i, r in enumerate(results):
    im_bgr = r.plot() 
    im_rgb = Image.fromarray(im_bgr[..., ::-1])  
    try:
        im_rgb.show()
    except Exception as e:
        print(f"无法显示图像: {e}")
    r.save(filename=f"results{i}.jpg")