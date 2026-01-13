import cv2

stereo_image_path = r"D:\Files\Data\StereoCamera\ImageStereo\RiverImage\1000\sync_20251227_122319_463.png"
stereo_image = cv2.imread(stereo_image_path)
cv2.imshow('stereo_image', stereo_image)
cv2.waitKey(0)