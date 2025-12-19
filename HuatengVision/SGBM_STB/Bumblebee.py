#左右相机仅相差个平移参数baseline，旋转忽略
import numpy as np

fx = 822.79041
fy = 822.79041
tx = 318.47345
ty = 250.31296
base = 120.054
#增广矩阵计算方便
R_l = np.asarray([
  [1,0,0,0],
  [0,1,0,0],
  [0,0,1,0]])
R_r = R_l.copy()
R_r[0, 3] = -base #作为平移参数
#内参矩阵
K = np.asarray([
  [fx,0,tx],
  [0,fy,ty],
  [0,0,1]])

#世界坐标系点，4*21矩阵，[x,y,z,1]增广矩阵，计算方便
points = XXX

#平移+内参
left_point = np.dot(np.dot(K , R_l), points)
right_point = np.dot(np.dot(K , R_r), points)

#消除尺度z
image_cood = left_point / left_point[-1, ...]
image_left = (image_cood[:2,...].T).astype(np.uint)
image_cood = right_point / right_point[-1, ...]
image_right = (image_cood[:2,...].T).astype(np.uint)
