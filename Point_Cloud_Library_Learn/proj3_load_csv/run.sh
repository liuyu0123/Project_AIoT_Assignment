csv_path=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/0000000000.csv
pcd_path=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/0000000000.pcd
python3 csv2pcd.py $csv_path $pcd_path

point_color_white=255,255,255
point_size=2
pcl_viewer $pcd_path -fc $point_color_white -ps $point_size


