from func import *
import numpy as np
import open3d as o3d
import os
import argparse
def create_points(rows=9, columns=6, spacing=0.0245,origin=[0,0,0],rotate_angle=[85,90,90]):
    points = np.zeros((rows*columns, 3))
    # x, y = np.meshgrid(np.linspace(0,-0.0245*(columns-1), columns), np.linspace(0, -0.0245*(rows-1), rows))
    x, y = np.meshgrid(np.linspace(0, -0.0245*(rows-1), rows),np.linspace(0,-0.0245*(columns-1), columns))
    points[:, :2] = np.stack((x.reshape(-1), y.reshape(-1)), axis=-1)
    # 根據原點調整點的位置
    points += np.array(origin)

    points = rotate_points(points, 'x', np.radians(rotate_angle[0]),origin)
    points = rotate_points(points, 'y', np.radians(rotate_angle[1]),origin)
    points = rotate_points(points, 'z', np.radians(rotate_angle[2]),origin)

    return points 
    
def rotate_points(points, axis, angle ,center):
    # 將點雲的坐標系移動到旋轉中心
    points_centered = points - center
    # 創建旋轉矩陣
    c, s = np.cos(angle), np.sin(angle)
    if axis == 'x':
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == 'y':
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == 'z':
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    # 將旋轉矩陣應用於每個點
    points_rot_centered = np.dot(points_centered, R.T)

    # 將點雲的坐標系移動回原來的位置
    points_rot = points_rot_centered + center
    return points_rot

def parse_args():
    parse = argparse.ArgumentParser()
    parse.add_argument('--pcd_path', type=str, default="./pcd/amr_1.pcd",choices=["./pcd/amr_1.pcd", "./pcd/a01.pcd","./pcd/new_1.pcd"])
    return parse.parse_args()

if __name__ == '__main__':
    args = parse_args()
    pcd_path=args.pcd_path
    pcd_name=pcd_path.split("/")[2][0:-4]
    save_path=os.path.join("./cal_file/",pcd_name)
 
    # 設定原始點雲 x、y、z三軸的範圍，範圍的格式為[min, max]
    x_range = [-4.5, 0]
    y_range = [-3, 2.5]
    z_range = [-3 ,1]
    # x_range = None
    # y_range = None
    # z_range = None

    org_pcd=read_edit_pt(pcd_path,color=[0, 0, 0],x_range = x_range,y_range = y_range,z_range = z_range)
    
    # 創建棋盤格坐標並設置顏色
    ### new_1 :
    if pcd_path=="./pcd/new_1.pcd":
        origin=[1.35,0.37,0.18]
        rotate_angle=[85,90,90]
    ### a01 :
    elif pcd_path=="./pcd/a01.pcd":
        origin=[0.654,0.25,0.36]
        rotate_angle=[95,45.5,90]
    ### amr_1 :
    elif pcd_path=="./pcd/amr_1.pcd":
        origin=[0.654,0.25,0.36]
        rotate_angle=[95,45.5,90]
    
    points = create_points(rows=9, columns=6, spacing=0.0245,origin=origin,rotate_angle=rotate_angle)
    
    create_path_if_not_exists(save_path)

    with open(os.path.join(save_path,pcd_name+"_org_rot.txt") , "w") as file:
        np.savetxt(file, origin,fmt='%.2f')
        np.savetxt(file, rotate_angle,fmt='%.2f')

    save_array(points, os.path.join(save_path,pcd_name+"_3dcorner.npy"))

    ## show pcd
    created_pcd = o3d.geometry.PointCloud()
    created_pcd.points = o3d.utility.Vector3dVector(points)
    colors = np.zeros((54, 3)) 
    colors[:] = [0, 0, 1]  # 將所有點設置為藍色
    colors[0] = [1, 0, 0]  # 將索引為 0 的點設置為紅色
    colors[5] = [0, 1, 0]  # 將索引為 5 的點設置為綠色
    pcd_ls=[org_pcd,created_pcd]
    created_pcd.colors = o3d.utility.Vector3dVector(colors)
    show_pcd(pcd_ls)



