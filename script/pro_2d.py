import cv2
import numpy as np
import open3d as o3d
from func import *

def load_image_and_point_cloud(img_path, pcd_path):
    img = cv2.imread(img_path)
    pcd_np = pcd_to_numpy(pcd_path)
    return img, pcd_np

def project_points(point_cloud, mtx, dist, rvecs, tvecs,cv):
    if cv:
        projected_points, _ = cv2.projectPoints(point_cloud, rvecs, tvecs, mtx,dist)
        projected_points = np.squeeze(projected_points, axis=1)
    else:
        R, _ = cv2.Rodrigues(rvecs)
        RT = np.column_stack((R, tvecs))
        P = np.dot(mtx, RT)
        point_cloud_homogeneous = np.column_stack((point_cloud, np.ones(point_cloud.shape[0])))
        projected_points = np.dot(P, point_cloud_homogeneous.T).T
        projected_points = projected_points[:, :2] / projected_points[:, 2, np.newaxis]

    return projected_points

def project_2points(point_cloud, mtx, dist, rvecs, tvecs,cv):
    ## Use CV
    projected_points_cv, _ = cv2.projectPoints(point_cloud, rvecs, tvecs, mtx,dist)
    projected_points_cv = np.squeeze(projected_points_cv, axis=1)
    ## Use Project
    R, _ = cv2.Rodrigues(rvecs)
    RT = np.column_stack((R, tvecs))
    P = np.dot(mtx, RT)
    point_cloud_homogeneous = np.column_stack((point_cloud, np.ones(point_cloud.shape[0])))
    projected_points = np.dot(P, point_cloud_homogeneous.T).T
    projected_points = projected_points[:, :2] / projected_points[:, 2, np.newaxis]

    return projected_points,projected_points_cv

def create_img(image, points,word):
    cv2.putText(image, word, (80, 450), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0), 2, cv2.LINE_AA)

    points = np.round(points).astype(int)
    for pt in points:
        cv2.circle(image, tuple(pt), 3, (0, 0, 255), -1)
    return image

def draw_image(image):
    cv2.imshow('Image with Points', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



if __name__ == '__main__':
# Load data
    img_path = "./img/a01.jpg"
    pcd_path = "./pcl/a01.pcd"
    img_path = "./img/new_1.jpg"
    pcd_path = "./pcl/new_1.pcd"
    img, pcd_np = load_image_and_point_cloud(img_path, pcd_path)
    img_name = pcd_path.split("/")[2][0:-4]
    mtx, dist = load_txt('cal_file.txt')
    rvecs, tvecs = load_rvecs_tvecs(os.path.join("cal_file", img_name, img_name+'_extrinsic.txt'))
    img_copy = np.copy(img)
    # Project points
    # points_2d = project_points(pcd_np, mtx, dist, rvecs, tvecs,True)
    points_2d,points_2d_cv= project_2points(pcd_np, mtx, dist, rvecs, tvecs,True)
    img1=create_img(img,points_2d,"Use cv2.projectPoints")
    img2=create_img(img_copy,points_2d_cv,"Project")
    img3 = np.hstack((img1, img2))
    draw_image(img3)


    # 假設你已經有了點雲和2D點的坐標
    points_2d = np.round(points_2d).astype(int)  # 取整數像素坐標
    img = cv2.imread(img_path)
    height, width = img.shape[:2]
    mask = (0 <= points_2d[:, 0]) & (points_2d[:, 0] < width) & (0 <= points_2d[:, 1]) & (points_2d[:, 1] < height)


    # 從圖像中提取顏色
    colors = np.zeros_like(pcd_np)  # 創建一個與點雲相同大小的0數組，用來儲存顏色
    colors[mask] = img[points_2d[mask, 1], points_2d[mask, 0]]  # 只對影像範圍內的點設定顏色
    colors = colors / 255  # 歸一化到[0, 1]

    # 建立點雲並設置顏色
    # 建立點雲並設置顏色
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_np)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    # 顯示彩色點雲
    o3d.visualization.draw_geometries([pcd])