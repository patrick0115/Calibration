from func import *
import open3d as o3d

# Parameter  and file
square_size = 0.0215
pattern_size = (9, 6)
# extra_img_path= "./img/new_1.jpg"
extra_img_path= "./img/a01.jpg"
img_name=extra_img_path.split("/")[2][0:-4]
save_path= os.path.join('./cal_file/',img_name)
corner_points_np = load_array(os.path.join(save_path,img_name+'_3dcorner.npy'))
img = cv2.imread(extra_img_path)


# 1. Loadmtx, dist
mtx, dist = load_txt('cal_file.txt')
print('Camera matrix (mtx) : \n', mtx)
print('Distortion coefficient (dist) : \n', dist)

# 2. Find rvecs, tvecs 
rvecs, tvecs = find_pose(extra_img_path, mtx, dist, square_size, pattern_size,corner_points_np)
print('Rotation Vectors : \n', rvecs)
print('Translation Vectors : \n', tvecs)
RT ,R , T=vec_to_mat(rvecs, tvecs )

with open(os.path.join(save_path,img_name+"_extrinsic.txt") , "w") as file:
    np.savetxt(file, rvecs,fmt='%.8f')
    np.savetxt(file, tvecs,fmt='%.8f')







