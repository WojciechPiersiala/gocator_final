import numpy as np
from scipy.spatial.transform import Rotation as R

import open3d as o3d

def crop_cloud(pcd):
    num_points = len(pcd.points)
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[10:, :])
    num_points = len(pcd.points)
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[:num_points - 10, :])
    return pcd


def get_deviation(C_transform, R_transformforms, point_clouds):
    R_N = R_transformforms
    C = C_transform
    p_N = point_clouds


    transformed_points = []
    transformed_points_z = []
    
    iterator = 0
    for p_n in p_N:
        points = np.asarray(p_n.points)
        for p_i in points:

            tmp_R = R_N[iterator]
            tmp_C = C
            tmp_point = p_i

            tmp_point_hom= np.append(tmp_point, 1)
            tmp_point_result= np.dot(tmp_R, np.dot(tmp_C, tmp_point_hom))
            tmp_point_transform = tmp_point_result[:3] / tmp_point_result[3]

            transformed_points.append(tmp_point_transform)
        iterator += 1

    for p in transformed_points:
        transformed_points_z.append(p[2])

    var = np.var(transformed_points_z)

    transformed_points_arr = np.array(transformed_points)
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(transformed_points_arr)
    o3d.visualization.draw_geometries([new_pcd])
    return var

def main():
    file1 = open('../surface_cld/transforms.txt', 'r')
    Lines = file1.readlines()

    R_transformforms = []
    point_clouds = []
    C_transform_translation = [0, 0, 0]
    C_transform_quaternion = R.from_quat([0.5, 0.5, 0.5, 0.5])
    C_transform= np.eye(4)
    C_transform[:3, :3] = C_transform_quaternion.as_matrix()
    C_transform[:3, 3] = C_transform_translation

    for line in Lines:
        data = line.split(';')
        tr_x = data[1]
        tr_y = data[2]
        tr_z = data[3]

        rot_x = data[4]
        rot_y = data[5]
        rot_z = data[6]
        rot_w = data[7]

        pcl_path = data[8].strip()
        point_cloud = o3d.io.read_point_cloud(pcl_path,format='pcd')

        rotation = R.from_quat([rot_x, rot_y, rot_z, rot_w])
        translation = [tr_x, tr_y, tr_z]

        R_transform = np.eye(4)
        R_transform[:3, :3] = rotation.as_matrix()
        R_transform[:3, 3] = translation

        R_transformforms.append(R_transform)
        point_clouds.append(crop_cloud(point_cloud))
        print(R_transform)

    point_array = np.asarray(point_clouds[0].points)
    print(len(point_array))
    o3d.visualization.draw_geometries(point_clouds)

if __name__ == "__main__":
    main()