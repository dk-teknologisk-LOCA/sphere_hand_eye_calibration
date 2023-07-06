import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from utils import sphere

pc_data_dic = {'OG': ['data/direct_data/pointcloud/05062023/kinect_20230605_122349_points2_1.npy', -0.1, 0.5, 0.0, -1.0, 2.0, 0.0],  # noqa: E501
            'painted': ['data/direct_data/pointcloud/23062023/kinect_20230623_131252_points2_1.npy', -0.3, 0.3, 0.170, -0.2, 1.5, 0.95],  # noqa: E501
            'painted creaform': ['data/direct_data/pointcloud/23062023/kinect_20230623_150100_points2_2.npy', -0.22, 0.1, 0.2, 0.0, 1.0, 0.4],  # noqa: E501
            'bad angle painted creaform': ['data/direct_data/pointcloud/26062023/kinect_20230626_112102_points2_1.npy', -0.1, 0.18, 0.0, -0.2, 1.05, 0.75],  # noqa: E501
            'centered at 100 0 5': ['data/direct_data/pointcloud/26062023/kinect_20230626_130250_points2_2.npy', -0.2, 0.1, 0.15, -0.1, 0.7, 0.5], # noqa: E501
            'al robot': ['data/direct_data/pointcloud/26062023/kinect_20230626_162521_points2_3.npy', -0.2, 0.1, 0.15, -0.1, 0.7 , 0.5],  # noqa: E501
            'robot_test_1': ['data/direct_data/pointcloud/29062023/kinect_20230629_164559_points2_1.npy', -0.2, 0.2, 0.1, -0.1], # noqa: E501
            'robot_test_2': ['data/direct_data/pointcloud/29062023/kinect_20230629_164600_points2_2.npy', -0.2, 0.2, 0.1, -0.1]} # noqa: E501

imgs_data_dic = {'OG': ['data/direct_data/imgs/05062023/kinect_20230605_122347_color_1.jpg', 'data/direct_data/imgs/05062023/kinect_20230605_122347_depth_1.npy', 'data/direct_data/imgs/05062023/kinect_20230605_122347_ir_1.npy'], # noqa: E501
            'painted': ['data/direct_data/imgs/23062023/kinect_20230623_131252_color_1.jpg', 'data/direct_data/imgs/23062023/kinect_20230623_131252_depth_1.npy', 'data/direct_data/imgs/23062023/kinect_20230623_131252_ir_1.npy'], # noqa: E501
            'painted creaform': ['data/direct_data/imgs/23062023/kinect_20230623_150101_color_2.jpg', 'data/direct_data/imgs/23062023/kinect_20230623_150101_depth_2.npy', 'data/direct_data/imgs/23062023/kinect_20230623_150101_ir_2.npy'], # noqa: E501
            'bad angle painted creaform': ['data/direct_data/imgs/26062023/kinect_20230626_112102_color_1.jpg', 'data/direct_data/imgs/26062023/kinect_20230626_112102_depth_1.npy', 'data/direct_data/imgs/26062023/kinect_20230626_112102_ir_1.npy'], # noqa: E501
            'centered at 100 0 5': ['data/direct_data/imgs/26062023/kinect_20230626_130249_color_2.jpg', 'data/direct_data/imgs/26062023/kinect_20230626_130249_depth_2.npy', 'data/direct_data/imgs/26062023/kinect_20230626_130249_ir_2.npy'], # noqa: E501
            'al robot': ['data/direct_data/imgs/26062023/kinect_20230626_162520_color_3.jpg', 'data/direct_data/imgs/26062023/kinect_20230626_162520_depth_3.npy', 'data/direct_data/imgs/26062023/kinect_20230626_162520_ir_3.npy']} # noqa: E501

def clustering():
    #labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10))
    #max_label = labels.max()
    #colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    #colors[labels < 0] = 0
    #pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return None

def cut_pcd_temp(xyz, data_dict, downsample = 0):
    if downsample:
        # Downsample the points
        xyz = np.asarray([np.array(xyz[:,0])[::100], 
            np.array(xyz[:,1])[::100], np.array(xyz[:,2])[::100]]).T

    #for data 05062023
    # Adjust x
    xyz = xyz[xyz[:,0] > data_dict[1]]
    xyz = xyz[xyz[:,0] < data_dict[2]]
    # Adjust y
    xyz = xyz[xyz[:,1] < data_dict[3]]
    xyz = xyz[xyz[:,1] > data_dict[4]]

    return xyz

#load and downsize NUMPY point cloud
data_name = 'robot_test_2'
namefile_pc = pc_data_dic[data_name][0]
#namefile_imgs = imgs_data_dic[data_name]

print(namefile_pc)
xyz = np.load(namefile_pc)
print(xyz)
xyz = cut_pcd_temp(xyz, pc_data_dic[data_name], downsample = 0)
#transform into o3d point cloud and process it
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
#pcd = pcd.uniform_down_sample(every_k_points=40)
#R = pcd.get_rotation_matrix_from_xyz((np.pi, 0, 0))
#pcd = pcd.rotate(R, center=(0,0,0))
pcd, ind = pcd.remove_radius_outlier(nb_points=40, radius=0.005)
#o3d.visualization.draw_geometries([pcd])

#create sphere object and fit it to the point cloud
sph = sphere.Sphere()
#lower threshold of inlier makes the  algoithm "harsher", it means it will exclude 
#closer points from the sequentially determined "hull" (surface of the sphere).
center, radius, inliers = sph.fit(np.asarray(pcd.points), thresh=0.0001, radius_known = 0.15/2.) # noqa: E501
#Output center of the fitted sphere and its radius
print("Center of the fitted sphere ", np.dot(center, 1000), " and its diameter ", radius * 2, " with error ", (radius * 2 - 0.15)*1000, " mm") # noqa: E501

#Plot point cloud and the fitted sphere
extr_xyz = np.asarray(pcd.points)
fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d'})
ax.scatter(extr_xyz[:,0], extr_xyz[:,1], extr_xyz[:,2], s=1, c='r', zorder=10)
ax.scatter(center[0], center[1], center[2], c = 'b')
ax.set_xlabel("x"), ax.set_ylabel("y"), ax.set_zlabel("z")
u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
x = radius * np.cos(u) * np.sin(v)
y = radius * np.sin(u) * np.sin(v)
z = radius * np.cos(v)
ax.plot_surface(x + center[0], y + center[1], z + center[2], alpha = 0.1)
ax.set_aspect('equal')
plt.show()


