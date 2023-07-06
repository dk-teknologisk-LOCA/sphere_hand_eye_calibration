from numpy import asarray, mgrid, pi, cos, sin, load, array
import numpy
import matplotlib.pyplot as plt
import open3d as o3d
import glob
from utils.sphere import Sphere



def fit_sphere_and_compare(sphere_point_cloud):

    
    return center, radius

def get_depth_map():
    namefile_depth = glob.glob("data/direct_data/imgs/*/*depth*")[0]
    depth = o3d.geometry.Image(load(namefile_depth))
    o3d.visualization.draw_geometries([depth])
    return depth

def get_ir_map():
    namefile_ir = glob.glob("data/direct_data/imgs/*/*ir*")[0]
    ir = o3d.geometry.Image(load(namefile_ir).astype(numpy.uint16))
    o3d.visualization.draw_geometries([ir])
    return ir

def get_colour_map():
    namefile_colour = glob.glob("data/direct_data/imgs/*/*color*")[0]
    colour = o3d.io.read_image(namefile_colour)
    o3d.visualization.draw_geometries([colour])
    return colour

def get_pcd_from_pcd():
    namefile_pc = glob.glob("data/direct_data/pointcloud/*/*")[0]
    pc = load(namefile_pc)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pc)
    o3d.visualization.draw_geometries([pcd2])

def get_pcd_from_colour_and_depth(color_image, depth_image):
    cam = o3d.camera.PinholeCameraIntrinsic()
    #cam.intrinsic_matrix = [[3131.58, 0.00, 1505.62], 
    #                        [0.00, 3131.58, 2004.13], 
    #                        [0.00, 0.00,    1.00]]
    cam.intrinsic_matrix =  [[1959.37890625, 0.00, 0.0] , 
                            [0.00, 1958.880126953125, 0.0], 
                            [2044.2911376953125, 1565.8837890625, 1.00]]
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(colour_im, depth_im)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)
    pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    return pcd

def fit_sphere(point_cloud):
    #cut off some points to isolate a bit the sphere
    xyz = asarray(point_cloud.points)
    xyz = xyz[xyz[:,2] > -0.60]
    xyz = xyz[xyz[:,2] < -0.30, :]

    #recreate another point cloud, but trimmed
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.visualization.draw_geometries([pcd])

    #Fit the sphere
    true_radius = 0.150/2.
    sph = Sphere()
    center, radius, inliers = sph.fit(pcd, 
                                      thresh=0.1)
    print(center, radius)
    return xyz, center, radius

def plot_point_cloud(xyz):
    #cut off some points for easy plotting
    xyz = xyz[xyz[:,2] > -0.60]
    xyz = xyz[xyz[:,2] < -0.35, :]
    xyz = xyz[xyz[:,0] > 0.05]
    xyz = xyz[xyz[:,0] < 0.25, :]
    xyz = xyz[xyz[:,1] > -0.30, :]
    xyz = asarray([array(xyz[:,0])[::10], array(xyz[:,1])[::10], array(xyz[:,2])[::10]]).T

    fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d'})
    #ax.plot_wireframe(x, y, z, color='k', rstride=1, cstride=1)
    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=1, c='r', zorder=10)
    ax.set_xlabel("x"), ax.set_ylabel("y"), ax.set_zlabel("z")
    #ax.set_aspect('equal')
    ax.set_xlim((0, 0.4))
    ax.set_ylim((-0.05, -0.45))
    ax.set_zlim((-0.35, -0.75))
    return ax

depth_im = get_depth_map()
colour_im = get_colour_map()
pcd = get_pcd_from_colour_and_depth(colour_im, depth_im)
xyz, center, radius = fit_sphere(pcd)
ax = plot_point_cloud(xyz)

u, v = mgrid[0:2 * pi:30j, 0:pi:20j]
x = radius * cos(u) * sin(v)
y = radius * sin(u) * sin(v)
z = radius * cos(v)
ax.plot_surface(x + center[0], y + center[1], z + center[2])
ax.set_xlabel("x"), ax.set_ylabel("y"), ax.set_zlabel("z")

#o3d.geometry.create_mesh_sphere(true_radius, resolution = 20)
plt.show()

