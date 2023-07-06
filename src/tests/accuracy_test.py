import matplotlib.pyplot as plt
from numpy import (asarray, linalg, where, random, delete, 
                    mean, mgrid, pi, cos, sin)
import src.sphere as sphere
import random as rnd

"""
Initial test software to get an idea of the error margins
Initial test are done ith fabricated point clouds in the 
case of full point cloud and then replicating more accurately 
what the camera would see. In addition, the same test is run 
n_test_runs time with different observer position to get an 
average error statistic. To be noted that the sphere diameter 
to be predicted is improsed withh the true, known, diameter of the sphere.
""" 

#Position of the camera
global obs_pos, true_center, true_radius, pillar_radius, pillar_length
obs_pos = asarray([1, -2/3, 2/3])
true_center = asarray([0,0,0])
true_radius = 0.150/2.
pillar_radius = 0.04493/2.
pillar_length = 0.035
n_test_runs = 100


def get_closest_point(object_point_cloud):
    global obs_pos
    distances = []
    for i in sphere_pc:
        distances.append(linalg.norm(i-obs_pos))
    return sphere_pc[where(distances==min(distances))[0][0], :]

def get_point_cloud(n_points, add_cylinder = False):
    x = []
    y = []
    z = []
    for i in range(n_points):
        u = random.normal(0,1) * true_radius
        v = random.normal(0,1) * true_radius
        w = random.normal(0,1) * true_radius
        norm = (u*u + v*v + w*w)**(0.5)
        #xi,yi,zi = u/norm,v/norm,w/norm
        xi,yi,zi = u,v,w
        if norm < true_radius + 0.01 and norm > true_radius - 0.01:
            x.append(xi)
            y.append(yi)
            z.append(zi)

    if add_cylinder:
        for i in range(n_points):
            y_temp = random.random()-1-true_radius
            if y_temp < - true_radius - pillar_length:
                continue
            r = pillar_radius
            theta = random.random()*360
            x.append(r*cos(theta))
            y.append(y_temp)
            z.append(r*sin(theta))

    pos_sphere = asarray([x, y, z])
    
    return pos_sphere.T

def plot_pc(sphere_points, plot_observer = False):
    global obs_pos
    fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d'})
    #ax.plot_wireframe(x, y, z, color='k', rstride=1, cstride=1)
    ax.scatter(sphere_points[:,0], sphere_points[:,1], sphere_points[:,2], 
                                                    s=1, c='r', zorder=10)
    ax.set_xlabel("x"), ax.set_ylabel("y"), ax.set_zlabel("z")
    if plot_observer:
        ax.scatter(obs_pos[0], obs_pos[1], obs_pos[2])
    ax.set_aspect('equal')

    return fig, ax

def fit_sphere_and_compare(sphere_point_cloud, return_data = False):
    global true_center, true_radius
    sph = sphere.Sphere()
    center, radius, inliers = sph.fit(sphere_point_cloud, thresh=0.1, 
                                      radius_known = true_radius)
    center_err = linalg.norm(asarray(true_center) - asarray(center))
    radius_err = asarray(true_radius) - asarray(radius)
    if not return_data:
        print("Error on center position: ", center_err, 
              "\nError on radius: ", radius_err)
    if return_data:
        return center_err, radius_err, center, radius

#Get point cloud
sphere_pc = get_point_cloud(2000, True)
fig1, ax1 = plot_pc(sphere_pc, True)

#Find closest point
closest_point = get_closest_point(sphere_pc)

#Call the RANSAC software on full sphere and compare
print("Full sphere\n")
fit_sphere_and_compare(sphere_pc)

#now it must be tested with partial spherical data (only one "side")
rem_idx = []
for i, val in enumerate(sphere_pc):
    if val[0] < 0.0:
        rem_idx.append(True)
    else:
        rem_idx.append(False)
half_sphere = delete(sphere_pc, rem_idx, 0)

fig2 , ax2 = plot_pc(half_sphere, True)

print("Half sphere\n")
fit_sphere_and_compare(half_sphere, True)

#Now, insert the cylinder in the point cloud
print("Adding the cylinder\n")
pc = get_point_cloud(2000, True)
fig3, ax3 = plot_pc(pc, True)

"""#Do it once
rem_idx = []s
for i, val in enumerate(pc):
    if val[0] < 0:
        rem_idx.append(True)
    else:
        rem_idx.append(False)
half_pc = delete(pc, rem_idx, 0)

fig = plot_pc(half_pc, True)

fit_sphere_and_compare(half_pc)"""

#Do it more than once
center_errors = []
radius_errors = []
for i in range(n_test_runs):
    obs_pos = [rnd.uniform(-10, 10), rnd.uniform(-10, 10), rnd.uniform(-10, 10)]
    norm = linalg.norm(obs_pos)
    dx = (obs_pos[0] - true_center[0]) / norm
    dy = (obs_pos[1] - true_center[1]) / norm
    dz = (obs_pos[2] - true_center[2]) / norm

    """x_plane = linspace(-1, 1, 100)
    y_plane = linspace(-1, 1, 100)
    x_plane, y_plane = meshgrid(x_plane, y_plane)
    eq = - dx/dz * x_plane - dy/dz * y_plane + 0"""

    rem_idx = []
    for i, val in enumerate(pc):
        if val[2] > (- dx/dz * val[0] - dy/dz * val[1] + 0):
            rem_idx.append(True)
        else:
            rem_idx.append(False)
    cut_pc = delete(pc, rem_idx, 0)
    
    """#Plot all the middle trials. 
    PAY ATTENTION TO THE NUMBER OF FIGURES THAT YOU'RE ABOUT TO OPEN
    fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d'})
    ax.scatter(cut_pc[:, 0], cut_pc[:, 1], cut_pc[:, 2], s=1, c='r', zorder=10)
    ax.set_xlabel("x"), ax.set_ylabel("y"), ax.set_zlabel("z")
    ax.scatter(obs_pos[0], obs_pos[1], obs_pos[2])
    ax.set_aspect('equal')"""

    #plot_sphere(cut_pc, True)
    center_err, radius_err, center, radius = fit_sphere_and_compare(cut_pc, True)
    center_errors.append(center_err)
    radius_errors.append(radius_err)

print("Mean center error: ", mean(center_errors), 
    "\nMean radius error", mean(radius_errors))
#fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d'})
#ax.plot_wireframe(x, y, z, color='k', rstride=1, cstride=1)
r = radius
u, v = mgrid[0:2 * pi:30j, 0:pi:20j]
x = cos(u) * sin(v) - center[0]
y = sin(u) * sin(v) - center[1]
z = cos(v) - center[2]
ax3.plot_surface(x * true_radius, y * true_radius, z * true_radius)
ax3.set_xlabel("x"), ax3.set_ylabel("y"), ax3.set_zlabel("z")

plt.show()