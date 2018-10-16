

import numpy as np
"""
p1 = np.array([10, 2, 3])
p2 = np.array([10,30, 4])
p3 = np.array([10, 4, 5])


# These two vectors are in the plane
v1 = p3 - p1
v2 = p2 - p1
print(v1,v2)
# the cross product is a vector normal to the plane
cp = np.cross(v1, v2)
print(cp)
a, b, c = cp

print(a,b,c)

# This evaluates a * x3 + b * y3 + c * z3 which equals d
d = np.dot(cp, p3)
print (d)

print('The equation is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d))

"""

from sklearn.cluster import KMeans
import pcl
from pcl import pcl_visualization
cloud3 = pcl.PointCloud_PointXYZRGB()
point_cloud  = pcl.PointCloud()
import numpy as np
import matplotlib.pyplot as plt



#cloud = pcl.load_XYZRGB('/home/ribin/Downloads/object_02/test58.pcd')
#path = "/home/ribin/Desktop/pcdfiles"


#cloud = pcl.load_XYZRGB('/media/ribin/DATA/addverb/binpicking/pcd_files/object_pcd/soap_santoor/2.pcd')
#cloud = pcl.load_XYZRGB('/media/ribin/DATA/addverb/binpicking/pcd_files/object_pcd/colgate/2.pcd')
cloud = pcl.load_XYZRGB('/media/ribin/DATA/addverb/binpicking/pcd_files/2.pcd')
print(cloud)
#   print(cloud.shape)
cloud2 = np.zeros((307201,4),dtype=np.float32)
#cloud2 = np.array(cloud2)


def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.01, max_axis = 1):
  pass_filter = point_cloud.make_passthrough_filter()
  pass_filter.set_filter_field_name(name_axis);
  pass_filter.set_filter_limits(min_axis, max_axis)
  return pass_filter.filter()


def do_voxel_grid_filter(point_cloud, LEAF_SIZE):
    voxel_filter = point_cloud.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return voxel_filter.filter()

def do_ransac_plane_segmentation(point_cloud, max_distance = .0009):

  segmenter = point_cloud.make_segmenter()
  segmenter.set_model_type(pcl.SACMODEL_PLANE)
  segmenter.set_method_type(pcl.SAC_RANSAC)
  segmenter.set_distance_threshold(max_distance)
  #obtain inlier indices and model coefficients
  inlier_indices, coefficients = segmenter.segment()
  inliers = point_cloud.extract(inlier_indices, negative = False)
  outliers = point_cloud.extract(inlier_indices, negative = True)

  return inliers, outliers


downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.006)
filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud,
name_axis = 'z', min_axis = 0.6, max_axis = 1.1)
#table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 0.01)
table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 0.01)


table_cloud, objects_cloud = do_ransac_plane_segmentation(objects_cloud, max_distance = 0.005)




objects_cloud_mono = []
"""
for i in range(0,objects_cloud.size):
    #print i[0]
    points = (objects_cloud[i][0],objects_cloud[i][1],objects_cloud[i][2])
   # print(points)
    #print(i)
   # print(objects_cloud_mono[0])
    objects_cloud_mono.append(points)
   # objects_cloud_mono[i][1]=[i][1]
    #objects_cloud_mono[i][2]= [i][2]

"""

for i in range(0,table_cloud.size):
    #print i[0]
    points = (table_cloud[i][0],table_cloud[i][1],table_cloud[i][2])
   # print(points)
    #print(i)
   # print(objects_cloud_mono[0])
    objects_cloud_mono.append(points)
   # objects_cloud_mono[i][1]=[i][1]
    #objects_cloud_mono[i][2]= [i][2]



#for i in objects_cloud_mono:
 #   print i
objects_cloud_mono = np.array(objects_cloud_mono,dtype=np.float32)
#print(objects_cloud_mono[0][0])
#print(objects_cloud_mono)

x = []
y = []
z=  []

for i in range(0,len(objects_cloud_mono)):
    x.append(objects_cloud_mono[i][0])
    y.append(objects_cloud_mono[i][1])
    z.append(objects_cloud_mono[i][2])





import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#x = np.linspace(-2, 14, 5)
#y = np.linspace(-2, 14, 5)
#X, Y = np.meshgrid(x, y)

#Z = (d - a * X - b * Y) / c


# plot the mesh. Each array is 2D, so we flatten them to 1D arrays
"""ax.plot(X.flatten(),
        Y.flatten(),
        Z.flatten(), 'bo ')
"""

ax.plot(x,y,z,'bo')
# plot the original points. We use zip to get 1D lists of x, y and z
# coordinates.
#ax.plot(*zip(p1, p2, p3), color='r', linestyle=' ', marker='o')

# adjust the view so we can see the point/plane alignment
ax.view_init(0, 22)
plt.tight_layout()

plt.show()








""" lets do 







visual = pcl.pcl_visualization.CloudViewing()
#while 1:


    #visual = pcl.pcl_visualization.CloudViewing()
 #   visual.ShowColorCloud(cloud, b'cloud')
    #visual.ShowColorCloud(cloud,b'cloud')
    #visual.ShowMonochromeCloud(objects_cloud_mono)
    #visual.ShowColorCloud(downsampled_cloud)
    #visual.ShowColorCloud(objects_cloud)
    #visual.ShowColorCloud(filtered_cloud)