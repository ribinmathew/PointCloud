"""lets create a working code"""

from sklearn.cluster import KMeans
import pcl
from pcl import pcl_visualization
cloud3 = pcl.PointCloud_PointXYZRGB()
point_cloud  = pcl.PointCloud()
import numpy as np

import matplotlib.pyplot as plt



#cloud = pcl.load_XYZRGB('/home/ribin/Downloads/object_02/test58.pcd')
#path = "/home/ribin/Desktop/pcdfiles"


cloud = pcl.load_XYZRGB('/media/ribin/DATA/bagfiles/pcdfiles/Zed_new_HQ/120.pcd')
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






objects_cloud_mono = []
for i in range(0,objects_cloud.size):
    #print i[0]
    points = (objects_cloud[i][0],objects_cloud[i][1],objects_cloud[i][2])
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


#for i in z:
 #   print(i)


#plt.scatter(y,z, c='black', s=7)


#plt.show(

kmeans = KMeans(n_clusters=3)
kmeans = kmeans.fit(objects_cloud_mono)
C = kmeans.cluster_centers_


print(C)




plt.scatter(x,y,color='r')
#plt.scatter(x,y,color='b')
plt.xlabel('x')
plt.ylabel('y')
plt.show()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.plot3D(x, y,z, zdir='z', c= 'red')
ax.scatter3D(x, y, z, c=z, cmap='Greens')


ax.scatter(C[:, 0], C[:, 1], C[:,2], marker='.', c='#050505', s=1000)
ax.set_title('surface')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')



plt.show()
#for i in objects_cloud_mono:
   # print(i)


"""


width = 1/1.5
plt.bar(x, y, width, color="blue")

plt.plot(x,y)
plt.xlabel('x - axis')
plt.ylabel('y - axis')

plt.title('My first graph!')

#function to show the plot
plt.show()


"""


#point_cloud.from_array(objects_cloud_mono)
#print(objects_cloud_mono)


#visual = pcl.pcl_visualization.CloudViewing()
#while 1:


    #visual = pcl.pcl_visualization.CloudViewing()
    #visual.ShowColorCloud(cloud, b'cloud')
    #visual.ShowColorCloud(cloud,b'cloud')
    #visual.ShowMonochromeCloud(objects_cloud_mono)
    #visual.ShowColorCloud(downsampled_cloud)
    #visual.ShowColorCloud(objects_cloud)
    #visual.ShowColorCloud(filtered_cloud)



