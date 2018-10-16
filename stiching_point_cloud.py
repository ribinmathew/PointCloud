#here we are gonna try to stich point cloud which going get from the lazer to generate full point cloud
# so we are gonna load 2 pcl and stich it together


import pcl
from pcl import pcl_visualization
import numpy as np
<<<<<<< HEAD

from pykdtree.kdtree import KDTree
import matplotlib.pyplot as plt
cloud5 = pcl.PointCloud_PointXYZRGB()




=======
import matplotlib.pyplot as plt
cloud5 = pcl.PointCloud_PointXYZRGB()

>>>>>>> origin/master
#hwe we are gonna load one point cloud
cloud1 = pcl.load_XYZRGB('/media/ribin/DATA/addverb/binpicking/pcd files/object_pcd/soap_santoor/2.pcd')
print(cloud1)
#here we are gonna load the second point cloud
cloud2 = pcl.load_XYZRGB('/media/ribin/DATA/addverb/binpicking/pcd files/object_pcd/colgate/2.pcd')


array_size1 = cloud1.size
array_size2 = cloud2.size
array_size = array_size1 + array_size2 + 10
cloud3 = np.zeros((array_size,4),dtype=np.float32)
cloud2_size =cloud2.size

print(cloud2_size)

current_position = 0

for i in range(0,cloud1.size):
      #print(i)
      current_position = i
      cloud3[current_position][0] = cloud1[i][0]+0.54
      cloud3[current_position][1] = cloud1[i][1]
      cloud3[current_position][2] = cloud1[i][2]
      cloud3[current_position][3] = 255 << 16 | 255 << 8 | 255




for j in range(0,cloud2_size-200):
    current_position +=1
    #print(j)
    cloud3[current_position] = cloud2[j]
    cloud3[current_position][0] = cloud2[j][0]
    cloud3[current_position][1] = cloud2[j][1]
    cloud3[current_position][2] = cloud2[j][2]
    cloud3[current_position][3] =  255 << 16 | 255 << 8 | 255



cloud5.from_array(cloud3)


<<<<<<< HEAD
def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.01, max_axis = 1.1):
  pass_filter = point_cloud.make_passthrough_filter()
  pass_filter.set_filter_field_name(name_axis);
  pass_filter.set_filter_limits(min_axis, max_axis)
  return pass_filter.filter()


def do_ransac_plane_segmentation(point_cloud, max_distance = .00095):

  segmenter = point_cloud.make_segmenter()
  segmenter.set_model_type(pcl.SACMODEL_PLANE)
  segmenter.set_method_type(pcl.SAC_RANSAC)
  segmenter.set_distance_threshold(max_distance)
  #obtain inlier indices and model coefficients
  inlier_indices, coefficients = segmenter.segment()
  inliers = point_cloud.extract(inlier_indices, negative = False)
  outliers = point_cloud.extract(inlier_indices, negative = True)

  return inliers, outliers


cloud6 = do_passthrough_filter(cloud5, min_axis = 0.6)
table_cloud, objects_cloud = do_ransac_plane_segmentation(cloud6, max_distance = 0.016)



cloud_with_out_rgb = np.zeros((array_size,3),dtype=np.float32)

for i in range(0,objects_cloud.size):
    cloud_with_out_rgb[i][0] = objects_cloud[i][0]
    cloud_with_out_rgb[i][1] = objects_cloud[i][1]
    cloud_with_out_rgb[i][2] = objects_cloud[i][2]



kdtree = KDTree(cloud_with_out_rgb.astype(np.float32))
dist, idx = kdtree.query(cloud_with_out_rgb, k=2)

#for i in range(0,idx.size):
   # print(i)

print(cloud_with_out_rgb.size)
print(idx.size)
#for j in range(0,dist.size):
 #   print(j)
print("reached")
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(objects_cloud)
=======
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(cloud5)
>>>>>>> origin/master


v = True
while v:
    v=not(visual.WasStopped())
