#here we are gonna load a pcl file and gonna apply segmentation
#import rospy
import pcl
from pcl import pcl_visualization
cloud3 = pcl.PointCloud_PointXYZRGB()
import numpy as np

cloud = pcl.load_XYZRGB('/home/ribin/Downloads/object_02/test58.pcd')
#   print(cloud.shape)
cloud2 = np.zeros((307201,4),dtype=np.float32)
#cloud2 = np.array(cloud2)


def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.01, max_axis = 1):
  pass_filter = point_cloud.make_passthrough_filter()
  pass_filter.set_filter_field_name(name_axis);
  pass_filter.set_filter_limits(min_axis, max_axis)
  return pass_filter.filter()


def do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0):
    voxel_filter = point_cloud.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return voxel_filter.filter()

def do_ransac_plane_segmentation(point_cloud, max_distance = .001):

  segmenter = point_cloud.make_segmenter()


  segmenter.set_model_type(pcl.SACMODEL_PLANE)
  segmenter.set_method_type(pcl.SAC_RANSAC)
  segmenter.set_distance_threshold(max_distance)

  #obtain inlier indices and model coefficients
  inlier_indices, coefficients = segmenter.segment()

  inliers = point_cloud.extract(inlier_indices, negative = True)
  outliers = point_cloud.extract(inlier_indices, negative = False)

  return inliers, outliers


downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.01)
filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud,
name_axis = 'z', min_axis = 0.6, max_axis = 1.1)
#table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 0.01)
table_cloud, objects_cloud = do_ransac_plane_segmentation(cloud, max_distance = 0.01)



"""
print(cloud2)
count = 0


for i in cloud:
    #print(i)
    #cloud1=(i[0],i[1],i[2])
    count = count+1
    print(i)
    cloud2[count][0]=i[0]
    cloud2[count][1]=i[1]
    cloud2[count][2]=i[2]
    #cloud2[count][3] = i[3]



#print(count)
#print(cloud2[1])
cloud3.from_array(cloud2)

print(cloud2)
seg = cloud3.make_segmenter()


seg.set_optimize_coefficients (True)
# Mandatory
seg.set_model_type (pcl.SACMODEL_PLANE)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_distance_threshold (0.1)
inliers, model = seg.segment()
cloud3.from_array(cloud2)
"""





visual = pcl.pcl_visualization.CloudViewing()
while 1:


    #visual = pcl.pcl_visualization.CloudViewing()
   # visual.ShowColorCloud(cloud, b'cloud')
    #visual.ShowColorCloud(cloud,b'cloud')
    #visual.ShowMonochromeCloud(cloud)
    #visual.ShowColorCloud(downsampled_cloud)
    visual.ShowColorCloud(objects_cloud)
    #visual.ShowColorCloud(filtered_cloud)

"""  Loded point colud data and visualise it"""