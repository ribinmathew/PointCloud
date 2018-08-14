#this is a working program for cylinder segmentation

import pcl
from pcl import pcl_visualization

cloud = pcl.load("/home/ribin/Downloads/object_02/test58.pcd")
cloud1 = pcl.load_XYZRGB('/home/ribin/Downloads/object_02/test58.pcd')

print('PointCloud has: ' + str(cloud.size) + ' data points.')

"""
passthrough = cloud.make_passthrough_filter()
passthrough.set_filter_field_name ('z')
passthrough.set_filter_limits (0, 1.5)
cloud_filtered = passthrough.filter()
print('PointCloud has: ' + str(cloud_filtered.size) + ' data points.')
"""
# Estimate point normals
# ne.setSearchMethod (tree);
# ne.setInputCloud (cloud_filtered);
# ne.setKSearch (50);
# ne.compute (*cloud_normals);
#ne = cloud_filtered.make_NormalEstimation()
#tree = cloud_filtered.make_kdtree()
ne = cloud.make_NormalEstimation()
tree = cloud.make_kdtree()

ne.set_SearchMethod (tree)
ne.set_KSearch (50)
# cloud_normals = ne.compute ()

seg = cloud.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients (True)
seg.set_model_type (pcl.SACMODEL_NORMAL_PLANE)
seg.set_normal_distance_weight (0.1)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_max_iterations (100)
seg.set_distance_threshold (0.03)
# seg.set_InputNormals (cloud_normals)
[inliers_plane, coefficients_plane] = seg.segment ()
cloud_plane = cloud.extract(inliers_plane, False)

print('PointCloud representing the planar component: ' + str(cloud_plane.size) + ' data points.\n')
pcl.save(cloud_plane, 'table_scene_mug_stereo_textured_plane.pcd')

#cloud_filtered2 = cloud_filtered.extract(inliers_plane, True)
cloud_filtered2 = cloud.extract(inliers_plane, True)

seg = cloud_filtered2.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients (True)
seg.set_model_type (pcl.SACMODEL_CYLINDER)
seg.set_normal_distance_weight (0.1)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_max_iterations (10000)
seg.set_distance_threshold (0.05)
seg.set_radius_limits (0, 0.1)
# seg.set_InputNormals (cloud_normals2)
[inliers_cylinder, coefficients_cylinder] = seg.segment ()

cloud_cylinder = cloud_filtered2.extract(inliers_cylinder, False)
print(cloud_cylinder)
#
if cloud_cylinder.size == 0:
    print("Can't find the cylindrical component.")
else:
    print("PointCloud representing the cylindrical component: " + str(cloud_cylinder.size) + " data points.")
    #pcl.save(cloud_cylinder, 'table_scene_mug_stereo_textured_cylinder.pcd')


visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(cloud1)
#while 1:


    #visual = pcl.pcl_visualization.CloudViewing()
   # visual.ShowColorCloud(cloud, b'cloud')
 #   visual.ShowColorCloud(cloud1,b'cloud')
  #  visual.ShowMonochromeCloud(cloud_cylinder)
   # visual.ShowMonochromeCloud(cloud_cylinder)
   # visual.ShowColorCloud(cloud1)
    #visual.ShowColorCloud(cloud_cylinder)
    #visual.ShowColorCloud(filtered_cloud)

"""  Loded point colud data and visualise it"""