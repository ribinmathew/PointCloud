#this is a working code for plane segmentation
import pcl
import numpy as np
import random
from pcl import pcl_visualization



cloud = pcl.load("/home/ribin/Downloads/mug.pcd")


#cloud = pcl.PointCloud()

#points = np.zeros((), dtype=np.float32)
#RAND_MAX = 1024.0
#for i in range(0, 15):
 #   points[i][0] = 1024 * random.random () / (RAND_MAX + 1.0)
  #  points[i][1] = 1024 * random.random () / (RAND_MAX + 1.0)
   # points[i][2] = 1.0

#points[0][2] =  2.0
#points[3][2] = -2.0
#points[6][2] =  4.0

#cloud.from_array(points)

#   std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
#   for (size_t i = 0; i < cloud->points.size (); ++i)
#     std::cerr << "    " << cloud->points[i].x << " "
#                         << cloud->points[i].y << " "
#                         << cloud->points[i].z << std::endl;
#
#print ('Point cloud data: ' + str(cloud.size) + ' points')
#for i in range(0, cloud.size):
 #   print ('x: '  + str(cloud[i][0]) + ', y : ' + str(cloud[i][1])  + ', z : ' + str(cloud[i][2]))

#   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
#   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
#   // Create the segmentation object
#   pcl::SACSegmentation<pcl::PointXYZ> seg;
#   // Optional
#   seg.setOptimizeCoefficients (true);
#   // Mandatory
#   seg.setModelType (pcl::SACMODEL_PLANE);
#   seg.setMethodType (pcl::SAC_RANSAC);
#   seg.setDistanceThreshold (0.01);
#
#   seg.setInputCloud (cloud);
#   seg.segment (*inliers, *coefficients);
###
# http://www.pcl-users.org/pcl-SACMODEL-CYLINDER-is-not-working-td4037530.html
# NG?
# seg = cloud.make_segmenter()
# seg.set_optimize_coefficients(True)
# seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
# seg.set_method_type(pcl.SAC_RANSAC)
# seg.set_distance_threshold(0.01)
# indices, coefficients = seg.segment()
seg = cloud.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_distance_threshold(0.01)
seg.set_normal_distance_weight(0.01)
seg.set_max_iterations(100)
indices, coefficients = seg.segment()

cloud_plane =cloud.extract(indices,True)
print(cloud_plane)


#   if (inliers->indices.size () == 0)
#   {
#     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
#     return (-1);
#   }
#   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
#                                       << coefficients->values[1] << " "
#                                       << coefficients->values[2] << " "
#                                       << coefficients->values[3] << std::endl;
###
if len(indices) == 0:
    print('Could not estimate a planar model for the given dataset.')
    exit(0)

#print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))

#   std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
#   for (size_t i = 0; i < inliers->indices.size (); ++i)
#     std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
#                                                << cloud->points[inliers->indices[i]].y << " "
#                                                << cloud->points[inliers->indices[i]].z << std::endl;
###
#print('Model inliers: ' + str(len(indices)))
#for i in range(0, len(indices)):
 #   print (str(indices[i]) + ', x: '  + str(cloud[indices[i]][0]) + ', y : ' + str(cloud[indices[i]][1])  + ', z : ' + str(cloud[indices[i]][2]))
visual = pcl_visualization.CloudViewing()
#visual.ShowColorCloud(cloud)

cloud1 = pcl.load_XYZRGB('/home/ribin/Downloads/object_02/test58.pcd')

while 1:


    #visual = pcl.pcl_visualization.CloudViewing()
   # visual.ShowColorCloud(cloud, b'cloud')
 #   visual.ShowColorCloud(cloud1,b'cloud')
  #  visual.ShowMonochromeCloud(cloud_cylinder)
    visual.ShowMonochromeCloud(cloud_plane)
    #visual.ShowColorCloud(cloud1)
    #visual.ShowColorCloud(cloud_cylinder)
    #visual.ShowColorCloud(filtered_cloud)
