import numpy as np
import pcl
from pcl import pcl_visualization
cloud = pcl.load('/home/ribin/Downloads/object_02/test58.pcd')
vg = cloud.make_voxel_grid_filter()
vg.set_leaf_size(0.001, 0.001, 0.001)
cloud_filtered = vg.filter()

#print(cloud_filtered)

seg = cloud.make_segmenter()
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_MaxIterations(100)
seg.set_distance_threshold(0.03)

[inliers_plane, coefficients_plane] = seg.segment ()
cloud_plane = cloud.extract(inliers_plane, True)


i = 0
nr_points = cloud_filtered.size
print(nr_points)



tree = cloud_filtered.make_kdtree()
tree1 = cloud_filtered.make_kdtree_flann()


ec = cloud_filtered.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance (0.02)
ec.set_MinClusterSize (3000)
ec.set_MaxClusterSize (80000)
ec.set_SearchMethod (tree)
cluster_indices = ec.Extract()

cloud_cluster = pcl.PointCloud()

for j, indices in enumerate(cluster_indices):
    # cloudsize = indices
    print('indices = ' + str(len(indices)))
    # cloudsize = len(indices)
    points = np.zeros((len(indices), 3), dtype=np.float32)
    # points = np.zeros((cloudsize, 3), dtype=np.float32)

    # for indice in range(len(indices)):
    for i, indice in enumerate(indices):
        # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
        # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
        points[i][0] = cloud_filtered[indice][0]
        points[i][1] = cloud_filtered[indice][1]
        points[i][2] = cloud_filtered[indice][2]

    cloud_cluster.from_array(points)
    #ss = "cloud_cluster_" + str(i) + ".pcd";
print(points)


cloud_cluster = pcl.PointCloud(points)



visual = pcl.pcl_visualization.CloudViewing()
#visual.ShowColorCloud(cloud1)
while 1:


    #visual = pcl.pcl_visualization.CloudViewing()
   # visual.ShowColorCloud(cloud, b'cloud')
 #   visual.ShowColorCloud(cloud1,b'cloud')
    visual.ShowMonochromeCloud(cloud_filtered)
    visual.ShowMonochromeCloud(cloud_cluster)
   # visual.ShowColorCloud(cloud1)
    #visual.ShowColorCloud(cloud_cylinder)
    #visual.ShowColorCloud(filtered_cloud)
