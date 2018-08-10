import pcl
from pcl import pcl_visualization


cloud = pcl.load("/home/ribin/Downloads/mug.pcd")

print ('load table_scene_mug_stereo_textured.pcd')

# estimate normals
# pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
# pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
print ('make_IntegralImageNormalEstimation: ')
ne = cloud.make_IntegralImageNormalEstimation()


print ('set_NormalEstimation_Method_AVERAGE_3D_GRADIENT: ')
ne.set_NormalEstimation_Method_AVERAGE_3D_GRADIENT ()

print ('set_MaxDepthChange_Factor: ')
ne.set_MaxDepthChange_Factor(0.02)
print ('set_NormalSmoothingSize: ')
ne.set_NormalSmoothingSize(10.0)
print ('set OK')
print ('compute2 - start')
print ne
"""

normals = ne.compute2(cloud)
print ('compute2 - end')
print (str(normals.size))

while 1:

    # visualize normals
    viewer = pcl.pcl_visualization.PCLVisualizering()
    viewer.SetBackgroundColor (0.0, 0.0, 0.5)
    # viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    viewer.AddPointCloudNormals(cloud,normals)
    # viewer.AddPointCloud(cloud) """