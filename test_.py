import pcl
from pcl import pcl_visualization
import numpy as np
point_cloud = pcl.PointCloud()

p1 = pcl.load('/home/ribin/Downloads/mug.pcd')
points = np.zeros((150, 4), dtype=np.float32)


print(points)