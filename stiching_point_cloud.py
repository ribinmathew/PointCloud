#here we are gonna try to stich point cloud which going get from the lazer to generate full point cloud
# so we are gonna load 2 pcl and stich it together


import pcl
from pcl import pcl_visualization
import numpy as np
import matplotlib.pyplot as plt
cloud5 = pcl.PointCloud_PointXYZRGB()

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


visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(cloud5)


v = True
while v:
    v=not(visual.WasStopped())
