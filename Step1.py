"""In step one we have to load the cloud data from the zed cam directly
    So we have to create a code which load pointcloud from the zed cam directly
    Its geting published every second!  so our program first convert the point cloud file to pcd
    then the pcd will be processed! so we have to
    1 - subscribe to the pointcloud topic
    2 - Convert the point cloud to pcd file format
    3 - we will save the pcd file to a temp folder
    4 - This temp folder will get over write after every sucessful pick
    5 - From this pcd file we will generate co-ordinates we require and we will send it to the robot via ros
    6 - we will wait for the robot to confirm a pick success msg and then repeat the process
    7 - We are getting a failed replay from the robot then we will send the 2 nd co-ordinate to the robot for the pick
    Line_number 16- the ros_zed_cam.sh script contain a bash script which subscribe to the point cloud and convert the point cloud to the pcd file format"""


import subprocess
import pcl
import os
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import requests
import re
import time
from bs4 import BeautifulSoup
session = requests.Session()


from pcl import pcl_visualization

point_cloud  = pcl.PointCloud()

path = "/home/ribin/Desktop/pcd"

files = os.listdir(path)
file = files[0]


pcd_file_path = os.path.join(path,file)



def do_passthrough_filter(point_cloud, name_axis = 'y', min_axis = 0.01, max_axis = 1):
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
  inlier_indices, coefficients = segmenter.segment()
  inliers = point_cloud.extract(inlier_indices, negative = False)
  outliers = point_cloud.extract(inlier_indices, negative = True)
  return inliers, outliers



" now we have to access the pcd file for further processing"
def pcdfile_creator():

    subprocess.call(['/home/ribin/Desktop//ros_zed_cam.sh'])



def pcd_processing():

    cloud = pcl.load_XYZRGB(pcd_file_path)

    downsampled_cloud = do_voxel_grid_filter(point_cloud=cloud, LEAF_SIZE=0.008)
    filtered_cloud = do_passthrough_filter(point_cloud=cloud,name_axis='z', min_axis=0.6, max_axis=1.1)
    table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance=0.01)

    objects_cloud_mono = []
    for i in range(0, objects_cloud.size):
        points = (objects_cloud[i][0], objects_cloud[i][1], objects_cloud[i][2])
        objects_cloud_mono.append(points)


    objects_cloud_mono = np.array(objects_cloud_mono, dtype=np.float32)

    x = []
    y = []
    z = []

    for i in range(0, len(objects_cloud_mono)):
        x.append(objects_cloud_mono[i][0])
        y.append(objects_cloud_mono[i][1])
        z.append(objects_cloud_mono[i][2])

    kmeans = KMeans(n_clusters=3)
    kmeans = kmeans.fit(objects_cloud_mono)
    C = kmeans.cluster_centers_

    return C


def io_status():
    
    #here we have to call a script to find out whether the io pins are high or not
    # here we are gonna create a program to find out the io_status of the robot
    session = requests.Session()

    try:
        a = session.get("http://192.168.125.1/rw/iosystem/signals/Input1",
                        auth=requests.auth.HTTPDigestAuth('Default User', 'robotics'))


    except:
        session = requests.Session()
        a = session.get("http://192.168.125.1/rw/iosystem/signals/Input1",
                        auth=requests.auth.HTTPDigestAuth('Default User', 'robotics'))

    ac = a.content
    x = re.compile("<.+?>").findall(ac)
    ac.replace(x[0], "")

    dictval = {}
    soup = BeautifulSoup(ac)

    for link in soup.find_all("li"):
        if not link: break
        title = link.get('title')
        dictval[title] = {}
        soup2 = BeautifulSoup(str(link))
        for llink in soup2.find_all('span'):
            dictval[title][llink.get('class')[0]] = llink.encode_contents()
        # print dictval[title]
    return dictval[title]['lvalue']



def ThreeD_plotting(X,Y,Z,C):
    x = X
    y =Y
    z = Z
    C = C

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.plot3D(x, y,z, zdir='z', c= 'red')
    ax.scatter3D(x, y, z, c=z, cmap='Greens')

    ax.scatter(C[:, 0], C[:, 1], C[:, 2], marker='.', c='#050505', s=1000)
    ax.set_title('surface')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()

def Visualizing(cloud):

    # point_cloud.from_array(objects_cloud_mono)
    # print(objects_cloud_mono)


    visual = pcl.pcl_visualization.CloudViewing()
    while 1:


        # visual = pcl.pcl_visualization.CloudViewing()
        # visual.ShowColorCloud(cloud, b'cloud')
        # visual.ShowColorCloud(cloud,b'cloud')
        # visual.ShowMonochromeCloud(objects_cloud_mono)
        # visual.ShowColorCloud(downsampled_cloud)
        visual.ShowColorCloud(cloud)
        # visual.ShowColorCloud(filtered_cloud)

""" now we have to check the status of the io pins  so that we will be able to know whether the pick was successful or not"""



