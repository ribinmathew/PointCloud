import os
path = '/media/ribin/DATA/bagfiles/pcdfiles/Zed_new_HQ'
files = os.listdir(path)
i = 1

for file in files:
    os.rename(os.path.join(path, file), os.path.join(path, str(i)+'.pcd'))
    i = i+1