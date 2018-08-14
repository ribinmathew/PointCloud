import os
path = '/home/ribin/Desktop/pcdfiles'
files = os.listdir(path)
i = 1

for file in files:
    os.rename(os.path.join(path, file), os.path.join(path, str(i)+'.pcd'))
    i = i+1