import pyrosbag
from pyrosbag import BagPlayer as bag
import logging
import subprocess as sp
import time



file = pyrosbag.Bag("1.bag")
print(file)

with bag(pyrosbag.Bag("1.bag")) as files:
    files.play()
  #  while files.is_running():
        # Run for INTERVAL seconds.
   #     time.sleep(1)
