import freenect
import cv2
import numpy as np

def get_video():
    array,_ = freenect.sync_get_video()
    array= cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array

def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

if __name__ =="__main__":
    while 1:
        frame = get_video()
        depth = get_depth()
        cv2.imshow("RGB image",frame)
        cv2.imshow("Depth image", depth)
        k  = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()