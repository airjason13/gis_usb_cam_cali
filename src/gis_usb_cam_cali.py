#!/usr/bin/env python
import sys
import os
import yaml
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
#import gtk
import rospy
import rospkg
from sensor_msgs.msg import Image
from gi.repository.Gdk import Screen
screen = Screen.get_default()

width, height = screen.width(), screen.height()
screen_width=1920
screen_height=1080

image_start_x = 0
image_start_y = 0
image_cali_width_factor = 0
image_cali_height_factor = 0



color_red = (0, 0, 255)
#print("width :", width)
#print("height :", height)
#global g_screen_width #= gtk.gdk.screen_width()
#global g_screen_height #= gtk.gdk.screen_height()


class Img_Sub():
    def __init__(self,image_cam, image_tmp='none'):
        print("image_cam:", image_cam)
        self.bridge = CvBridge()
        self.image_cam_sub= rospy.Subscriber(image_cam, Image, self.callback_od)
        self.image_cam_ok = False
    def callback_od(self, msg):
        self.image_cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #print("get od image")
        #cv2.imshow("sub od", self.image_od)
        self.image_cam_ok = True



def window_size():
        global g_screen_width
        global g_screen_height
        screen_width = g_screen_width
        screen_height = g_screen_height
        
        if cv2.waitKey(1) == ord('z'):
            screen_width = g_screen_width - 100
        
        if cv2.waitKey(1) == ord('c'):
            screen_height = g_screen_height - 100
        g_screen_width = screen_width
        g_screen_height = screen_height
        return screen_width,screen_height

def get_screen_res():
    global g_screen_width
    global g_screen_height
    
    return g_screen_width,g_screen_height

######################################################
#   od_img : yolov4 and drive_area ublish image
#
#
######################################################
def GIS_usb_cam_cali(sub_img):

    screen_width=1920
    screen_height=1080
    image_start_x = 0
    image_start_y = 0
    image_cali_width_factor = 0
    image_cali_height_factor = 0
    pattern_width = 32
    pattern_height = 32
    
    bridge = CvBridge()
    while not rospy.is_shutdown():
        key = cv2.waitKey(10)

        blank_image = np.zeros((screen_height, screen_width, 3), np.uint8)
        frame_cam = sub_img.image_cam.copy()
        usb_cam_height, usb_cam_width, color_channels = frame_cam.shape
        print("usb_cam_height = ", usb_cam_height)
        print("usb_cam_width = ", usb_cam_width)

        #start to adjust 
        if key == ord("w"):  # height factor ++
            image_cali_height_factor += 1
        elif key == ord("x"):  # height factor --
            image_cali_height_factor -= 1
        elif key == ord("d"):  #width factor ++
            image_cali_width_factor += 1
        elif key == ord("a"):  #height factor --
            image_cali_width_factor -= 1
        elif key == ord("u"):  # start y ++
            image_start_y += 1
        elif key == ord("m"):  # start y --
            image_start_y -= 1
        elif key == ord("k"):  # start x ++
            image_start_x += 1
        elif key == ord("h"):  # start x --
            image_start_x -= 1

        frame_cam_resized = cv2.resize(frame_cam, (usb_cam_width + image_cali_width_factor, usb_cam_height + image_cali_height_factor))
        resized_frame_cam_width = usb_cam_width + image_cali_width_factor
        resized_frame_cam_height = usb_cam_height + image_cali_height_factor
        xp = [0, resized_frame_cam_width - pattern_width, 0, resized_frame_cam_width- pattern_width]
        yp = [0, 0, resized_frame_cam_height - pattern_height, resized_frame_cam_height - pattern_height]
        p01 = (xp[0], yp[0])
        p02 = (xp[0] + pattern_width, yp[0] + pattern_height)
        p11 = (xp[1], yp[1])
        p12 = (xp[1] + pattern_width, yp[1] + pattern_height)
        p21 = (xp[2], yp[2])
        p22 = (xp[2] + pattern_width, yp[2] + pattern_height)
        p31 = (xp[3], yp[3])
        p32 = (xp[3] + pattern_width, yp[3] + pattern_height)
        cv2.rectangle(frame_cam_resized, p01, p02, color_red, 3)
        cv2.rectangle(frame_cam_resized, p11, p12, color_red, 3)
        cv2.rectangle(frame_cam_resized, p21, p22, color_red, 3)
        cv2.rectangle(frame_cam_resized, p31, p32, color_red, 3)

        blank_image[image_start_y:image_start_y + usb_cam_height + image_cali_height_factor, 
                    image_start_x:image_start_x + usb_cam_width + image_cali_width_factor] = frame_cam_resized
        #fusion_frame_result = cv2.addWeighted(fusion_frame_cam, 1, fusion_frame_lane,0.3 , 0)
        #fusion_frame_result = cv2.flip(fusion_frame_result, -1)
        
        #cv2.namedWindow("gis_usb_cali", cv2.WND_PROP_FULLSCREEN)
        #cv2.setWindowProperty("gis_usb_cali", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        #bg_img = cv2.resize(bg_img, (1280, 720), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("GIS_usb_cali", blank_image)
        if cv2.waitKey(1) == ord('q'):
            exit(0)
    

if __name__ == "__main__":
    print("GIS USB Cali")
    rospy.init_node('gis_usb_cam_cali', anonymous=True)
    rospack = rospkg.RosPack()
    pkg_root = os.path.join(rospack.get_path('gis_usb_cam_cali'), 'src')
    sys.path.append(pkg_root)

    with open(os.path.join(pkg_root, 'configs', "demo.yaml")) as fp:
        cfg = yaml.load(fp)

    cfg['pkg_root'] = pkg_root
    cfg['data']['publish_rate'] = rospy.get_param("~det_rate")
    print(cfg)
    
    img_sub = Img_Sub(cfg['data']['image_src'] )

    while not (img_sub.image_cam_ok ):
        time.sleep(0.5)
        #print("waiting image from USB cam")
        if cv2.waitKey(2) == ord('q'):
            exit(0)
    
    GIS_usb_cam_cali(img_sub)
