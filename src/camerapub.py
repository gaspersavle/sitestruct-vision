#1/usr/bin/env python
from colorama import Fore
import numpy as np
import ipdb
import pyrealsense2 as rs
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import glob

class example():
    def __init__(self):
        self.NODE_NAME = "chinesium_pub"
        self.CAMERA_PATH = "/dev/video"
        self.generic_topic = 'generic_cam/colour/image_raw/compressed'
        self.rs_depth_topic = 'rs_cam/depth/image_raw/compressed'
        self.rs_rgb_topic = 'rs_cam/colour/image_raw/compressed'
        self.cap = None
        #ipdb.set_trace(context = 10)
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            ctx = rs.context()
            devices = ctx.query_devices()
            device = devices[0]
            device_name = device.get_info(rs.camera_info.name)
            sensors = device.query_sensors()
            device_product_line = str(device.get_info(rs.camera_info.product_line))
            print(f"{Fore.GREEN}Camera: {device_name}\n")
            self.realsense_paths = []
            for sensor in sensors:
                sensor_type = sensor.get_info(rs.camera_info.name)
                sensor_adress = "/dev/"+sensor.get_info(rs.camera_info.physical_port).split("/")[-1]
                self.realsense_paths.append(sensor_adress)
                self.realsense_paths.append("/dev/video"+str(int(sensor_adress[-1])+1))
                print(f"    {Fore.CYAN}Sensor Type: {Fore.BLUE}{sensor_type} | {Fore.CYAN}Sensor Adress: {Fore.BLUE}{sensor_adress}")  
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            self.pipeline.start(self.config)

        except Exception as e:
            print(e)
        self.pub_rs_depth = rospy.Publisher(self.rs_depth_topic, CompressedImage, queue_size=10)
        self.pub_rs_rgb = rospy.Publisher(self.rs_rgb_topic, CompressedImage, queue_size=10)
        print(f"{Fore.LIGHTBLUE_EX}Realsense camera, mounted at:{self.realsense_paths}\n    Publishing to:\n    RGB: {self.rs_rgb_topic}\n    Depth: {self.rs_depth_topic}")
        self.pub_generic = rospy.Publisher(self.generic_topic, CompressedImage, queue_size=10)
        self.bridge = CvBridge()
        rospy.init_node(self.NODE_NAME)
        self.init_generic_usb()

    def init_generic_usb(self):
        video_devs = glob.glob(self.CAMERA_PATH+"*")
        sorted_video_devs = sorted(video_devs, key=lambda x: int(x.split('video')[1]))
        valid_entries = []
        for device in sorted_video_devs:
            if device not in self.realsense_paths:
                cap = cv2.VideoCapture(device)
                if cap.isOpened():
                    valid_entries.append(device)
                cap.release()
        cameras = valid_entries[:2]
        print(f"{Fore.MAGENTA} Cameras: {cameras}{Fore.RESET}")
        self.cap_generic = cv2.VideoCapture(cameras[0])
        print(f"{Fore.LIGHTBLUE_EX}Generic camera, mounted at:{cameras[0]}\n    Publishing to: {self.generic_topic}")
        self.cap_generic.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    def get_feed(self):    
        frames = self.pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        colour = frames.get_color_frame()

        depth_img = np.asanyarray(depth.get_data())
        depth_img = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
        rosdepth = self.bridge.cv2_to_compressed_imgmsg(depth_img)

        colour_img = np.asanyarray(colour.get_data())
        roscolour = self.bridge.cv2_to_compressed_imgmsg(colour_img)

        self.pub_rs_depth.publish(rosdepth)
        self.pub_rs_rgb.publish(roscolour)
        
        ret, frame = self.cap_generic.read()
        if ret:
            ros_image = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.pub_generic.publish(ros_image)

if __name__ == "__main__":
    ex = example()
    while not rospy.is_shutdown(): 
        ex.get_feed()
