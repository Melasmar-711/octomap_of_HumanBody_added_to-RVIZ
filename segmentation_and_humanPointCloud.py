import cv2
import mediapipe as mp
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import rospy
#import moveit_commander
#from geometry_msgs.msg import Pose, Vector3
#from geometry_msgs.msg import Pose, Vector3,PoseStamped 
#from geometry_msgs.msg import Point, Quaternion
#import sys
import time
import open3d as o3d
import subprocess
from sensor_msgs.msg import PointCloud2, PointField,Image,CameraInfo

import std_msgs.msg as std_msgs

import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header

from cv_bridge import CvBridge, CvBridgeError





class PoseDetector:

    def __init__(self, mode=True, upBody=False, smooth=True, detectionCon=0.8, trackCon=0.8,model=1):

        self.mode = mode
        self.upBody = upBody
        self.smooth = smooth
        self.enableSegmentation = True
        self.smoothSegmentation = True
        self.detectionCon= detectionCon
        self.trackCon = trackCon
        self.model=model
        self.BG_COLOR = (255, 255, 255) 

        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.drawing_soln= mp.solutions.drawing_styles
        self.pose = self.mpPose.Pose(static_image_mode=self.mode, model_complexity=self.model,min_tracking_confidence=self.trackCon,min_detection_confidence=self.detectionCon,smooth_segmentation=self.smoothSegmentation,enable_segmentation=self.enableSegmentation,smooth_landmarks=self.smooth)

    def findPose(self, img, draw=True):
    
        
        #print(1)
        self.img_copy = img
        self.imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(self.imgRGB)
        #print(self.results.pose_landmarks)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS,landmark_drawing_spec=self.drawing_soln.get_default_pose_landmarks_style())
                if self.enableSegmentation==True:
                 self.draw_segmented_image()
                 try:
                
                  #print(self.annotated_image)
                  return img,self.annotated_image
                 except:
                 
                  return img 
            

    def getPosition(self, img, draw=False):
    
        self.body_parts = []
        lmList=[]
        #print(2)
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                #print(id,"aaa", lm)
                cx, cy = int(lm.x * w), int(lm.y * h)
                #print(cx,cy)
                lmList.append([id, cx, cy])
                #print(lmList)
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        self.body_parts=lmList
        return lmList
        
        
    def draw_segmented_image(self):
        try:
        
            condition = np.stack((self.results.segmentation_mask,) * 3, axis=-1) > 0.1
            bg_image = np.zeros(self.img_copy.shape, dtype=np.uint8)
            bg_image[:] = self.BG_COLOR
            self.annotated_image = np.where(condition, self.img_copy, bg_image)
            
            
    #print(type(results.pose_landmarks))
        except:
            pass
        return self.annotated_image
    
class rs_camera:
	def __init__(self):
                
		self.pipeline = rs.pipeline()
		self._cv_bridge = CvBridge()

		# Create a config and configure the pipeline to stream
		#  different resolutions of color and depth streams
		self.config = rs.config()

		# Get device product line for setting a supporting resolution
		self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
		self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
		self.device = self.pipeline_profile.get_device()
		self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

		found_rgb = False
		for s in self.device.sensors:
			if s.get_info(rs.camera_info.name) == 'RGB Camera':
				found_rgb = True
				break
		if not found_rgb:
			print("The demo requires  camera_info_msg.header = Header()Depth camera with Color sensor")
			exit(0)

		self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

		if self.device_product_line == 'L500':
			self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
		else:
			self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

		# Start streaming
		self.profile = self.pipeline.start(self.config)

		# Getting the depth sensor's depth scale (see rs-align example for explanation)
		self.depth_sensor = self.profile.get_device().first_depth_sensor()
		self.depth_scale = self.depth_sensor.get_depth_scale()
		#print("Depth Scale is: " , self.depth_scale)

		# We will be removing the background of objects more than
		#  clipping_distance_in_meters meters away
		clipping_distance_in_meters = 1 #1 meter
		clipping_distance = clipping_distance_in_meters / self.depth_scale
		self.intr = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
		self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(self.intr.width, self.intr.height, self.intr.fx, self.intr.fy, self.intr.ppx, self.intr.ppy)

                
	def align(self,dst="color"):
		if dst =="color":

			self.align_to = rs.stream.color
		else:
			self.align_to = rs.stream.depth	
		self.align = rs.align(self.align_to)
		return self.align
	def get_frames(self):
		try:

        # Get frameset of color and depth
			frames = self.pipeline.wait_for_frames()				
			# frames.get_depth_frame() is a 640x360 depth image				
			# Align the depth frame to color frame
			self.aligned_frames = self.align.process(frames)
			# Get aligned frames


			self.aligned_depth_frame = self.aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image


			#self.aligned_depth_frame = rs.decimation_filter(1).process(self.aligned_depth_frame)######
			#self.aligned_depth_frame = rs.disparity_transform(True).process(self.aligned_depth_frame)#########3
			#self.aligned_depth_frame = rs.spatial_filter().process(self.aligned_depth_frame)#######3
			#self.aligned_depth_frame = rs.temporal_filter().process(self.aligned_depth_frame)#########33
			#depth_frame = rs.disparity_transform(False).process(depth_frame)
			self.aligned_depth_frame = rs.hole_filling_filter().process(self.aligned_depth_frame)######3
			#print(aligned_depth_frame[1])


			self.color_frame = self.aligned_frames.get_color_frame()
			self.depth_img=np.asanyarray(self.aligned_depth_frame.get_data())
			self.color_img=color_image = np.asanyarray(self.color_frame.get_data())
			return self.depth_img,self.color_img
		except:
				pass

	
	def pc(self,img,depth):

		
		#self.depth_segmented=np.where(img==255,0,depth)
		self.mask=img
		#print(depth.shape)
		depth_segmented=depth
		depth_segmented=np.where(img==255,0,depth_segmented)
		depth_segmented=np.where(img==0,0,depth_segmented)
		depth_segmented=np.asanyarray(depth_segmented) #*self.depth_scale
		
		
		depth_segmented=(depth_segmented.astype(np.uint16))
		
		#print(depth_segmented[200][200])

		#subprocess.run("rosservice call /oct/reset ",shell=True)
		depth_segmented=self.convert_cv2_to_ros_msg(depth_segmented)

		depth_publisher.publish(depth_segmented)
		camera_info_publisher.publish(camera_info_msg)
		
		




	def convert_cv2_to_ros_msg(self, cv2_data, image_encoding='16UC1'):
		"""
		Convert from a cv2 image to a ROS Image message.
		"""
		return self._cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)   
	def point_cloud_callback(self,msg):
    # Read the point cloud data

    # Modify the header frame
		msg.header.frame_id = "camera_depth_optical_frame"

    # Publish the modified point cloud
		cloud_pub.publish(msg) 
		









camera_1=rs_camera()
camera_1.align("depth")

detector=PoseDetector()
rospy.init_node('pose_estimator')
depth_publisher = rospy.Publisher('depth_pub', Image, queue_size=10)

camera_info_publisher=rospy.Publisher('rs_depth_info', CameraInfo, queue_size=10)

cloud_sub=rospy.Subscriber('/camera/depth/points', PointCloud2, camera_1.point_cloud_callback)


cloud_pub=rospy.Publisher('/camera/depth/points/with_frame', PointCloud2, queue_size=10)


camera_info_msg = CameraInfo()
camera_info_msg.width = camera_1.intr.width
camera_info_msg.height = camera_1.intr.height
camera_info_msg.distortion_model = 'plumb_bob'
camera_info_msg.K = [camera_1.intr.fx, 0.0, camera_1.intr.ppx,
                         0.0, camera_1.intr.fy, camera_1.intr.ppy,
                         0.0, 0.0, 1.0]
camera_info_msg.R = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
camera_info_msg.P = [camera_1.intr.fx, 0.0, camera_1.intr.ppx, 0.0,
                         0.0, camera_1.intr.fy, camera_1.intr.ppy, 0.0,
                         0.0, 0.0, 1.0, 0.0]
camera_info_msg.D=[0.0,0.0,0.0,0.0,0.0]
camera_info_msg.header = Header()
#camera_info_msg.header.stamp = rospy.Time.now()
camera_info_msg.header.frame_id = "camera_depth_optical_frame"


#camera_1=kinect_c()
pTime=0
while True:
    
    

	
	depth_img,img=camera_1.get_frames()
	img.flags.writeable = False
	image = img.copy()
	try:
                
	    
		
		
		
		img_tracked,img_segmented = detector.findPose(image)
		cTime = time.time()
		fps = 1 / (cTime - pTime)
		#print(fps)
		cv2.putText(img_tracked, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
		pTime = cTime

		lm = detector.getPosition(img_tracked) # retuns a list of landmark points

		
	

		cv2.imshow("tracked",img_tracked)

		cv2.imshow("segmented",img_segmented)
		img_segmented=cv2.cvtColor(img_segmented,cv2.COLOR_RGB2GRAY)
		camera_1.pc(img_segmented,depth_img)


		#camera_1.visualize_point_cloud()


	

	except:
		pass
	#img_segmented=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
	#camera_1.pc(img_segmented,depth_img)

	#pcl=[]
	#depth_image_3d = np.dstack((depth_img,depth_img,depth_img)) #depth image is 1 channel, color is 3 channels

		
	#np.where(image[:,:]==(255,255,255),[10,10,10,101,20,10],pcl)
	#try:           
		#camera_1.create_point_cloud(img_segmented)
	#xcept:
	#	pass


	if cv2.waitKey(5) & 0xFF == 27:
		break
	#rospy.spin()                                
	#rospy.spin()
	     
#"""	    
