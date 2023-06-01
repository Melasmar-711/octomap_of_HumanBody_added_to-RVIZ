# file setup
1) you need to add the sensors_3d.yaml to your moveit config pkg in the config files
2) add the sensor_manager.launch.xml to your moveit config pkg in the launch files

# workflow
1) launch the depth_to_pc.launch which creates a depth_image_proc node
2) run segmentation and final point cloud publisher `python3 segmentation_and_humanPointCloud.py`
3) run reset_octo.py if you want to have the octomap reset every couple of seconds to avoid unremoved traces of dynamic obstacles `python3 reset_octo.py`

# customizing with your own pkg
- I am using a realsense D435 camera and i am using the depht_image_proc pkg to create the point_cloud out of the of the modified depth image
- the depth image proc takes arguments (CameraInfo ,depth_image_rect) which are two topics that generated and published in the code.
- the CameraInfo msg 
- the depth image proc return a PointCloud2 msg with no header frame id . i subscribe again to this msg and add the frame id that i desire the point cloud to be relative to . make sure to adjust that in the segmentation_and_humanPointCloud.py if your desired frame has a different name you can find this at the beggining of the code 


