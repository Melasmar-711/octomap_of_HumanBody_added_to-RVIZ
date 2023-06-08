# file setup
1) you need to add the sensors_3d.yaml to your moveit config pkg in the config files
2) add the sensor_manager.launch.xml to your moveit config pkg in the launch files
3) replace the pointcloud_octomap_updater.cpp in your moveit package with the file provided ,Otherwise there it's going to try to mask the robot points in the point cloud and remove it . "this is actually a good thing if we are using the entire point cloud but we are using a segmented pointcloud" . you can find the file under moveit/moveit_ros/perception/pointcloud_octomap_updater/src.


# workflow
1) launch the depth_to_pc.launch which creates a depth_image_proc node
2) run segmentation and final point cloud publisher `python3 segmentation_and_humanPointCloud.py`
3) run reset_octo.py if you want to have the octomap reset every couple of seconds to avoid unremoved traces of dynamic obstacles `python3 reset_octo.py`

# customizing with your own pkg
- I am using a realsense D435 camera and i am using the depht_image_proc pkg to create the point_cloud out of the of the modified depth image
- the depth image proc takes arguments (CameraInfo ,depth_image_rect) which are two topics that generated and published in the code.
- the CameraInfo msg 
- the depth image proc return a PointCloud2 msg with no header frame id . i subscribe again to this msg and add the frame id that i desire the point cloud to be relative to . make sure to adjust that in the segmentation_and_humanPointCloud.py if your desired frame has a different name you can find this at the beggining of the code.

# what you need to know 
- if you change the final published Pointcloud2 msg with the topic name "/camera/depth/points/with_frame" you need to adjust this in the sensors_3d.yaml file to make the PointCloudOctomapUpdater listen to the modified topic name.

- same for modifying the "rs_depth_info" ,"depth_pub" ,"/camera/depth/points" Topics names. you will need to modify them in the depth_to_pc.launch file too . 
<p align="center">GOOD LUCK</p>




