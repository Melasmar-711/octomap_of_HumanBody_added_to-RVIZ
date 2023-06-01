# file setup
1) you need to add the sensors_3d.yaml to your moveit config pkg in the config files
2) add the sensor_manager.launch.xml to your moveit config pkg in the launch files

# workflow
1) launch the depth_to_pc.launch 
2) run segmentation and final point cloud publisher `python3 segmentation_and_humanPointCloud.py`
3) run reset_octo.py if you want to have the octomap reset every couple of seconds to avoid unremoved traces of dynamic obstacles
