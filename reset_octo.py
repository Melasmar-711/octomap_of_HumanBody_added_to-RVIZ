import subprocess
import rospy

rospy.init_node("reset_octo",anonymous=True)
#rat=rospy.Rate()
while not rospy.is_shutdown():

    subprocess.run("rosservice call /clear_octomap ",shell=True)
    #rospy.sleep(0.5)