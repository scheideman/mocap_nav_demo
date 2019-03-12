#!/usr/bin/python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

robot_pose = None

def pose_callback(msg, args):
    global robot_pose
    if args == "jackal":
        model_name = "jackal"
        index = msg.name.index(model_name)
        robot_pose = msg.pose[index]
    elif args == "turtlebot":
        model_name = "mobile_base"
        index = msg.name.index(model_name)
        robot_pose = msg.pose[index]

def main():
    global robot_pose
    robot_name = rospy.get_param("~robot_name")
    rospy.Subscriber("/gazebo/model_states", ModelStates,callback=pose_callback, callback_args=(robot_name))
    mocap_sim_pub = rospy.Publisher("/vrpn_client_node/robot/pose", PoseStamped, queue_size=1)

    rate = rospy.Rate(120)
    out_msg = PoseStamped()
    while not rospy.is_shutdown():
        if robot_pose is not None:
            out_msg.header.stamp = rospy.Time.now()
            out_msg.pose = robot_pose
            mocap_sim_pub.publish(out_msg)

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("mocap_nav")
    main()    