#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
import math
import numpy as np
import sensor_msgs.point_cloud2 as pc2

# globals
robot_pose = None
scan = None

def segmented_cte(segment, robot_xy):
    delta = segment[1] - segment[0]
    robot_delta = robot_xy - segment[0]

    # project robot_delta onto delta to get distance along path
    u = (robot_delta.dot(delta) / delta.dot(delta)) 

    # calculate CTE, vector rejection
    print("num:",robot_delta[1] * delta[0] - robot_delta[0] * delta[1])
    print("div:",np.sqrt(delta.dot(delta)))
    cte = (robot_delta[1] * delta[0] - robot_delta[0] * delta[1]) / np.sqrt(delta.dot(delta))
    return u, cte

def pose_callback(msg):
    global robot_pose
    robot_pose = msg.pose

def scan_callback(msg):
    global scan
    scan = msg

def obstacle_check(scan_msg):
    distances = []
    lp = lg.LaserProjection()
    pc2_msg = lp.projectLaser(scan_msg)
    point_generator = pc2.read_points(pc2_msg)

    for point in point_generator:
        if not math.isnan(point[2]):
            distances.append(np.sqrt(point[0] ** 2 + point[1] ** 2))

    length = len(distances)
    mid = length/2

    # only consider points in front of robot
    if(length >= 65):
        distances = distances[mid - 65/2 : mid + 65/2]

    if min(distances) < 0.6:
        return True

    return False

def get_lemnisicate(offset = None):
    # https://stackoverflow.com/questions/27795765/pyplot-lemniscate
    alpha = 0.9
    t = np.linspace(0, 2*np.pi, num=200)
    x = alpha * np.sqrt(2) * np.cos(t) / (np.sin(t)**2 + 1)
    y = alpha * np.sqrt(2) * np.cos(t) * np.sin(t) / (np.sin(t)**2 + 1)

    
    min_val = np.min(x[x>0])

    x = list(x)
    y = list(y)

    min_ind = x.index(min_val)

    x = [0] + x[min_ind:] + x[:min_ind]
    y = [0] + y[min_ind:] + y[:min_ind]

    ret = np.array(zip(y,x))
    if offset is not None:
        ret = ret + offset

    return ret 

def main():
    global scan, robot_pose
    gt_topic = rospy.get_param("~gt_topic")
    scan_topic = rospy.get_param("~scan_topic")
    cmd_topic = rospy.get_param("~cmd_topic")
    debug_plot = rospy.get_param("debug_plot",default=False)

    rospy.Subscriber(gt_topic, PoseStamped, callback=pose_callback)
    rospy.Subscriber(scan_topic, LaserScan, callback=scan_callback)
    cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)

    rate = rospy.Rate(40)
    twist_msg = Twist()

    print("waiting for robot pose...")
    while robot_pose is None:
        rate.sleep()
    print("got robot pose")

    robot_xy = np.array([robot_pose.position.x,
                                 robot_pose.position.y])

    # segmented path to follow
    shape = get_lemnisicate(offset=robot_xy)
    cur_ind = 0
    next_ind = 1
    max_ind = shape.shape[0] - 1
    
    # pd control
    p = 3.0
    d = 100
    last_cte = 0
    lin = 0.25

    if debug_plot:
        import pyqtgraph as pg
        win = pg.GraphicsWindow()
        win.resize(800, 800)
        plot = win.addPlot()
        # full path
        plot.plot(shape[:,0], shape[:,1], pen = "w")
        # current segment
        curve2 = plot.plot(shape[cur_ind:next_ind+1,0],shape[cur_ind:next_ind+1,1],pen="r")
        # robots position relative to current segment
        curve3 = plot.plot(shape[cur_ind:next_ind+1,0],shape[cur_ind:next_ind+1,1],pen="g")

    while not rospy.is_shutdown():
        pg.QtGui.QApplication.processEvents()
        # first check that there are no obstacles nearby
        if scan is not None:
            if obstacle_check(scan):
                print("Obstacle!!!")
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                cmd_pub.publish(twist_msg)
                continue

        if robot_pose is not None:
            robot_xy = np.array([robot_pose.position.x,
                                 robot_pose.position.y])
            start_xy = shape[cur_ind,:]
            next_xy = shape[next_ind,:]

            u, cte = segmented_cte([start_xy,next_xy],robot_xy)
            control = -p*cte - d * (cte-last_cte)
            last_cte = cte
            print("CTE: ", cte)

            if u >= 1:
                cur_ind = next_ind
                if cur_ind == max_ind:
                    next_ind = 0
                else:
                    next_ind = cur_ind + 1
                    continue
            
            twist_msg.linear.x = lin
            twist_msg.angular.z = control
            cmd_pub.publish(twist_msg)

            if debug_plot:
                curve2.setData(shape[cur_ind:next_ind+1,0],shape[cur_ind:next_ind+1,1])
                curve3.setData([shape[cur_ind][0],robot_xy[0]] ,[shape[cur_ind][1],robot_xy[1]])

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("mocap_nav")
    main()    