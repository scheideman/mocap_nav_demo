# mocap_nav_demo

Navigation demo assuming perfect ground truth pose from motion capture system. Uses mocap localization feedback to drive Jackal robot along figure 8 path. Segmented cross-track error (CTE) error is used for PD control.

# Dependencies

*Required:*
* [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) (need full desktop installation to test in Gazebo)
* [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros)

*Optional:*

* [urg_node](http://wiki.ros.org/urg_node) (for stopping if an obstacle is in the way)
* [pyqtgraph](http://www.pyqtgraph.org/) (for debug plot)

# Robot

* [Clearpath Jackal](https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/)

# Execution

On real jackal (assuming you are using a Jackal with laserscanner and it is running default bringup scripts and you have a motion capture system running a VRPN)

1. Add a tracker named "robot" to your motion capture VRPN

2. Launch motion capture client
```bash
roslaunch mocap_nav_demo vrpn_client.launch server:=<ip address of machine running motion capture server>
```

3. Launch laserscanner node (I am using Hokuyo urg-04lx. NOTE: this step can be skipped and the demo will still run, but make sure the robot has room to navigate and won't collide with anything)
```bash
roslaunch mocap_nav_demo hokuyo.launch
```

4. Launch mocap navigation demo for Jackal robot
```bash
roslaunch mocap_nav_demo jackal.launch 
```

On simulated jackal

1. Launch Jackal gazebo simulation with front laser (move robot to area where it has space to navigate)
```bash
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```

2. Launch mocap navigation demo for simulated Jackal
```bash
roslaunch mocap_nav_demo jackal_gazebo.launch
```

# Control Pseudocode
  
    1. S = segmented figure 8 shape
    2. i=0 is current segment index
    3. While True:
      a. Get robot pose from motion capture
      a. CTE = distance to current segment S_i
      b. ang_vel = PD_control(CTE)
      c. Publish twist command 
      d. If current segment is done -> i+=1




