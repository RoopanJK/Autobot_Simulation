#!/usr/bin/env python3

from math import sqrt, pow, pi, cos
from numpy.lib.function_base import angle
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

waypoint_x = [-0.022, 0.81, 0.82, 1.96, 1.96, 2.70, 2.70, 3.75, 3.75, 4.5, 4.5]
waypoint_y = [5, 5, -.2, -.2, 5, 5, -.2, -.2, 3, 3, -.2]

angle_z = [0, 4.70, 0, 1.57, 0, 4.70, 0, 1.57, 0, 4.70, 0, 1.57]


class controller:

    def __init__(self) -> None:
        self.waypoint_count = 1
        self.z_ang = 0
        self.regions = 0
        self.px = 0
        self.py = 0
        self.lin_tol = 0.02
        self.ang_tol = 0.02
        self.d = 0.65
        rospy.init_node("waypoints")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        self.px = data.pose.pose.position.x
        self.py = data.pose.pose.position.y
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.z_ang = euler_from_quaternion([x, y, z, w])[2]

    # Normalization for rotate
    def normailize(self, ang_z):
        if(ang_z < 0):
            ang = pi - ang_z
        else:
            ang = ang_z
        # print(ang)
        return ang

    # Rotate
    def rotate(self, i, direction):
        rotate_msg = Twist()
        while((abs(self.normailize(self.z_ang) - i)) > self.ang_tol):
            rotate_msg.angular.z = 0.5 * direction
            self.pub.publish(rotate_msg)
        rotate_msg.angular.z = 0
        self.pub.publish(rotate_msg)

    # Euc distance
    def euc(self, x, y):
        ed = sqrt(pow((x - self.px), 2) + pow((y - self.py), 2))
        return ed

    # Move to goal algorithm
    def movetogoal(self, gx, gy, tolerance=0.5):
        movetogoal_msg = Twist()
        while(self.euc(gx, gy) > tolerance):
            movetogoal_msg.linear.x = 0.8
            self.pub.publish(movetogoal_msg)
        movetogoal_msg.linear.x = 0
        self.pub.publish(movetogoal_msg)
        print(self.waypoint_count)
        self.waypoint_count = self.waypoint_count + 1


if __name__ == '__main__':
    try:
        go = controller()
        print("Starting the Bot")
        rospy.sleep(3)
        go.movetogoal(waypoint_x[0], waypoint_y[0])
        rospy.sleep(0.5)
        go.rotate(angle_z[0], -1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[1], waypoint_y[1])
        rospy.sleep(0.5)
        go.rotate(angle_z[1], -1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[2], waypoint_y[2])
        rospy.sleep(0.5)
        go.rotate(angle_z[2], 1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[3], waypoint_y[3])
        rospy.sleep(0.5)
        go.rotate(angle_z[3], 1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[4], waypoint_y[4])
        rospy.sleep(0.5)
        go.rotate(angle_z[4], -1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[5], waypoint_y[5])
        rospy.sleep(0.5)
        go.rotate(angle_z[5], -1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[6], waypoint_y[6])
        rospy.sleep(0.5)
        go.rotate(angle_z[6], 1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[7], waypoint_y[7])
        rospy.sleep(0.5)
        go.rotate(angle_z[7], 1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[8], waypoint_y[8])
        rospy.sleep(0.5)
        go.rotate(angle_z[8], -1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[9], waypoint_y[9])
        rospy.sleep(0.5)
        go.rotate(angle_z[9], -1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[10], waypoint_y[10])
        rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
