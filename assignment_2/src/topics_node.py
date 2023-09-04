#!/usr/bin/env python
# ROS node to convert visual odometry and GPS data for fusion through particle filter

# Import libs and msgs
import rospy
import math
import numpy as np
import geonav_conversions as gc
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class DataCollector:

    # Contructor
    def __init__(self):
        rospy.loginfo("Initialising topic conversion node")
        rospy.init_node("topicsNode")

        # Initialise all subscribers, publishers and services
        self.visOdomSub = rospy.Subscriber(
            '/visual_odometry', Odometry, self.get_odometry_exp)
        self.gpsSub = rospy.Subscriber('/gps', NavSatFix, self.get_gps_exp)

        self.odomPub = rospy.Publisher(
            '/odometry_exp', Odometry, queue_size=10)
        self.gpsPub = rospy.Publisher('/gps_exp', Odometry, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz

        # Data structure to hold dead reckoning data
        self.odom = Odometry()
        self.odom.child_frame_id = 'camera'
        self.odom.header.frame_id = 'odom'

        # Data structure to hold GPS data
        self.gps = Odometry()
        self.gps.child_frame_id = 'gps'
        self.gps.header.frame_id = 'odom'
        self.gps.pose.pose.orientation.w = 1
        # Give GPS Origin
        self.olat = 51.765504
        self.olon = -1.258642

        # Record timestamp so program knows when to terminate
        self.ts = ''
        self.finalTimeStamp = '1403773071971149000'

        # Shutdown function
        # Used to tell other nodes that this node is inactive
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("Initialised data collector node")

        # Wait for topics to start recieving data
        rospy.loginfo("Waiting for data collector topics to initialise")
        rospy.wait_for_message("/visual_odometry", Odometry)
        rospy.wait_for_message("/gps", NavSatFix)
        rospy.loginfo("Data collector topics initialised")

    # Shutdown function called on ctrl_c
    def shutdownhook(self):
        rospy.loginfo("Data collector shutting down...")
        self.ctrl_c = True
        exit()

    # Transform some quaternion coordinates into roll, pitch and yaw
    def getEulerAngles(self, quaternion):
        orientation_list = [quaternion.x,
                            quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    # Dead reckoning implementation
    def get_odometry_exp(self, odomMsg):
        # Update header meta data
        self.odom.header.seq = odomMsg.header.seq
        self.odom.header.stamp = odomMsg.header.stamp

        # Populate covariance x, y and yaw
        self.odom.pose.covariance[0] += 1e-3          # X += 1 mm
        self.odom.pose.covariance[7] += 1e-3          # Y += 1 mm
        self.odom.pose.covariance[35] += 0.00174533   # Yaw += 0.1 deg

        # Calculate change in displacement
        prevYaw = self.getEulerAngles(self.odom.pose.pose.orientation)[2]
        dk = math.hypot(odomMsg.twist.twist.linear.x,
                        odomMsg.twist.twist.linear.y)

        # Calculate new x, y position using motion model
        self.odom.pose.pose.position.x += dk*np.cos(prevYaw)
        self.odom.pose.pose.position.y += dk*np.sin(prevYaw)

        # Update yaw
        yaw = prevYaw - odomMsg.twist.twist.angular.z
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.odom.pose.pose.orientation.z = quaternion[2]
        self.odom.pose.pose.orientation.w = quaternion[3]

        self.odomPub.publish(self.odom)

        # Update timestamp
        self.ts = str(odomMsg.header.stamp.secs) + \
            str(odomMsg.header.stamp.nsecs)

    # Update GPS data from world (long, lat) to local (x, y) in odom frame
    def get_gps_exp(self, gpsMsg):
        # Update header meta data
        self.gps.header.seq = gpsMsg.header.seq
        self.gps.header.stamp = gpsMsg.header.stamp

        # Get local x, y odometry using langitude and logitude
        xg, yg = gc.ll2xy(gpsMsg.latitude, gpsMsg.longitude,
                          self.olat, self.olon)

        self.gps.pose.pose.position.x = xg
        self.gps.pose.pose.position.y = yg

        # Populate covariance x, y which is already given from sensor
        # X (m)
        self.gps.pose.covariance[0] = gpsMsg.position_covariance[0]
        # Y (m)
        self.gps.pose.covariance[7] = gpsMsg.position_covariance[4]

        self.gpsPub.publish(self.gps)

# Main


def main():
    dataCollector = DataCollector()
    try:
        while not rospy.is_shutdown():
            if (dataCollector.ts == dataCollector.finalTimeStamp):
                dataCollector.shutdownhook()
            dataCollector.rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
