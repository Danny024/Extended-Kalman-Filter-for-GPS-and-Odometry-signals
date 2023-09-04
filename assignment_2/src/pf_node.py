#!/usr/bin/env python

# ROS node to fuse gps and visual odometry data

# Import libs and msgs
import rospy
import math
import csv
import scipy.stats
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ParticleFilter:

    # Contructor
    def __init__(self):
        rospy.loginfo("Initialising particle filter node")
        rospy.init_node("particleFilterNode")

        # Initialise all subscribers, publishers and services
        self.OdomSub = rospy.Subscriber('/visual_odometry', Odometry, self.predict)
        self.gpsSub = rospy.Subscriber('/gps_exp', Odometry, self.update)

        self.fusedOdomPub = rospy.Publisher('/fused_pose_exp', Odometry, queue_size=10)

        self.rate = rospy.Rate(10) #10 Hz

        # Data structure to hold fused pose
        self.fusedPose = Odometry()
        self.fusedPose.header.frame_id = 'odom'

        # Initialse particles and weights
        self.particles, self.weights = self.makeParticles((-0.1, 0.1), (-0.1, 0.1), (0.4, 0.6), 3000)

        # Member variables to store last recorded state estimate
        self.mean = []
        self.covariance = []

        # Array to store all results for writing to csv later
        self.results = []

        # Record timestamp so program knows when to terminate
        self.ts = ''
        self.finalTimeStamp = '1403773071971149000'

        # Shutdown function
        # Used to tell other nodes that this node is inactive
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("Initialised particle filter node")

        # Wait for topics to start recieving data
        rospy.loginfo("Waiting for particle filter topics to initialise")
        rospy.wait_for_message("/visual_odometry", Odometry)
        rospy.wait_for_message("/gps_exp", Odometry)
        rospy.loginfo("Particle filter topics initialised")

    # Shutdown function called on ctrl_c
    def shutdownhook(self):
        rospy.loginfo("Particle filter shutting down...")
        self.ctrl_c = True
        exit()

    # Transform some quaternion coordinates into roll, pitch and yaw
    def getEulerAngles(self, quaternion):
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    # Initialize particles with random x, y, and yaw given some range
    def makeParticles(self, xRange, yRange, yawRange, numParts):
        particles = np.empty((numParts, 3))
        particles[:, 0] = np.random.uniform(xRange[0], xRange[1], size = numParts)      # Particle X
        particles[:, 1] = np.random.uniform(yRange[0], yRange[1], size = numParts)      # Particle Y
        particles[:, 2] = np.random.uniform(yawRange[0], yawRange[1], size = numParts)  # Particle Yaw
        particles[:, 2] %= 2 * np.pi            # Ensure particles yaw are between 0 and 2pi radians
        weights = np.ones(numParts) / numParts  # Initialize weights to each have equal probability
        return particles, weights

    # Predict particles state using motion model
    def predict(self, odomMsg):
        dk = math.hypot(odomMsg.twist.twist.linear.x, odomMsg.twist.twist.linear.y) # Calculate change in displacement
        self.particles[:, 0] += dk*np.cos(self.particles[:, 2])     # Predict particles X
        self.particles[:, 1] += dk*np.sin(self.particles[:, 2])     # Predict particles Y
        self.particles[:, 2] -= odomMsg.twist.twist.angular.z       # Predict particles Yaw
        self.ts = str(odomMsg.header.stamp.secs) + str(odomMsg.header.stamp.nsecs)  # Update timestamp

    # Update weights based on how far particles are from GPS measurement using multivariate gaussian ditrobution
    def update(self, gpsMsg):
        # Contruct distribution using GPS measurement and GPS covariance
        mean = np.array([gpsMsg.pose.pose.position.x, gpsMsg.pose.pose.position.y])
        covMatrix = np.matrix([[gpsMsg.pose.covariance[0], 0], [0, gpsMsg.pose.covariance[7]]])
        normal = scipy.stats.multivariate_normal(mean=mean, cov=covMatrix)
        # Update each weight using probability density function
        for i in range(len(self.weights)):
            particle = np.array([self.particles[i, 0], self.particles[i, 1]])
            self.weights[i] = normal.pdf(particle)
        self.weights += 1.0e-300            # Add a tiny number so there's no divide by zero when normalising
        self.weights /= sum(self.weights)   # Normalise weights so they add up to 1 (Total Probability Theorem)

        self.resample()
        self.estimate()

    # Resample particles to replace lower probability particles with clones of higher probability particles
    def resample(self):
        N = len(self.weights)
        reParticles = np.empty((N, 3))
        index = np.random.randint(N-1)
        beta = 0
        maxWeight = max(self.weights)
        for i in range(N):
            beta += np.random.uniform() * 2 * maxWeight
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % N
            reParticles[i] = self.particles[index]
        self.particles = reParticles

    # Calculate the weighted average of the particles to find the state estimate
    def estimate(self):
        weights = self.weights.flatten()    # Flatten so numpy array is correct shape
        self.mean = np.average(self.particles, weights = weights, axis = 0)
        self.covariance = np.average((self.particles - self.mean)**2, weights = weights, axis = 0)

        # Update results for csv
        data = self.ts + "," + str(self.mean[0]) + "," + str(self.mean[1]) + "," + str(self.mean[2]) + "\n"
        self.results.append(data)

        self.publishMsg()

    # Publish estimated state as odometry for visualisation in rviz
    def publishMsg(self):
        # Update estimated x, y
        self.fusedPose.pose.pose.position.x = self.mean[0]
        self.fusedPose.pose.pose.position.y = self.mean[1]

        # Update estimated yaw
        quaternion = quaternion_from_euler(0, 0, self.mean[2])
        self.fusedPose.pose.pose.orientation.z = quaternion[2]
        self.fusedPose.pose.pose.orientation.w = quaternion[3]

        # Update covariance
        self.fusedPose.pose.covariance[0] = self.covariance[0]    # X
        self.fusedPose.pose.covariance[7] = self.covariance[1]    # Y
        self.fusedPose.pose.covariance[35] = self.covariance[2]   # Yaw

        self.fusedOdomPub.publish(self.fusedPose)

    # Write results to csv
    def writeToFile(self):
        rospy.loginfo("Writing to csv...")
        # Open csv file
        with open("/home/user/catkin_ws/src/assignment2_exp/fused_estimates_exp.csv", "w") as f:
            f.write("timestamp,x,y,yaw\n")
            for entry in self.results:
                f.write(entry)
        rospy.loginfo("Finished! Now shutting down...")
        self.shutdownhook()

# Main
def main():
    particleFilter = ParticleFilter()
    try:
        while not rospy.is_shutdown():
            # Check if the final message of the rosbag has been recieved
            if (particleFilter.ts == particleFilter.finalTimeStamp):
                particleFilter.writeToFile()
            particleFilter.rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
