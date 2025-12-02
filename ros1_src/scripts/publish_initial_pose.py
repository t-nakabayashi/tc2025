#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import math

def publish_initial_pose():
    rospy.init_node('initial_pose_publisher', anonymous=True)

    # Get parameters
    x = rospy.get_param('~x', 0.0)
    y = rospy.get_param('~y', 0.0)
    z = rospy.get_param('~z', 0.0)
    yaw = rospy.get_param('~yaw', 0.0)
    delay = rospy.get_param('~delay', 3.0)

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    # Wait for other nodes to start
    rospy.sleep(delay)

    # Create message
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'

    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z

    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]

    # Covariance
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068

    rospy.loginfo(f'Publishing initial pose: x={x}, y={y}, yaw={yaw}')
    pub.publish(msg)
    rospy.sleep(0.5)
    pub.publish(msg)  # Publish twice to ensure reception

    rospy.loginfo('Initial pose published')

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
