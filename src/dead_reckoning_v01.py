#!/usr/bin/env python

# =============================================================================
# Peter G. Adamczyk
# Updated 2019-10-12
# =============================================================================

import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from mobrob_util.msg import ME439WheelDisplacements

# ==============================================================================
# # Get parameters from rosparam
# # NOTE this is the Estimator, so we should use the "model" parameters.
# ==============================================================================
wheel_width = rospy.get_param('/wheel_width_model')
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter / 2.0

# Global variables for the robot's position
r_center_world_estimated = np.array(
    [0., 0.])  # Position (r) of the robot in the World frame.
# heading angle (theta) of the robot relative to the World frame.
theta_estimated = 0.

# Global variables for the robot's wheel displacements (to keep knowledge
# of it from one step to the next)
d_left_previous = 0.
d_right_previous = 0.

# Rate to set how often the estimated "pose" is published
f = 10.  # Hz


def listener():
    global r_center_world_estimated, theta_estimated

    rospy.init_node('dead_reckoning', anonymous=False)
    rospy.Subscriber('/robot_wheel_displacements', ME439WheelDisplacements,
                     dead_reckoning)

    rospy.Subscriber('/robot_position_override', Pose2D, set_pose)

    pub_robot_pose_estimated = rospy.Publisher('/robot_pose_estimated',
                                               Pose2D,
                                               queue_size=1)
    robot_pose_estimated_message = Pose2D()
    r = rospy.Rate(f)

    while not rospy.is_shutdown():
        # Publish the pose
        robot_pose_estimated_message.x = r_center_world_estimated[0]
        robot_pose_estimated_message.y = r_center_world_estimated[1]
        robot_pose_estimated_message.theta = theta_estimated
        pub_robot_pose_estimated.publish(robot_pose_estimated_message)

        # Log the info to the ROS log.
        rospy.loginfo(pub_robot_pose_estimated)

        r.sleep()


# =============================================================================
# # Callback function for "dead-reckoning" (alternatively called "odometry")
# =============================================================================
def dead_reckoning(msg_in):
    # These global variables hold this node's estimate of robot pose.
    global r_center_world_estimated, theta_estimated
    # This parameter is necessary.
    global wheel_width
    # More globals to store the previous values of the wheel displacements
    global d_left_previous, d_right_previous

    d_left = msg_in.d_left
    d_right = msg_in.d_right

    diff_left = d_left - d_left_previous
    diff_right = d_right - d_right_previous

    d_left_previous = d_left
    d_right_previous = d_right

    diff_pathlength = (diff_left + diff_right) / 2
    diff_theta = (diff_right - diff_left) / wheel_width

    theta_avg = theta_estimated + diff_theta / 2.

    r_center_world_estimated[0] += diff_pathlength * -np.sin(
        theta_avg)  # x-direction position
    r_center_world_estimated[1] += diff_pathlength * np.cos(
        theta_avg)  # y-direction position
    theta_estimated += diff_theta  # angle


# ==============================================================================
#     End of function "dead_reckoning"
# ==============================================================================


# =============================================================================
# # Callback function for "set_pose" (to override the estimated position)
# # This node receives a "Pose2D" message type, which should be used to replace
# # the current estimated pose.
# =============================================================================
def set_pose(msg_in):
    # These global variables hold this node's estimate of robot pose.
    global r_center_world_estimated, theta_estimated

    r_center_world_estimated[0] = msg_in.x
    r_center_world_estimated[1] = msg_in.y
    theta_estimated = msg_in.theta


# ==============================================================================
#     # End of function "set_pose"
# ==============================================================================

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
