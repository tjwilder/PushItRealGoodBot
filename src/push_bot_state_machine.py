#!/usr/bin/env python

import numpy as np
import rospy
import traceback
import time
from mobrob_util.msg import ME439WaypointXY, ME439SensorsProcessed
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

# Waypoints to hit: a "numpy.array" of [x,y] coordinates.
# Example: Square
# waypoints = np.array([[0.5, 0.], [0.5, 0.5], [0., 0.5], [0., 0.]])
goal = np.array([0, 2])

##################################################################
# Run the Publisher
##################################################################
# initialize the current "segment" to be the first one (index 0) # (you
# could skip segments if you wanted to)
waypoint_number = 0  # for waypoint seeking.
path_complete = Bool()
path_complete.data = False

states = {'Standby': 0, 'Push': 1, 'Scan': 2, 'Reposition': 3}
x = 0
y = 0
theta = 0

# TODO: Transition from Standby to a new goal state via subscribing


# Publish desired waypoints at the appropriate time.
def talker():
    global waypoints, waypoint_number, path_complete, pub_waypoint
    global pub_path_complete, pub_find_object, states, state
    # Launch this node with the name "set_waypoints"
    rospy.init_node('state_machine', anonymous=False)

    pub_waypoint = rospy.Publisher('/waypoint_xy',
                                   ME439WaypointXY,
                                   queue_size=1)

    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)
    pub_find_object = rospy.Publisher('/find_object', Bool, queue_size=1)

    # Wait for other nodes to start...
    time.sleep(10)
    rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)
    rospy.Subscriber('/robot_pose_estimated', Pose2D, update_location)
    rospy.Subscriber('/sensors_data_processed', ME439SensorsProcessed,
                     process_sensor_data)
    # TODO: Fix data type to be location
    rospy.Subscriber('/found_object', Pose2D, found_object)

    # Start by finding the object
    state = states['Scan']
    find_object()

    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10)  # in Hz
    try:
        # start a loop
        while not rospy.is_shutdown():
            if state == states['Standby']:
                print('Standing by...')
                time.sleep(5)
            elif state == states['Push']:
                if path_complete.data or waypoint_number >= waypoints.shape[0]:
                    rospy.loginfo('Goal reached')
                    state = states['Standby']
                    path_complete.data = True

                    pub_path_complete.publish(path_complete)
                else:
                    publish_waypoint([waypoints[waypoint_number, 0],
                                     waypoints[waypoint_number, 1]], False)
            elif state == states['Scan']:
                # Wait for scan to complete
                time.sleep(.2)
            elif state == states['Reposition']:
                if path_complete.data:
                    rospy.loginfo('Repositioning complete, pushing')
                    state = states['Push']
                    waypoints = np.array([goal])
                    waypoint_number = 0
                    publish_waypoint([waypoints[waypoint_number, 0],
                                     waypoints[waypoint_number, 1]], True)
                else:
                    publish_waypoint([waypoints[waypoint_number, 0],
                                     waypoints[waypoint_number, 1]], False)

            r.sleep()

    except Exception:
        traceback.print_exc()
        pass


def publish_waypoint(waypoint, toPrint):
    global waypoints, pub_waypoint, path_complete
    if toPrint:
        rospy.loginfo('Commanding waypoint %.2f, %2f', waypoint[0], waypoint[1])
    path_complete.data = False
    msg_waypoint = ME439WaypointXY()
    msg_waypoint.x = waypoint[0]
    msg_waypoint.y = waypoint[1]

    pub_waypoint.publish(msg_waypoint)


def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete

    if msg_in.data:
        waypoint_number = waypoint_number + 1

    rospy.loginfo('Incrementing to waypoint %d', waypoint_number)

    # Handle the last waypoint:
    # If the last waypoint was reached, set "path_complete" and publish it
    if waypoint_number >= waypoints.shape[0]:
        path_complete.data = True
        # waypoint_number = waypoint_number - 1  # This line prevents an array
        # out of bounds error to make sure the node stayed alive.
    else:
        path_complete.data = False


def find_object():
    global pub_find_object
    pub_find_object.publish(Bool(True))


def update_location(msg):
    global x, y, theta
    x = msg.x
    y = msg.y
    theta = msg.theta


def process_sensor_data(msg):
    global states, state, x, y
    if state == states['Push']:
        # If there's not an object directly ahead, find it
        if msg.u0meters > .1:
            rospy.loginfo('Object lost; Scanning...')
            state = states['Scan']
            # "Go to" the current location
            publish_waypoint([x, y], True)
            find_object()


def found_object(msg):
    global states, state, waypoints, waypoint_number, goal, x, y
    # Store object location
    rospy.loginfo('Found object at: %.2f %.2f', msg.x, msg.y)

    # Scan => Reposition
    if state == states['Scan']:
        rospy.loginfo('Scan complete; repositioning')
        # Add msg's relative position to global robot position
        o_x = msg.x + x
        o_y = msg.y + y
        # Calculate line between object and goal
        dx = goal[0] - o_x
        dy = goal[1] - o_y
        d = np.sqrt(dx**2 + dy**2)
        # Unit dx
        udx = dx / d

        m = dy / dx
        b = goal[1] - m * goal[0]
        # Extrapolate line and make start and block waypoints
        cx = o_x
        # Start .5m away from the object
        xs = [cx - 0.5 * udx, cx - 0.3 * udx, cx - 0.1 * udx]
        waypoints = np.array([[x1, m * x1 + b] for x1 in xs])
        waypoint_number = 0

        # rospy.loginfo('Locations: R=(%.2f, %.2f), O=(%.2f, %.2f), W=(%.2f, %.2f)',
        #               x, y, o_x, o_y, waypoints[0, 0], waypoints[0, 1])

        state = states['Reposition']


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
