#!/usr/bin/python

# This pans the servo around the front 180-degree arc and retruns to center
# from mobrob_util.msg import ME439SensorsRaw
import numpy as np
import rospy
from std_msgs.msg import Bool, Int8
import time
import traceback

import Adafruit_PCA9685

from mobrob_util.msg import ME439SensorsProcessed
from geometry_msgs.msg import Pose2D
# Initialise the PWM device using the default address
pwm = Adafruit_PCA9685.PCA9685()

# Get parameters?
# integral_error_max = rospy.get_param('/vel_integral_limit')
servo_pulse_frequency = 60
# microseconds for each pulse
servo_pulse_duration_us = 1.0e6 / servo_pulse_frequency
# Counter is 12-bit, so 4096 levels (0-4095)
servo_pulse_width_to_count_multiplier = 1. / servo_pulse_duration_us * 4095.0

# Set the Frequency of all servo outputs
pwm.set_pwm_freq(servo_pulse_frequency)

# Shut down servos until they get a real command (temporary)
pwm.set_all_pwm(0, 0)

# angle range
angle_range = np.array([-90., 90.])
# microseconds range
us_range = np.array([410, 2500])

# Remember panning position
panning = False
panningCoords = []


# Function to command any servo with a given pulse width in microseconds
def command_servo(pulse_width_us):
  pulse_width_count = int(
      round(
          pulse_width_us
          * servo_pulse_width_to_count_multiplier))
  # set_pwm(servo_number, 0, pulse_width_count)
  pwm.set_pwm(0, 0, pulse_width_count)


def find_object(msg_in):
  global panning
  if msg_in.data:
    panning = True


def pan(start, end, t_max, pub):
  global angle
  print('Calculating trajectory and panning servo')
  # duration of trajectory
  # How often can you send a command to the servo?
  dt_traj = 1. / servo_pulse_frequency
  n = np.int(np.ceil(t_max / dt_traj))
  # Time array for the trajectory
  traj_time = np.linspace(0., t_max, n)
  # ANGLES array for the trajectory
  traj_ang = np.linspace(start, end, n)

  tstart = time.time()
  while time.time() - tstart < t_max:
    t_elapsed = time.time() - tstart
    angle_to_command = traj_ang[traj_time <= t_elapsed][-1]
    pulse_to_command = np.interp(angle_to_command, angle_range, us_range)
    command_servo(pulse_to_command)
    pub.publish(Int8(int(angle_to_command)))


    angle = angle_to_command

    rospy.loginfo(angle_to_command)
    time.sleep(dt_traj / 10.)
    
angle = 0
sensor_data = []      #create empty list of sensor data

def receivesensordata(msg_in):
    global angle
    global sensor_data
    sensor_data.append((angle, msg_in.a0))
    
def min_angledist():
    global sensor_data
    min_pair = (0,100)
    for data in sensor_data:
        if data[1] < min_pair[1]:
            min_pair = data
    return min_pair
    
def calc_obj_pos(pair):
    y_obj = 0.2 + np.cos(pair[0])*(pair[1]+0.1) #fix 0.2 and 0.1 offsets if needed
    x_obj = np.sin(pair[0])*(pair[1]+0.1)
    return x_obj,y_obj
    
def panner():
  global panning
  global sensor_data
  rospy.init_node('pan_servo', anonymous=False)

  # Register publisher and subscriber
  find_publisher = rospy.Publisher('/find_object', Bool, queue_size=1)
  angle_publisher = rospy.Publisher('/servo_angle', Int8, queue_size=1)
  rospy.Subscriber('/find_object', Bool, find_object)
  rospy.Subscriber('/sensors_data_processed',ME439SensorsProcessed,receivesensordata)
  found_publisher = rospy.Publisher('/found_object', Pose2D, queue_size=1)
  

  while not rospy.is_shutdown():
    if not panning:
      time.sleep(.2)
      continue

    sensor_data = []
    t_max = 8.
    pan(-25, -90, t_max / 2, angle_publisher)

    pan(-90, 90, t_max, angle_publisher)
    rospy.loginfo('Panning back to center')
    pan(90, -25, t_max / 2, angle_publisher)

    panning = False
    find_publisher.publish(Bool(False))
    pair = min_angledist()
    xy = calc_obj_pos(pair)
    Pose = Pose2D()
    Pose.x = xy[0]
    Pose.y = xy[1]
    rospy.logerr('Publishing pose %.2f, %.2f', xy[0], xy[1])
    rospy.logerr('Smallest: [%.2f, %.4f]', pair[0], pair[1])
    rospy.logerr('Printing sensor_data')
    for data in sensor_data:
        rospy.logerr('[%.2f, %.4f]', data[0], data[1])
    found_publisher.publish(Pose)

if __name__ == '__main__':
  try:
    panner()
  except rospy.ROSInterruptException:
    traceback.print_exc()
    command_servo(0)
    pass
