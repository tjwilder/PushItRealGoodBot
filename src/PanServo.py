#!/usr/bin/python

# This pans the servo around the front 180-degree arc and retruns to center
# from mobrob_util.msg import ME439SensorsRaw
import numpy as np
import rospy
from std_msgs.msg import Bool, Int
import time
import traceback

import Adafruit_PCA9685

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
  print('Calculating trajectory and panning servo')
  # duration of trajectory
  # How often can you send a command to the servo?
  dt_traj = 1. / servo_pulse_frequency
  n = np.int(np.ceil(t_max / dt_traj))
  # Time array for the trajectory
  traj_time = np.linspace(0., t_max, n)
  # ANGLES array for the trajectory
  traj_ang = np.linspace(-90., 90., n)

  tstart = time.time()
  while time.time() - tstart < t_max:
    t_elapsed = time.time() - tstart
    angle_to_command = traj_ang[traj_time <= t_elapsed][-1]
    pulse_to_command = np.interp(angle_to_command, angle_range, us_range)
    command_servo(pulse_to_command)
    pub.publish(Int(int(angle_to_command)))
    print(angle_to_command)
    time.sleep(dt_traj / 10.)


def panner():
  global panning
  rospy.init_node('pan_servo', anonymous=False)

  # Register publisher and subscriber
  find_publisher = rospy.Publisher('/find_object', Bool, queue_size=1)
  angle_publisher = rospy.Publisher('/servo_angle', Int, queue_size=1)
  rospy.Subscriber('/find_object', Bool, find_object)

  while not rospy.is_shutdown():
    if not panning:
      time.sleep(.2)
      continue

    t_max = 5.
    pan(0, -90, t_max / 2, angle_publisher)
    time.sleep(1)
    pan(-90, 90, t_max, angle_publisher)
    print('Panning back to center')
    pan(90, 0, t_max / 2, angle_publisher)

    panning = False
    find_publisher.publish(Bool(False))


if __name__ == '__main__':
  try:
    panner()
  except rospy.ROSInterruptException:
    traceback.print_exc()
    command_servo(0)
    pass
