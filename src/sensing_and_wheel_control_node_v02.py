#!/usr/bin/env python

import numpy as np
import rospy
import serial
import traceback
from pololu_drv8835_rpi import motors  # MAX_SPEED is 480 (hard-coded)

import encoders_and_motors_v01 as encmot

from mobrob_util.msg import (ME439SensorsRaw, ME439WheelSpeeds,
                             ME439MotorCommands, ME439WheelAngles,
                             ME439WheelDisplacements)

# ==============================================================================
# # Get parameters from rosparam
# ==============================================================================
counts_per_encoder_revolution = rospy.get_param(
    '/counts_per_encoder_revolution')  # 12.0
gear_ratio = rospy.get_param('/gear_ratio')  # standard ME439 robot: 120.0
wheel_radius = rospy.get_param('/wheel_diameter_actual') / 2.0

e0_direction_sign = rospy.get_param('/left_encoder_sign')
e1_direction_sign = rospy.get_param('/right_encoder_sign')
m0_direction_sign = rospy.get_param('/left_motor_sign')
m1_direction_sign = rospy.get_param('/right_motor_sign')

integral_error_max = rospy.get_param('/vel_integral_limit')
integral_resetting = rospy.get_param('/integral_resetting')
cmd_rate_of_change_max = rospy.get_param('/cmd_rate_of_change_max')
motor_command_max = rospy.get_param('/motor_command_max')

Kf0 = rospy.get_param('/vel_left_f')  #
Kp0 = rospy.get_param('/vel_left_p')
Ki0 = rospy.get_param('/vel_left_i')
Kd0 = rospy.get_param('/vel_left_d')
Kf1 = rospy.get_param('/vel_right_f')  #
Kp1 = rospy.get_param('/vel_right_p')
Ki1 = rospy.get_param('/vel_right_i')
Kd1 = rospy.get_param('/vel_right_d')

# Max encoder increment (full speed) - useful for eliminating errors
enc_increment_max = 1000


# ==============================================================================
# # Set up a system to coordinate the motor closed-loop control
# ==============================================================================
#   (Motor control is not ROS-friendly using ordinary messages due to delays.)
def talker():
    # Launch a node called "sensing_and_wheel_control_node"
    rospy.init_node('sensing_and_wheel_control_node', anonymous=False)

    # ==============================================================================
    #     # Create two Quadrature Encoder instances, one for each wheel
    # ==============================================================================
    # constructor function call: new_variable =
    # encmot.quadrature_encoder(serial_string_identifier,
    # counts_per_encoder_revolution, gear_ratio, wheel_radius,
    # forward_encoder_rotation_sign)
    qe0 = encmot.quadrature_encoder("E0", counts_per_encoder_revolution,
                                    gear_ratio, wheel_radius,
                                    e0_direction_sign)
    qe1 = encmot.quadrature_encoder("E1", counts_per_encoder_revolution,
                                    gear_ratio, wheel_radius,
                                    e1_direction_sign)

    # ==============================================================================
    #     # Create two motor controller instances, one for each wheel
    # ==============================================================================
    # constructor function call: FPID_controller (): new_object =
    # encmot.FPID_controller(motor,Kf, Kp,Ki,Kd, error_integral_limit=np.inf,
    # integral_resetting = True, motor_command_max_rate_of_change=1500.,
    # forward_motor_command_sign=1)
    mc0 = encmot.FPID_controller(
        motor=motors.motor1,
        Kf=Kf0,
        Kp=Kp0,
        Ki=Ki0,
        Kd=Kd0,
        error_integral_limit=integral_error_max,
        integral_resetting=integral_resetting,
        motor_command_max_change_rate=cmd_rate_of_change_max,
        motor_command_max=motor_command_max,
        forward_motor_command_sign=m0_direction_sign)
    mc1 = encmot.FPID_controller(
        motor=motors.motor2,
        Kf=Kf1,
        Kp=Kp1,
        Ki=Ki1,
        Kd=Kd1,
        error_integral_limit=integral_error_max,
        integral_resetting=integral_resetting,
        motor_command_max_change_rate=cmd_rate_of_change_max,
        motor_command_max=motor_command_max,
        forward_motor_command_sign=m1_direction_sign)

    # ==============================================================================
    #     # Here start a Subscriber to the "wheel_speeds_desired" topic.
    # ==============================================================================
    rospy.Subscriber('/wheel_speeds_desired', ME439WheelSpeeds,
                     set_wheel_speed_targets, [mc0, mc1])

    # ==============================================================================
    #     # Start the loop that listens and publishes.
    # ==============================================================================
    # Give it an argument of the [qe0,qe1] encoders and the [mc0,mc1] motor
    # controllers so it can update them all.
    serial_port_publisher([qe0, qe1], [mc0, mc1])


# ==============================================================================
# # Function to send updated Commands directly to the motor controllers.
# ==============================================================================
#   These commands come from the Subscriber above.
def set_wheel_speed_targets(msg_in, motor_controllers):
    motor_controllers[0].update_target_value(msg_in.v_left)
    motor_controllers[1].update_target_value(msg_in.v_right)


# ==============================================================================
# # Main loop function that listens to the serial port, calls motor control,
# # and publishes the sensor data
# ==============================================================================
def serial_port_publisher(quad_encoders, mot_controllers):
    # Create the publisher for the topic "/sensors_data_raw", with message
    # type "ME439SensorsRaw"
    pub_sensors = rospy.Publisher('/sensors_data_raw',
                                  ME439SensorsRaw,
                                  queue_size=10)
    # Create the publisher for the topic "/motor_commands", with message type
    # "ME439MotorCommands"
    pub_motorcommands = rospy.Publisher('/motor_commands',
                                        ME439MotorCommands,
                                        queue_size=10)
    # Create the publisher for the topic "/robot_wheel_angles", with message
    # type "ME439WheelAngles"
    pub_robot_wheel_angles = rospy.Publisher('/robot_wheel_angles',
                                             ME439WheelAngles,
                                             queue_size=10)
    # Create the publisher for the topic "/robot_wheel_displacements", with
    # message type "ME439WheelDisplacements"
    pub_robot_wheel_displacements = rospy.Publisher(
        '/robot_wheel_displacements', ME439WheelDisplacements, queue_size=10)

    # Data comes in on the Serial port. Set that up and start it.
    # ----------setup serial--------------
    ser = serial.Serial('/dev/ttyS0')  # serial port to alamode
    ser.baudrate = 115200
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1  # one second time out.

    ser.flushInput()
    ser.readline()

    # We could also put data in it right away - here initializing to zero.
    pub_sensors_message = ME439SensorsRaw()
    pub_sensors_message.e0 = 0
    pub_sensors_message.e1 = 0
    pub_sensors_message.a0 = 0
    pub_sensors_message.a1 = 0
    pub_sensors_message.a2 = 0
    pub_sensors_message.a3 = 0
    pub_sensors_message.a4 = 0
    pub_sensors_message.a5 = 0
    pub_sensors_message.u0 = 0
    pub_sensors_message.u1 = 0
    pub_sensors_message.u2 = 0

    pub_motorcommands_message = ME439MotorCommands()
    pub_motorcommands_message.cmd0 = 0
    pub_motorcommands_message.cmd1 = 0

    robot_wheel_angles_message = ME439WheelAngles()
    robot_wheel_angles_message.ang_left = 0.
    robot_wheel_angles_message.ang_right = 0.

    robot_wheel_displacements_message = ME439WheelDisplacements()
    robot_wheel_displacements_message.d_left = 0.
    robot_wheel_displacements_message.d_right = 0.

    t0_previous = t1_previous = rospy.get_rostime()

    init0 = 1
    init1 = 1
    e0 = 0
    e1 = 0
    # MAIN LOOP to keep loading the message with new data.
    # NOTE that at the moment the data are coming from a separate thread, but
    # this will be replaced with the serial port line reader in the future.
    while not rospy.is_shutdown():
        newu2 = 0

        try:
            # serial port has strings like "e0:123456"
            # wait until we read an entire line
            line = ser.readline().decode().strip()
            line = line.split(":")
            data_type = line[0]
            data_value = int(line[1])
            # only use it as an encoder0 reading if it is one.
            if data_type == 'E0':
                # runs if it's the initialization step or if the encoder
                # increment is reasonable (not obviously erroneous)
                if init0 or np.abs(int(data_value) - e0) < enc_increment_max:
                    e0 = data_value  # Here is the actual encoder reading.
                    init0 = 0

                    pub_sensors_message.e0 = e0
                    t0 = rospy.get_rostime()  # or time.time()
                    dt0 = (t0 - t0_previous).to_sec()
                    t0_previous = t0  # store it for the next round
                    quad_encoders[0].update(e0, dt0)
                    mot_controllers[0].update_current_value(
                        quad_encoders[0].meters_per_second, dt0)
                    pub_motorcommands_message.cmd0 = int(
                        mot_controllers[0].motor_command)
                    robot_wheel_angles_message.ang_left = quad_encoders[
                        0].radians
                    robot_wheel_displacements_message.d_left = quad_encoders[
                        0].meters
            elif data_type == 'E1':
                # runs if it's the initialization step or if the encoder
                # increment is reasonable (not obviously erroneous)
                if init1 or np.abs(int(data_value) - e1) < enc_increment_max:
                    e1 = data_value  # Here is the actual encoder reading.
                    init1 = 0

                    pub_sensors_message.e1 = e1
                    t1 = rospy.get_rostime()  # or time.time()
                    dt1 = (t1 - t1_previous).to_sec()
                    t1_previous = t1  # store it for the next round
                    # ******* Update the
                    quad_encoders[1].update(e1, dt1)
                    mot_controllers[1].update_current_value(
                        quad_encoders[1].meters_per_second, dt1)
                    pub_motorcommands_message.cmd1 = int(
                        mot_controllers[1].motor_command)
                    robot_wheel_angles_message.ang_right = quad_encoders[
                        1].radians
                    robot_wheel_displacements_message.d_right = quad_encoders[
                        1].meters
    #                print(cnt1)
            elif data_type == 'A0':
                pub_sensors_message.a0 = data_value
            elif data_type == 'U1':
                pub_sensors_message.u1 = data_value
            elif data_type == 'U2':
                pub_sensors_message.u2 = data_value
                newu2 = 1
            if newu2:
                pub_sensors_message.t = rospy.get_rostime()

                # Publish a message to the "/sensors_data_raw" topic.
                pub_sensors.publish(pub_sensors_message)
                # Publish a message to the "/motor_commands" topic.
                pub_motorcommands.publish(pub_motorcommands_message)
                # Publish a message to the "/robot_wheel_angles" topic
                pub_robot_wheel_angles.publish(robot_wheel_angles_message)
                # Publish a message to the "/robot_wheel_displacements" topic
                pub_robot_wheel_displacements.publish(
                    robot_wheel_displacements_message)

                # Log the info (optional)
                rospy.loginfo(pub_sensors)

        except Exception:
            traceback.print_exc()
            pass


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        traceback.print_exc()
        # Stop the wheels if this crashes or otherwise ends.
        stop_wheel_speeds_message = ME439WheelSpeeds()
        stop_wheel_speeds_message.v_left = 0
        stop_wheel_speeds_message.v_right = 0
        set_wheel_speed_targets(stop_wheel_speeds_message,
                                [motors.motor1, motors.motor2])
        pass
