#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32MultiArray

class STMEncoderNode:
    def __init__(self):
        rospy.init_node("wheel_interface_node")

        # ROS publishers/subscribers
        self.wheel_pub = rospy.Publisher(
            "/wheel_ticks", Float32MultiArray, queue_size=10
        )
        self.wheel_sub = rospy.Subscriber(
            "/wheel_speeds", Float32MultiArray, self.wheel_speed_callback
        )

        # Serial port parameters
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB2")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)

        rospy.loginfo("Wheel interface node started, reading from %s", self.serial_port)

        # Send RESET immediately to STM32
        self.reset_encoders()

        # Calibration offset: will store first encoder reading
        self.offset_ticks = None

        self.run()

    def reset_encoders(self):
        """Send RESET command to STM32 to zero the encoders"""
        try:
            self.ser.write(b"RESET\n")
            rospy.loginfo("Sent RESET command to STM32")
        except serial.SerialException as e:
            rospy.logwarn("Failed to send RESET: {}".format(e))

    def wheel_speed_callback(self, msg):
        """Send wheel speeds to STM32 as a SET command."""
        if len(msg.data) != 4:
            rospy.logwarn("Wheel speed message must have 4 elements")
            return
        line = "SET {} {} {} {}\n".format(
            int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[3])
        )
        try:
            self.ser.write(line.encode('utf-8'))
        except serial.SerialException as e:
            rospy.logwarn("Failed to write wheel speeds: {}".format(e))

    def run(self):
        """Main loop to read serial data from STM32 and publish encoder ticks."""
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue

                # Encoder line: ENC:<fl>\t<fr>\t<rl>\t<rr>
                if line.startswith("ENC:"):
                    _, data = line.split(":", 1)
                    tick_strings = [x for x in data.split('\t') if x.strip() != '']
                    if len(tick_strings) == 4:
                        try:
                            ticks = [int(x) for x in tick_strings]

                            # --- Calibrate / offset ---
                            if self.offset_ticks is None:
                                self.offset_ticks = ticks
                                rospy.loginfo("Encoder calibration offset set: {}".format(self.offset_ticks))

                            # Subtract offset to start ROS ticks from 0
                            calibrated_ticks = [ticks[i] - self.offset_ticks[i] for i in range(4)]

                            msg = Float32MultiArray()
                            msg.data = calibrated_ticks
                            self.wheel_pub.publish(msg)
                        except ValueError as e:
                            rospy.logwarn("Failed to parse ticks: {} ({})".format(tick_strings, e))
                    else:
                        rospy.logwarn("Incomplete encoder data: '{}'".format(line))
                else:
                    # STM startup/info messages
                    rospy.loginfo_throttle(10, "STM32 says: {}".format(line))

            except serial.SerialException as e:
                rospy.logerr("Serial error: {}".format(e))
            except UnicodeDecodeError:
                rospy.logwarn("Failed to decode serial line")


if __name__ == "__main__":
    try:
        STMEncoderNode()
    except rospy.ROSInterruptException:
        pass
