import math
import rclpy
from rclpy.node import Node
from collections import deque
from threading import Thread
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from numpy import zeros as np_zeros
import serial
import struct

# Class for a node
class VDCS_Receiver(Node):
    # Init function
    def __init__(self):
        # Init and define the node name
        super().__init__('vdcs_receiver')
        # Open serial port, and setup timeout to prevent blocking
        self.ser = serial.Serial('/dev/teensy', 115200, timeout=0.01)

        # Subscriber for receive control signal
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.recv_Control, 3)
        # Publisher to publish the odometry signal
        self.odom_pub = self.create_publisher(Odometry, "/odom/unfiltered", 3)

        # Timer to handle the message publish
        self.timer_odom = self.create_timer(0.02, self.odomPubTime)  # 50Hz

        # Cov value for pose and twist
        pose_cov = 0.0001
        twist_cov = 0.00001

        # Frame name
        self.header_frame = 'odom'
        self.child_frame = 'base_footprint'

        # Odometry stores
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_rz = 0.0

        # Time packet exchanged?
        self.time_sync = False
        # First robot speed packet received?
        self.first_rspd_recv = False

        # Generate covariance array
        self.twist_covariance = np_zeros(36)
        self.pose_covariance = np_zeros(36)
        self.twist_covariance[0] = twist_cov
        self.twist_covariance[7] = twist_cov
        self.twist_covariance[35] = twist_cov
        self.pose_covariance[0] = pose_cov
        self.pose_covariance[7] = pose_cov
        self.pose_covariance[35] = pose_cov

        # Deque to store the robot_speed data received from VDCS
        self.robot_speed_duque = deque(maxlen=3)
        # Deque to store the control_speed data received from topic
        self.control_speed_duque = deque(maxlen=3)

        # Update the last send time
        # For thread
        now_time = self.get_clock().now().nanoseconds
        self.last_ping_time = now_time
        # For node(main thread)
        self.last_odom_time = now_time

        # Time difference between VDCS timer and ROS Environment
        self.time_diff_vdcs = 0

        # Maybe have remaining data from the MCU's buffer, clear it!
        self.get_logger().info(f"Clear remaining datas...")
        in_waiting = 0
        # Increase timeout to 1s to ensure remaining data send
        self.ser.timeout = 1
        while(True):
            buf = self.ser.read(1)
            in_waiting += self.ser.in_waiting + len(buf)
            if(self.ser.in_waiting + len(buf) > 0):
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            else:
                self.get_logger().info(f"Already clear remaining {in_waiting} bytes data in serial!")
                # Back timeout to 10ms
                self.ser.timeout = 0.01
                break
        
        # Split a thread to receive data from serial
        self.rw_loop_thread = Thread(target=self.rw_loop, daemon=True)
        self.rw_loop_thread.start()

    # Calculate checksum by XOR method
    # Just put the data for all payload, it will calculate the all payload checksum
    def chksum_cal(self, data):
        chksum = 0
        for data_byte in data:
            chksum ^= data_byte
        return chksum

    # For debug purpose, print hex data
    def print_bytes_hex(self, byte_array):
        for b in byte_array:
            print(f"{b:02X}", end=' ')
        print()  # for newline after printing all bytes

    # Thread for the serial receive and send
    def rw_loop(self):
        # Some variables for receiving
        recv_step = 0
        buf = b''
        # Run thread's program in a loop
        while(True):
            # Get the wall clock's time
            now_time = self.get_clock().now().nanoseconds
            # To start the send function, need to sync time first (Ping vdcs will start send data)
            # Disconnect handshake will implement at future
            if(self.time_sync):
                # Ping VDCS each 3 seconds (Timeout of VDCS is 5 seconds)
                if((now_time - self.last_ping_time) / 1e6 > 3000):
                    # Ping vdcs
                    # print("Ping once!")
                    self.ping_vdcs()

                # If control speed deque not empty, means new control speed command arrives
                # So need to send it
                if (len(self.control_speed_duque) > 0):
                    # Get latest speed in deque
                    vx, vy, vrz = self.control_speed_duque[-1]
                    # Clear the remaining old value
                    self.control_speed_duque.clear()
                    # Make the packet for control speed
                    # Header
                    packet = bytearray(30)
                    packet[0:2] = b'Ac'
                    # Length
                    packet[2] = 24
                    # Payload
                    # Pack vx, vy, and vrz as IEEE 754 floats
                    # < = Little-endian, f = float
                    packet[3:7]   = struct.pack('<f', vx)
                    packet[7:11]  = struct.pack('<f', vy)
                    packet[23:27] = struct.pack('<f', vrz)
                    # Checksum over payload (bytes 3 to 26)
                    payload = packet[3:27]
                    packet[27] = self.chksum_cal(payload)
                    # Footer
                    packet[28:30] = b'pk'
                    # Send it
                    self.ser.write(packet)
            else:
                # Send time packet at very first time
                # Send it every 500ms, 因為ping的timer此時還不會用到，因此挪來這邊先用
                if((now_time - self.last_ping_time) / 1e6 > 500):
                    # Reset timer
                    self.last_ping_time = now_time
                    self.get_logger().info("Send time packet!")
                    # One line create time request packet
                    packet = b'As\x04Time' + self.chksum_cal(b'Time').to_bytes(1, 'little') + b'pk'
                    # Send time request packet
                    self.ser.write(packet) 
            # Have a header length data in the buffer
            # Packet format:
            # head(2) len(1) payload(len) chksum(1) foot(2) 
            # Read first byte
            if recv_step == 0:
                header = self.ser.read(1)
                # Not equal the packet head, skip this package
                if header != b'A':
                    continue
                # Continue read the header
                header += self.ser.read(1)
                # And read the length
                buf = self.ser.read(1)
                # Check header vaild
                if header == b'At' or header == b'Ar':
                    recv_step = 1
                else:
                    self.get_logger().warn("Got uncorrect header!")
            elif recv_step == 1:
                # Read length
                len_i = int.from_bytes(buf, byteorder='little')
                # Check data remains larger than length
                if self.ser.in_waiting < len_i + 3:
                    # Not larger, wait data arrives
                    continue
                # while self.ser.in_waiting < len_i + 3:
                #     time.sleep(0.001)
                # Receive remaining data
                buf = self.ser.read(len_i + 3)
                # Slice the footer
                footer = buf[-2:]
                # Footer check
                if footer == b'pk':
                    recv_step = 2
            elif recv_step == 2:
                # Slice the payload
                payload = buf[:-3]
                # Slice the checksum
                chksum = buf[-3]
                # Chksum check
                chksum_calc = self.chksum_cal(payload)
                # Check checksum, one byte can compare with int
                if chksum_calc == chksum:
                    recv_step = 3
            elif recv_step == 3:
                # Data vaild, 按照header進行下一步處理
                if header == b'At':  # Control Response
                    self.handle_time_packet(payload)
                elif header == b'Ar':  # Robot Speed
                    self.handle_robotspeed_packet(payload)
                # Back to first step
                recv_step = 0

    # Ping function
    def ping_vdcs(self):
        # Reset timer
        self.last_ping_time = self.get_clock().now().nanoseconds
        # Create ping packet use one line
        packet = b'As\x04Ping' + self.chksum_cal(b'Ping').to_bytes(1, 'little') + b'pk'
        # Send packet
        self.ser.write(packet)

    # Time packet
    def handle_time_packet(self, payload):
        # Unpack the timer value of VDCS's controller
        time_base = int.from_bytes(payload, 'little')
        # Get the current wall clock time
        now_time = self.get_clock().now().nanoseconds
        self.get_logger().info(f"Time from base = {time_base}")
        # Calculate the difference
        self.time_diff_vdcs = now_time - (time_base * 1e6)
        # Reset odom time
        self.last_odom_time = now_time
        # Time synced, can start exchange others packet!
        self.time_sync = True

        # First ping to get data immediately
        self.get_logger().info("First ping!")
        self.ping_vdcs()
        

    # Robot speed packet
    def handle_robotspeed_packet(self, payload):
        # self.print_bytes_hex(payload)
        # Unpack timestamp
        timestamp = int.from_bytes(payload[:4], 'little')
        # Unpack velocities
        vx = struct.unpack('f', payload[4:8])[0]
        vy = struct.unpack('f', payload[8:12])[0]
        vrz = struct.unpack('f', payload[24:])[0]
        # print("RP: Ts =", timestamp, ", vx =", vx, ", vy =", vy, ", vrz =", vrz)
        # Append to duque to let thread send it to ROS
        self.robot_speed_duque.append((timestamp, vx, vy, vrz))

    # Timer to handle odom message and publish
    def odomPubTime(self):
        # Check the duque, if it have data need to publish
        if(len(self.robot_speed_duque) > 0):
            # Grab the latest velocity data
            timestamp, vx, vy, vrz = self.robot_speed_duque[-1]
            # Clear the remaining old value
            self.robot_speed_duque.clear()
            # Calculate odom time based on vdcs measured
            odom_time_now = ((timestamp * 1e6) + self.time_diff_vdcs)
            # Delta time in seconds
            dt = (odom_time_now - self.last_odom_time) / 1e9
            # Update odom time
            self.last_odom_time = odom_time_now
            # Calculate delta displacement
            delta_x = (vx * math.cos(self.odom_rz) - vy * math.sin(self.odom_rz)) * dt
            delta_y = (vx * math.sin(self.odom_rz) + vy * math.cos(self.odom_rz)) * dt
            delta_rz = vrz * dt
            # Add delta displacement to odom
            self.odom_x += delta_x
            self.odom_y += delta_y
            self.odom_rz += delta_rz
            # Pack message
            odom_msg = Odometry()
            # Euler angle to Quaternion
            q = quaternion_from_euler(0, 0, self.odom_rz)
            # Frame ID
            odom_msg.header.frame_id = self.header_frame
            odom_msg.child_frame_id = self.child_frame
            # Timestamp(Convert to ROS's Time format)
            odom_msg.header.stamp.sec = int(odom_time_now // 1e9)
            odom_msg.header.stamp.nanosec = int(odom_time_now % 1e9)
            # Pose part
            odom_msg.pose.pose.position.x = self.odom_x
            odom_msg.pose.pose.position.y = self.odom_y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.pose.covariance = self.pose_covariance
            # Twist part
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = vrz
            odom_msg.twist.covariance = self.twist_covariance
            # Publish message
            self.odom_pub.publish(odom_msg)
            # Hint user
            if not(self.first_rspd_recv):
                self.first_rspd_recv = True
                self.get_logger().info("First robotspeed message received and published!")

    def recv_Control(self, msg):
        # Put new command in deque and let thread send it to vdcs
        self.control_speed_duque.append((msg.linear.x, msg.linear.y, msg.angular.z))

# Init the node
def main(args=None):
    rclpy.init(args=args)
    node = VDCS_Receiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
