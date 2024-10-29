#!/usr/bin/env python3

import rospy
import socket
import struct
import threading
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

class LidarUdpReceiver:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('lidar_udp_receiver', anonymous=True)
        self.width=0.5 #m
        # Lấy tham số từ file launch
        self.esp32_ip = rospy.get_param("~esp32_ip", "172.20.10.14")
        self.udp_port_lidar = rospy.get_param("~udp_port_lidar", 12345)
        self.udp_port_cmd_vel = rospy.get_param("~udp_port_cmd_vel", 12345)

        # Thiết lập socket
        self.sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_lidar.bind(("0.0.0.0", self.udp_port_lidar))
        self.sock_lidar.settimeout(0.1)  # Timeout cho recv để không bị block mãi

        self.sock_cmd_vel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Publisher và Subscriber
        self.pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # TF broadcaster

        # Thread cho việc nhận dữ liệu lidar
        self.lidar_thread = threading.Thread(target=self.receive_lidar_data)
        self.lidar_thread.daemon = True  # Đảm bảo thread sẽ kết thúc khi node dừng
        self.lidar_thread.start()

   
    def cmd_vel_callback(self, msg):
        # Chuyển đổi vận tốc thành hai giá trị (-255 đến 255)
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z 
        speed_left_m_s=linear_velocity-angular_velocity*self.width
        speed_right_m_s=linear_velocity+angular_velocity*self.width
        speed_pwm_left=(float(speed_left_m_s)*200.0/0.7)
        speed_pwm_right=(float(speed_right_m_s)*200.0/0.7)
        # Gửi dữ liệu qua UDP tới ESP32
        print(speed_pwm_left)
        print(speed_pwm_right)
        data = struct.pack('hh', int(speed_pwm_left), int(speed_pwm_right))
        self.sock_cmd_vel.sendto(data, (self.esp32_ip, self.udp_port_cmd_vel))

    def receive_lidar_data(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock_lidar.recvfrom(65535)
                lidar_range = [struct.unpack('H', data[i:i+2])[0] / 1000.0 for i in range(0, len(data), 2)]

                scan_msg = LaserScan()
                scan_msg.header = Header()
                scan_msg.header.stamp = rospy.Time.now()
                scan_msg.header.frame_id = "laser_frame"
                scan_msg.angle_min = 0.0
                scan_msg.angle_max = 2.0 * math.pi
                scan_msg.angle_increment = (2.0 * math.pi) / 360.0
                scan_msg.range_min = 0.05
                scan_msg.range_max = 5.5
                scan_msg.ranges = lidar_range

                self.pub_scan.publish(scan_msg)

            except socket.timeout:
                pass  # Không có dữ liệu, tiếp tục vòng lặp mà không chặn

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        lidar_receiver = LidarUdpReceiver()
        lidar_receiver.run()
    except rospy.ROSInterruptException:
        pass
