#!/usr/bin/env python3

import rospy
import struct
import math
import asyncio
import websockets
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class LidarWebSocketReceiver:
    def __init__(self):
        rospy.init_node('lidar_websocket_receiver', anonymous=True)
        self.width = 0.5  # Chiều rộng của robot

        # Thiết lập WebSocket server IP và cổng từ tham số ROS
        self.esp32_ip = rospy.get_param("~esp32_ip", "ws://192.168.4.1")
        self.websocket_port_cmd_vel = rospy.get_param("~websocket_port_cmd_vel", 80)
        self.loop = asyncio.get_event_loop()  # Thêm vòng lặp sự kiện
        # Publisher và Subscriber
        self.pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Khởi tạo biến kết nối WebSocket
        self.websocket = None

        # Chạy asyncio event loop cho WebSocket
        self.loop.run_until_complete(self.websocket_connect())
        # self.loop.create_task(self.run_websocket())
    

    async def websocket_connect(self):
        # Tạo kết nối WebSocket tới ESP32
        uri = f"{self.esp32_ip}:{self.websocket_port_cmd_vel}"
        self.websocket = await websockets.connect(uri)
        print(f"WebSocket connected to {uri}")

        # Khởi chạy nhận dữ liệu lidar
        await self.receive_lidar_data()
    def cmd_vel_callback(self, msg):
        print("CMD VEL GET")
        asyncio.run_coroutine_threadsafe(self.handle_cmd_vel(msg), self.loop)
    async def handle_cmd_vel(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        speed_left_m_s = linear_velocity - angular_velocity * self.width
        speed_right_m_s = linear_velocity + angular_velocity * self.width
        speed_pwm_left = int(speed_left_m_s * 200.0 / 0.7)
        speed_pwm_right = int(speed_right_m_s * 200.0 / 0.7)

        # Gửi dữ liệu qua WebSocket
        if self.websocket:
            data = struct.pack('hh', speed_pwm_left, speed_pwm_right)
            await self.websocket.send(data)

    async def receive_lidar_data(self):
        while not rospy.is_shutdown():
            # Nhận dữ liệu lidar từ ESP32 qua WebSocket
            try:
                if self.websocket:
                    data = await self.websocket.recv()
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
                    # print("Lidar data received and published")
            except websockets.ConnectionClosed:
                print("WebSocket connection closed, retrying...")
                await asyncio.sleep(1)
                await self.websocket_connect()
            except Exception as e:
                print(f"Error receiving lidar data: {e}")
                await asyncio.sleep(1)

if __name__ == '__main__':
    try:
        LidarWebSocketReceiver()
    except rospy.ROSInterruptException:
        pass
