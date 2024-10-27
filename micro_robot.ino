#include <HardwareSerial.h>
#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/LaserScan.h>

const char* ssid = "Ngoi nha vui ve";
const char* password = "06011997";

HardwareSerial LiDARSerial(1);
const int RX_PIN = 16;
const int TX_PIN = 17;
const long BAUD_RATE = 115200;
const float range_max_now = 5.5;
const float range_min_now = 0.05;

ros::NodeHandle nh;
sensor_msgs::LaserScan scan_msg;
ros::Publisher lidar_pub("scan", &scan_msg);

float ranges[360];
float intensities[360];
float rpms = 0;
volatile bool new_scan_received = false;

TaskHandle_t lidarTaskHandle = NULL;
TaskHandle_t rosTaskHandle = NULL;

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void lidarTask(void* pvParameters) {
  uint8_t raw_bytes[1980];
  int start_count = 0;
  bool got_scan = false;

  while (true) {
    if (LiDARSerial.available() > 0) {
      uint8_t temp_byte = LiDARSerial.read();

      if (start_count == 0 && temp_byte == 0xFA) {
        start_count = 1;
      } else if (start_count == 1 && temp_byte == 0xA0) {
        start_count = 0;
        got_scan = true;
        LiDARSerial.readBytes(&raw_bytes[2], 1978);

        uint32_t motor_speed = 0;
        uint16_t good_sets = 0;

        for (int i = 0; i < 1980; i += 22) {
          if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22)) {
            good_sets++;
            motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
            rpms = motor_speed / good_sets / 64;

            for (int j = i + 4; j < i + 20; j += 4) {
              int index = (4 * i) / 22 + (j - 4 - i) / 4;
              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j + 1];
              uint8_t byte2 = raw_bytes[j + 2];
              uint8_t byte3 = raw_bytes[j + 3];

              uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
              uint16_t intensity = (byte3 << 8) + byte2;

              float range_float = range / 1000.0;
              if (range_float >= range_max_now || range_float <= range_min_now || intensity <= 5) {
                range_float = 0;
              }

              ranges[index] = range_float;
              intensities[index] = intensity;
            }
          }
        }

        // Lưu các thông số scan cho ROS
        scan_msg.angle_min = 0.0;
        scan_msg.angle_max = 2.0 * M_PI;
        scan_msg.angle_increment = (2.0 * M_PI / 360.0);
        scan_msg.range_min = range_min_now;
        scan_msg.range_max = range_max_now;
        scan_msg.ranges = ranges;
        scan_msg.intensities = intensities;
        

        // Cập nhật thời gian trong header và đánh dấu cờ new_scan_received
        scan_msg.header.stamp = nh.now();
        scan_msg.header.frame_id = "laser";
        new_scan_received = true;
      }
    }
  }
}

void rosTask(void* pvParameters) {
  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
  Serial.println("ROS Connected");

  while (true) {
    if (new_scan_received) {
      lidar_pub.publish(&scan_msg);
      new_scan_received = false;
    }
    nh.spinOnce();
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  LiDARSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  setupWiFi();

  nh.getHardware()->setConnection(ESP32WiFiClient());
  nh.initNode();
  nh.advertise(lidar_pub);

  xTaskCreatePinnedToCore(
    lidarTask, "LiDAR Task", 4096, NULL, 1, &lidarTaskHandle, 0
  );
  xTaskCreatePinnedToCore(
    rosTask, "ROS Task", 4096, NULL, 1, &rosTaskHandle, 1
  );
}

void loop() {
  // Không cần xử lý trong hàm loop vì tất cả đều chạy trong các tasks
}
