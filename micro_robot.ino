#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#define LED_ 4
#define BUTTON_LEFT 14
#define BUTTON_RIGHT 27
#define run_every(t) for (static uint16_t last_; \
                          (uint16_t)(uint16_t(millis()) - last_) >= (t); \
                          last_ += (t))
WiFiUDP udp;
HardwareSerial LiDARSerial(1);  // Sử dụng UART1 trên ESP32
const int RX_PIN = 16;          // RX của ESP32 (kết nối với TX của LDS-007)
const int TX_PIN = 17;          // TX của ESP32 (kết nối với RX của LDS-007)
const long BAUD_RATE = 115200;  // Tốc độ baud cho LDS-007
const char* ROS_IP = "172.20.10.6";
const unsigned int ROS_PORT = 12345;
int last_speed[2] = { 0, 0 };
// Cấu hình các biến cho dữ liệu
float rpms = 0;  // Vòng quay trên phút
const int range_max_now = 6000;
const int range_min_now = 50;
#define MIN_SPEED 35
#define MIN_SPEED_RUN 50
const char* ssid = "Thinh_wifi";
const char* password = "thinhdaica1";

const int dir[2] = { 18, 23 };
const int pwm[2] = { 19, 5 };
#define M_LEFT 1
#define M_RIGHT 0
int speed_motor_now[2] = { 0, 0 };
void setupLidar() {
  LiDARSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // Gửi lệnh để khởi động LDS-007
  LiDARSerial.write('$');
  LiDARSerial.print("startlds$");
}
void task_receive_signal() {
  int16_t incomingPacket[2];  // Bộ đệm nhận dữ liệu cho int16

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read((uint8_t*)incomingPacket, sizeof(incomingPacket));
    if (len == sizeof(incomingPacket)) {
      int motor_speed_left = incomingPacket[0];
      int motor_speed_right = incomingPacket[1];

      Serial.print("Received motor speeds: ");
      Serial.print("Left = ");
      Serial.print(motor_speed_left);
      Serial.print(", Right = ");
      Serial.println(motor_speed_right);
    }
  }
}
void lidatTaskFake() {
  static unsigned long time_delay = millis();
  if (millis() - time_delay > 200) {
    uint16_t ranges[360];
    for (int i = 0; i < 360; i++) {
      ranges[i] = i * 10;
    }
    time_delay = millis();
    udp.beginPacket(ROS_IP, ROS_PORT);
    udp.write((uint8_t*)ranges, sizeof(ranges));
    udp.endPacket();
    static bool state_led=0;
    state_led=1-state_led;
    digitalWrite(LED_,state_led);
  }
}
void lidarTask() {
  uint8_t raw_bytes[1980];
  int start_count = 0;
  bool got_scan = false;


  // unsigned long time_ = millis();
  // Đọc một byte dữ liệu và kiểm tra trình tự bắt đầu của gói dữ liệu
  while (LiDARSerial.available() > 0) {
    uint8_t temp_byte = LiDARSerial.read();

    // Tìm kiếm trình tự bắt đầu
    if (start_count == 0 && temp_byte == 0xFA) {
      start_count = 1;
    } else if (start_count == 1 && temp_byte == 0xA0) {
      start_count = 0;
      got_scan = true;

      // Đọc toàn bộ gói tin còn lại
      LiDARSerial.readBytes(&raw_bytes[2], 1978);

      uint16_t ranges[360];
      float intensities[360];
      uint32_t motor_speed = 0;
      uint16_t good_sets = 0;

      // Xử lý các gói tin, 4 byte mỗi lần đọc
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
            // Serial.println(range);
            // float range_float = range / 1000.0;
            if (range >= range_max_now || range <= range_min_now || intensity <= 5) {
              range = 0;
            }

            ranges[index] = range;
            intensities[index] = intensity;
          }
        }
      }
      udp.beginPacket(ROS_IP, ROS_PORT);
      udp.write((uint8_t*)ranges, sizeof(ranges));
      udp.endPacket();
      // Serial.print("RPM: ");
      // Serial.println(rpms);
      // Serial.print("First distance: ");
      // Serial.println(ranges[200], 5);
      // Serial.print("First intensity: ");
      // Serial.println(intensities[200]);
    }
  }
}
void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
  udp.begin(ROS_PORT);
  Serial.println("UDP initialized.");
}
void control_motor_left(int speed) {

  bool direct = speed > 0 ? 0 : 1;
  static bool wait_init_motor = 0;
  static unsigned long wait_time = millis();
  if (last_speed[M_LEFT] == 0 && abs(speed) < 70 && abs(speed) > 0 && wait_init_motor == 0) {
    int dir_motor = speed > 0 ? 1 : -1;
    last_speed[M_LEFT] = 140 * dir_motor;
    speed = 140 * dir_motor;
    wait_init_motor = 1;
    if (direct) {
      analogWrite(pwm[M_LEFT], 255 - abs(speed));
      digitalWrite(dir[M_LEFT], 1);
    } else {
      analogWrite(pwm[M_LEFT], abs(speed));
      digitalWrite(dir[M_LEFT], 0);
    }
    wait_time = millis();
    // Serial.println("speed_left:" + String(speed));
    return;
  }
  if (wait_init_motor == 1) {
    if (millis() - wait_time > 100) {
      // digitalWrite(dir[motor],direct);

      if (direct) {
        analogWrite(pwm[M_LEFT], 255 - abs(speed));
        digitalWrite(dir[M_LEFT], 1);
      } else {
        analogWrite(pwm[M_LEFT], abs(speed));
        digitalWrite(dir[M_LEFT], 0);
      }
      wait_init_motor = 0;
    }
  } else {
    if (direct) {
      analogWrite(pwm[M_LEFT], 255 - abs(speed));
      digitalWrite(dir[M_LEFT], 1);
    } else {
      analogWrite(pwm[M_LEFT], abs(speed));
      digitalWrite(dir[M_LEFT], 0);
    }
  }
  // Serial.println("speed_left:" + String(speed));
  last_speed[M_LEFT] = speed;
}
void control_motor_right(int speed) {
  bool direct = speed > 0 ? 1 : 0;
  static bool wait_init_motor = 0;
  static unsigned long wait_time = millis();
  if (last_speed[M_RIGHT] == 0 && abs(speed) < 70 && abs(speed) > 0 && wait_init_motor == 0) {
    int dir_motor = speed > 0 ? 1 : -1;
    last_speed[M_RIGHT] = 140 * dir_motor;
    speed = 140 * dir_motor;
    wait_init_motor = 1;
    if (direct) {
      analogWrite(pwm[M_RIGHT], 255 - abs(speed));
      digitalWrite(dir[M_RIGHT], 1);
    } else {
      analogWrite(pwm[M_RIGHT], abs(speed));
      digitalWrite(dir[M_RIGHT], 0);
    }
    wait_time = millis();
    // Serial.println("speed_right:" + String(speed));
    return;
  }
  if (wait_init_motor == 1) {
    if (millis() - wait_time > 100) {
      // digitalWrite(dir[motor],direct);

      if (direct) {
        analogWrite(pwm[M_RIGHT], 255 - abs(speed));
        digitalWrite(dir[M_RIGHT], 1);
      } else {
        analogWrite(pwm[M_RIGHT], abs(speed));
        digitalWrite(dir[M_RIGHT], 0);
      }
      wait_init_motor = 0;
    }
  } else {
    if (direct) {
      analogWrite(pwm[M_RIGHT], 255 - abs(speed));
      digitalWrite(dir[M_RIGHT], 1);
    } else {
      analogWrite(pwm[M_RIGHT], abs(speed));
      digitalWrite(dir[M_RIGHT], 0);
    }
  }
  last_speed[M_RIGHT] = speed;
  // Serial.println("speed_right:" + String(speed));
}
void control_motor(int motor, int speed) {
  bool direct = speed > 0 ? 1 : 0;
  static bool wait_init_motor = 0;
  static unsigned long wait_time = millis();
  if (motor == M_LEFT) direct = 1 - direct;

  if (direct) {
    analogWrite(pwm[motor], 255 - abs(speed));
    digitalWrite(dir[motor], 1);
  } else {
    analogWrite(pwm[motor], abs(speed));
    digitalWrite(dir[motor], 0);
  }

  last_speed[motor] = speed;
}
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 2; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
    control_motor(i, 0);
  }
  pinMode(LED_,OUTPUT);
  pinMode(BUTTON_LEFT,INPUT_PULLUP);
  pinMode(BUTTON_RIGHT,INPUT_PULLUP);
  setupWiFi();
  // setupLidar();
  // xTaskCreate(lidarTask, "Send LiDAR Data", 2048, NULL, 1, NULL);

  // Tạo task để nhận tín hiệu từ server
  // xTaskCreate(task_receive_signal, "Receive Signal", 2048, NULL, 1, NULL);
}

void loop() {
  // Không cần xử lý trong hàm loop vì tất cả đều chạy trong các tasks
  // unsigned long time_ = millis();
  lidatTaskFake();
  // lidarTask();
  task_receive_signal();
  // unsigned long over = millis() - time_;
  // if (over >= 30) {
  //   Serial.print("over");
  //   Serial.println(over);
  // }
  // if (Serial.available()) {
  //   String c = Serial.readStringUntil(';');
  //   int index_now = c.indexOf("/");
  //   int index_kp_desired = c.indexOf(":");
  //   int dir_mor = c.indexOf(".");
  //   int index_ff = c.indexOf("f");

  //   if (index_now != -1) {
  //     // speed_linear = c.substring(0, index_now).toFloat();
  //     // angular_speed = (c.substring(index_now + 1).toFloat());

  //     speed_motor_now[0] = c.substring(0, index_now).toInt();
  //     speed_motor_now[1] = c.substring(index_now + 1).toInt();

  //     // for (int j = 0; j < 4; j++) speed_desired[j] = speed_now;
  //     // speed_desired[index_motor]=speed_now;
  //     // if (index_motor == 1) robot_speed.x = speed_now;
  //     // else if (index_motor == 2) robot_speed.y = speed_now;
  //     // else if (index_motor == 3) yaw_desired = speed_now * M_PI / 180.0;
  //     // index_motor_pid = index_motor;
  //     // Serial.print("speed:");

  //     // Serial.print(speed_desired[0]);
  //     // Serial.print(",");
  //     // Serial.println(speed_desired[1]);
  //   }
  // }
  run_every(20) {
    for (int i = 0; i < 2; i++) {
      if (abs(speed_motor_now[i]) > 0 && abs(speed_motor_now[i]) < MIN_SPEED) {
        speed_motor_now[i] = speed_motor_now[i] > 0 ? MIN_SPEED_RUN : -MIN_SPEED_RUN;
      }
    }
    control_motor_left(speed_motor_now[M_LEFT]);
    control_motor_right(speed_motor_now[M_RIGHT]);
  }
}
