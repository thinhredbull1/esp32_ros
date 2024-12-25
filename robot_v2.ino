#include <WiFi.h>
#include <WebSocketsServer.h>

#include "driver/uart.h"
#include "esp_system.h"


bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
// orientation/motion vars
#include "I2Cdev.h"


#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
class SimplePID {
private:
  float kp, kd, ki, umax;
  float eprev, eintegral, last_u;
public:
  SimplePID()
    : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
  void reset_all() {
    eintegral = 0;
    eprev = 0;
  }
  void setParams(float kpIn, float kiIn, float kdIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
    reset_all();
  }
  float compute(int value, int target, float deltaT) {
    if (target == 0) {
      reset_all();
      return 0;
    }
    int e = target - value;
    float dedt = (e - eprev) / (deltaT);
    if (abs((int)last_u) >= umax && (((e >= 0) && (eintegral >= 0)) || ((e < 0) && (eintegral < 0)))) {
      eintegral = eintegral;
    } else {
      eintegral += e * deltaT;
    }
    eintegral = constrain(eintegral, -120, 120);
    float u = kp * e + kd * dedt + ki * eintegral;
    if (u > umax) u = umax;
    else if (u < -umax) u = -umax;
    last_u = u;
    eprev = e;
    return u;
  }
  float GetKp() {
    return kp;
  }
  float GetKi() {
    return ki;
  }
  float GetKd() {
    return kd;
  }
};

// Khai báo UART1
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define LIDAR_UART_NUM UART_NUM_1
// Kích thước gói tin LIDAR
#define LIDAR_PACKET_SIZE 1978
const float wheel_cm = 6.2;
const float ENCODER_PULSES = 45.0;  ///  count
const float GEAR_RATIO = 1.00;      // robot move > khoang cach thuc te --> giam 31.38 30.25
const float cm_per_count = PI * wheel_cm / (ENCODER_PULSES * GEAR_RATIO);
const float a_coeff = 0.22826091;  // loc thong thap
const float b_coeff = 0.38586955;  // loc thong thap
uint8_t start_byte;
#define LED_ 4
#define BUTTON_LEFT 14
#define BUTTON_RIGHT 27
#define run_every(t) for (static uint16_t last_; \
                          (uint16_t)(uint16_t(millis()) - last_) >= (t); \
                          last_ += (t))
bool ff = 0;
WebSocketsServer webSocket(80);  // Cổng WebSocket Server
float yaw_send;
// HardwareSerial LiDARSerial(1);       // Sử dụng UART1 trên ESP32
const int RX_PIN = 16;                   // RX của ESP32 (kết nối với TX của LDS-007)
const int TX_PIN = 17;                   // TX của ESP32 (kết nối với RX của LDS-007)
const long BAUD_RATE = 115200;           // Tốc độ baud cho LDS-007
const char* ssid = "ESP32_lidar";        // Thay "your_SSID" bằng tên Wi-Fi của bạn
const char* password = "123456789";      // Thay "your_PASSWORD" bằng mật khẩu Wi-Fi của bạn
const char* ssid_2 = "Thinh_wifi";       // Thay "your_SSID" bằng tên Wi-Fi của bạn
const char* password_2 = "thinhdaica1";  // Thay "your_PASSWORD" bằng mật khẩu Wi-Fi của bạn
float last_speed[2] = { 0, 0 };
float speed_filter[2];
volatile int16_t speed_desired[2] = { 0, 0 };  //cm/s
// Cấu hình các biến cho dữ liệu
float rpms = 10;  // Vòng quay trên phút
const int range_max_now = 6000;
const int range_min_now = 50;
#define MIN_SPEED 35
#define MIN_SPEED_RUN 50
double PID_left_param[] = { 1.8, 0.0, 0.04 };   //0.645 0.242 0.025
double PID_right_param[] = { 2.0, 0.0, 0.09 };  // 0.65 0.215 0.25
const int freq = 25;
uint8_t raw_bytes[1980];
const int resolution = 8;
int dutyCycle = 200;
volatile bool connection_success = 0;
const int dir[2] = { 18, 23 };
const int pwm[2] = { 19, 5 };
const int encod_pin[2] = { 22, 21 };
const int pwmChannel[2] = { 0, 1 };
#define M_LEFT 1
#define M_RIGHT 0
volatile int speed_motor_now[2] = { 0, 0 };
SimplePID pid[2];
volatile int encoderCount[2] = { 0, 0 };
void IRAM_ATTR encoderLeftISR() {
  encoderCount[M_LEFT]++;
}
void mpu_dmp_init() {
  delay(500);
  //  join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize device
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(10);
  mpu.setYGyroOffset(-13);
  mpu.setZGyroOffset(29);
  mpu.setZAccelOffset(1556);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection

    //  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
  }
}
// ISR for Encoder 2
void IRAM_ATTR encoderRightISR() {
  encoderCount[M_RIGHT]++;
}
void setupLidar() {
  uart_write_bytes(LIDAR_UART_NUM, "$startlds$", 10);  // Lệnh khởi động
}
void initUart() {
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
  };

  uart_param_config(LIDAR_UART_NUM, &uart_config);
  uart_set_pin(LIDAR_UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(LIDAR_UART_NUM, 2048, 2048, 0, NULL, 0);
}
void processLidar() {
  static int start_count = 0;
  static bool got_scan = 0;
  if (start_count == 0) {
    int len = uart_read_bytes(LIDAR_UART_NUM, &start_byte, 1, 10 / portTICK_PERIOD_MS);
    if (len == 1) {
      if (start_byte == 0xFA) start_count = 1;
    }
  } else if (start_count == 1) {
    int len = uart_read_bytes(LIDAR_UART_NUM, &start_byte, 1, 10 / portTICK_PERIOD_MS);
    if (len == 1) {
      if (start_byte == 0xA0) {
        start_count = 2;
        got_scan = 1;
      }
    }
  } else if (start_count == 2) {
    int len = uart_read_bytes(LIDAR_UART_NUM, raw_bytes + 2, LIDAR_PACKET_SIZE, 100 / portTICK_PERIOD_MS);
    if (len == LIDAR_PACKET_SIZE) {
      uint16_t ranges[360];
      float intensities[360];
      uint32_t motor_speed = 0;
      uint16_t good_sets = 0;
      float rpms;
      // uint8_t check_byte[1980];
      // portENTER_CRITICAL(&timerMux);
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


            ranges[index] = range;
            intensities[index] = intensity;
          }
        }
      }
      uint8_t data[720];  // Mỗi giá trị là 2 byte
      for (int i = 0; i < 360; i++) {
        data[2 * i] = ranges[i] & 0xFF;
        data[2 * i + 1] = (ranges[i] >> 8) & 0xFF;
      }
      // portEXIT_CRITICAL(&timerMux);
      // Serial.println(ranges[200]);
      webSocket.broadcastBIN(data, sizeof(data));

      static bool state_led = 0;
      state_led = 1 - state_led;
      digitalWrite(LED_, state_led);
      // Serial.print("RPM: ");
      // Serial.println(rpms);
      // Serial.print("First distance: ");
      // Serial.println(ranges[200]);
      start_count = 0;
    }
  }
}
void reset_all_pid() {
  for (int i = 0; i < 2; i++) {
    pid[i].reset_all();
    last_speed[i] = 0;
  }
}
bool receive_speed_command() {
  if (Serial.available()) {
    String c = Serial.readStringUntil(';');
    int index_now = c.indexOf("/");
    int index_kp_desired = c.indexOf(":");
    int index_ff = c.indexOf("f");
    if (index_ff != -1) {
      ff = c.substring(0, index_ff).toInt();
    }
    if (index_now != -1) {
      speed_desired[M_RIGHT] = c.substring(0, index_now).toFloat();
      speed_desired[M_LEFT] = c.substring(index_now + 1).toFloat();
      return 1;
    }
    if (index_kp_desired != -1) {

      int index_cal = c.indexOf("#");
      if (index_cal != -1) {
        float new_kp = c.substring(0, index_kp_desired).toFloat();
        float new_ki = c.substring(index_kp_desired + 1, index_cal).toFloat();
        float new_kd = c.substring(index_cal + 1).toFloat();
        PID_left_param[0] = new_kp;
        PID_left_param[1] = new_ki;
        PID_left_param[2] = new_kd;
        PID_right_param[0] = new_kp;
        PID_right_param[1] = new_ki;
        PID_right_param[2] = new_kd;

        reset_all_pid();
        pid[M_LEFT].setParams(PID_left_param[0], PID_left_param[1], PID_left_param[2], 255);      //39.2 34.6
        pid[M_RIGHT].setParams(PID_right_param[0], PID_right_param[1], PID_right_param[2], 255);  //39.2 34.6
        for (int i = 0; i < 3; i++) {
          Serial.print(PID_left_param[i]);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  }
  return 0;
}

// void setupLidar() {
//   LiDARSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

//   // Gửi lệnh để khởi động LDS-007
//   LiDARSerial.write('$');
//   LiDARSerial.print("startlds$");
// }
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_BIN) {
    if (length == 4) {  // 2 int16_t, mỗi int16_t là 2 byte
      // portENTER_CRITICAL(&timerMux);

      int16_t sp_left = (int16_t)(payload[0] | (payload[1] << 8));
      int16_t sp_right = (int16_t)(payload[2] | (payload[3] << 8));
      // portEXIT_CRITICAL(&timerMux);
      speed_desired[M_LEFT] = (float)sp_left;
      speed_desired[M_RIGHT] = (float)sp_right;
      // Serial.printf("Received speed_left: %d, speed_right: %d\n", sp_left, speed_desired[M_RIGHT]);
    } else {
      Serial.println("Invalid binary payload length");
    }
  } else if (type == WStype_TEXT) {
    Serial.printf("Text message received: %s\n", payload);
  } else if (type == WStype_DISCONNECTED) {
    connection_success = 0;
    Serial.printf("Client [%u] disconnected!\n", num);
  } else if (type == WStype_CONNECTED) {
    connection_success = 1;
    setupLidar();
    Serial.printf("Client [%u] connected!\n", num);
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
    uint16_t lidarData[360];
    for (int i = 0; i < 360; i++) {
      lidarData[i] = random(500, 5500);  // Khoảng cách từ 0.5m đến 5.5m
    }

    // Đóng gói và gửi dữ liệu lidar qua WebSocket
    uint8_t data[720];  // Mỗi giá trị là 2 byte
    for (int i = 0; i < 360; i++) {
      data[2 * i] = lidarData[i] & 0xFF;
      data[2 * i + 1] = (lidarData[i] >> 8) & 0xFF;
    }
    // uint8_t header_lidar = 'l';                                   // Byte nhận diện cho dữ liệu lidar
    // webSocket.broadcastBIN(&header_lidar, sizeof(header_lidar));  // Gửi header
    webSocket.broadcastBIN(data, sizeof(data));
    static bool state_led = 0;
    state_led = 1 - state_led;
    digitalWrite(LED_, state_led);
  }
}
// void lidarTask() {
//   uint8_t raw_bytes[1980];
//   int start_count = 0;
//   bool got_scan = false;



//   // unsigned long time_ = millis();
//   // Đọc một byte dữ liệu và kiểm tra trình tự bắt đầu của gói dữ liệu
//   while (LiDARSerial.available() > 0) {

//     uint8_t temp_byte = LiDARSerial.read();

//     // Tìm kiếm trình tự bắt đầu
//     if (start_count == 0 && temp_byte == 0xFA) {
//       start_count = 1;
//     } else if (start_count == 1 && temp_byte == 0xA0) {
//       unsigned long time_=millis();
//       start_count = 0;
//       got_scan = true;

//       // Đọc toàn bộ gói tin còn lại
//       LiDARSerial.readBytes(&raw_bytes[2], 1978);
//      Serial.println(millis()-time_);
//       uint16_t ranges[360];
//       float intensities[360];
//       uint32_t motor_speed = 0;
//       uint16_t good_sets = 0;

//       // Xử lý các gói tin, 4 byte mỗi lần đọc
//       for (int i = 0; i < 1980; i += 22) {
//         if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22)) {

//           good_sets++;
//           motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
//           rpms = motor_speed / good_sets / 64;

//           for (int j = i + 4; j < i + 20; j += 4) {
//             int index = (4 * i) / 22 + (j - 4 - i) / 4;
//             uint8_t byte0 = raw_bytes[j];
//             uint8_t byte1 = raw_bytes[j + 1];
//             uint8_t byte2 = raw_bytes[j + 2];
//             uint8_t byte3 = raw_bytes[j + 3];

//             uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
//             uint16_t intensity = (byte3 << 8) + byte2;
//             // Serial.println(range);
//             // float range_float = range / 1000.0;
//             if (range >= range_max_now || range <= range_min_now || intensity <= 5) {
//               range = 0;
//             }

//             ranges[index] = range;
//             intensities[index] = intensity;
//           }
//         }
//       }
//       uint8_t data[720];  // Mỗi giá trị là 2 byte
//       for (int i = 0; i < 360; i++) {
//         data[2 * i] = ranges[i] & 0xFF;
//         data[2 * i + 1] = (ranges[i] >> 8) & 0xFF;
//       }
//       // webSocket.broadcastBIN(data, sizeof(data));
//       static bool state_led = 0;
//       state_led = 1 - state_led;
//       digitalWrite(LED_, state_led);
//       // Serial.print("RPM: ");
//       // Serial.println(rpms);
//       // Serial.print("First distance: ");
//       Serial.println(ranges[200]);
//       // Serial.print("First intensity: ");
//       // Serial.println(intensities[200]);

//     }
//   }
// }
// void lidarTaskLoop(void* pvParameters) {
//   while (true) {
//     if (connection_success) lidarTask();  // Gọi hàm đọc dữ liệu từ LiDAR
//     vTaskDelay(10 / portTICK_PERIOD_MS);  // Nghỉ 10ms giữa các lần đọc
//   }
// }
void setupWiFiEsp() {
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();  //mặc định là 192.168.4.1
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println("WIFI SUCCESS");
}
void WifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid_2, password_2);
  Serial.println("");
  Serial.print("Connecting to: ");
  Serial.println(ssid_2);
  Serial.print("Password: ");
  Serial.println(password_2);

  // try to connect with Wifi network about 8 seconds
  unsigned long currentMillis = millis();

  unsigned long previousMillis = currentMillis;
  while (WiFi.status() != WL_CONNECTED && currentMillis - previousMillis <= 8000) {
    delay(500);
    Serial.print(".");
    currentMillis = millis();
  }

  // if failed to connect with Wifi network set NodeMCU as AP mode
  IPAddress myIP;
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("*WiFi-STA-Mode*");
    Serial.print("IP: ");
    myIP = WiFi.localIP();
    Serial.println(myIP);
    digitalWrite(LED_, HIGH);  // Wifi LED on when connected to Wifi as STA mode
    delay(2000);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    myIP = WiFi.softAPIP();
    Serial.println("");
    Serial.println("WiFi failed connected to ");
    Serial.println("");
    Serial.println("*WiFi-AP-Mode*");
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    digitalWrite(LED_, LOW);  // Wifi LED off when status as AP mode
    delay(2000);
  }
}
void control_motor_left(int speed) {

  bool direct = speed > 0 ? 1 : 0;
  static bool wait_init_motor = 0;
  static unsigned long wait_time = millis();
  if (last_speed[M_LEFT] == 0 && abs(speed) < 70 && abs(speed) > 0 && wait_init_motor == 0) {
    int dir_motor = speed > 0 ? 1 : -1;
    last_speed[M_LEFT] = 100 * dir_motor;
    speed = 100 * dir_motor;
    wait_init_motor = 1;
    if (direct) {
      // analogWrite(pwm[M_LEFT], 255 - abs(speed));
      ledcWrite(pwmChannel[M_LEFT], 255 - abs(speed));
      digitalWrite(dir[M_LEFT], 1);
    } else {
      // analogWrite(pwm[M_LEFT], abs(speed));
      ledcWrite(pwmChannel[M_LEFT], abs(speed));
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
        // analogWrite(pwm[M_LEFT], 255 - abs(speed));
        ledcWrite(pwmChannel[M_LEFT], 255 - abs(speed));
        digitalWrite(dir[M_LEFT], 1);
      } else {
        // analogWrite(pwm[M_LEFT], abs(speed));
        ledcWrite(pwmChannel[M_LEFT], abs(speed));
        digitalWrite(dir[M_LEFT], 0);
      }
      wait_init_motor = 0;
    }
  } else {
    if (direct) {
      // analogWrite(pwm[M_LEFT], 255 - abs(speed));
      ledcWrite(pwmChannel[M_LEFT], 255 - abs(speed));
      digitalWrite(dir[M_LEFT], 1);
    } else {
      // analogWrite(pwm[M_LEFT], abs(speed));
      ledcWrite(pwmChannel[M_LEFT], abs(speed));
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
    last_speed[M_RIGHT] = 100 * dir_motor;
    speed = 100 * dir_motor;
    wait_init_motor = 1;
    if (direct) {
      // analogWrite(pwm[M_RIGHT], 255 - abs(speed));
      ledcWrite(pwmChannel[M_RIGHT], 255 - abs(speed));
      digitalWrite(dir[M_RIGHT], 1);
    } else {
      ledcWrite(pwmChannel[M_RIGHT], abs(speed));
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
        // analogWrite(pwm[M_RIGHT], 255 - abs(speed));
        ledcWrite(pwmChannel[M_RIGHT], 255 - abs(speed));
        digitalWrite(dir[M_RIGHT], 1);
      } else {
        ledcWrite(pwmChannel[M_RIGHT], abs(speed));
        digitalWrite(dir[M_RIGHT], 0);
      }
      wait_init_motor = 0;
    }
  } else {
    if (direct) {
      // analogWrite(pwm[M_RIGHT], 255 - abs(speed));
      ledcWrite(pwmChannel[M_RIGHT], 255 - abs(speed));
      digitalWrite(dir[M_RIGHT], 1);
    } else {
      // analogWrite(pwm[M_RIGHT], abs(speed));
      ledcWrite(pwmChannel[M_RIGHT], abs(speed));
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
  motor = 1 - motor;
  // if (motor == M_LEFT) direct = 1 - direct;

  if (direct) {
    ledcWrite(pwmChannel[motor], 255 - abs(speed));
    digitalWrite(dir[motor], 1);
  } else {
    ledcWrite(pwmChannel[motor], abs(speed));
    digitalWrite(dir[motor], 0);
  }

  // last_speed[motor] = speed;
}
void websocketTask(void* pvParameters) {
  while (1) {
    // Xử lý WebSocket ở đây
    webSocket.loop();
    control_motor_left(speed_motor_now[M_LEFT]);
    control_motor_right(speed_motor_now[M_RIGHT]);
    // Tránh reset bằng cách delay một khoảng thời gian nhỏ
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay
  }
}
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 2; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
    ledcSetup(pwmChannel[i], freq, resolution);
    ledcAttachPin(pwm[i], pwmChannel[i]);
    // pinMode(encod_pin[i], INPUT_PULLUP);
    // control_motor(i, 0);
  }
  // init_mpu();
  mpu_dmp_init();
  pinMode(LED_, OUTPUT);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  WifiConnect();
  // attachInterrupt(digitalPinToInterrupt(encod_pin[M_LEFT]), encoderLeftISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(encod_pin[M_RIGHT]), encoderRightISR, RISING);
  // pid[M_LEFT].setParams(PID_left_param[0], PID_left_param[1], PID_left_param[2], 255);      //39.2 34.6
  // pid[M_RIGHT].setParams(PID_right_param[0], PID_right_param[1], PID_right_param[2], 255);  //39.2 34.6
  // setupWiFiEsp();
  initUart();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  // Không cần xử lý trong hàm loop vì tất cả đều chạy trong các tasks
  // unsigned long time_ = millis();
  // lidatTaskFake();
  // lidarTask();
  if (connection_success) processLidar();
  webSocket.loop();

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // 131.0
  }
  // unsigned long over = millis() - time_;
  // if (over >= 30) {
  //   Serial.print("over");
  //   Serial.println(over);
  // }

  static bool start_running = 0;
  static unsigned long time_c_stop = millis();
  // if (receive_speed_command()) {
  //   start_running = 1;
  //   time_c_stop = millis();
  // }
  static unsigned long time_ = millis();
  // if (millis() - time_ >= LOOP_MS) {
  //   static int count_print = 0;
  //   count_print += 1;
  //   time_ = millis();
  //   // compute();
  //   static float last_yaw = get_yaw();
  //   yaw_send = get_yaw() - last_yaw;
  //   last_yaw = get_yaw();

  //   if (count_print >= (1000 / LOOP_MS)) {
  //     count_print = 0;
  //     Serial.println(yaw_send);
  //   }
  // }
  static unsigned long timer_2 = millis();
  // run_every(50) {
  //   for(int i=0;i<2;i++)control_motor(i, speed_desired[i]);
  //   // portENTER_CRITICAL(&timerMux);
  //   // Serial.println(encoderCount[0]);
  //   // Serial.println(encoderCount[1]);
  //   // encoderCount[0] = 0;
  //   // encoderCount[1] = 0;
  //   // portEXIT_CRITICAL(&timerMux);
  // }

  // speed_desired[0]=10;
  if (millis() - timer_2 > 100) {
    timer_2 = millis();
    int delta_encod[2] = { 0, 0 };
    float speed_cm_s[2] = { 0, 0 };
    int speed_cmd[2] = { 0, 0 };
    for (int i = 0; i < 2; i++) {
      // portENTER_CRITICAL(&timerMux);

      // delta_encod[i] = encoderCount[i];
      // // if (speed_desired[i] > 0) delta_encod[i] = encoderCount[i];
      // // else if (speed_desired[i] < 0) delta_encod[i] = -encoderCount[i];
      // // else delta_encod[i] = 0;
      // // Serial.println(encoderCount[i]);
      // // Serial.println(delta_encod[i]);
      // encoderCount[i] = 0;
      // portEXIT_CRITICAL(&timerMux);
      // if (speed_desired[i] < 0) delta_encod[i] = -delta_encod[i];

      // speed_cm_s[i] = (float)((float)delta_encod[i] * cm_per_count) / (0.1);

      // // else if(speed_desired[i]==0)speed_cm_s[i]=0;
      // // Serial.println(speed_cm_s[i]);
      // speed_filter[i] = a_coeff * speed_filter[i] + b_coeff * speed_cm_s[i] + last_speed[i] * b_coeff;  // filter van toc
      float p_des = 5.5;
      if (i == M_RIGHT) p_des = 5.95;
      speed_cmd[i] = speed_desired[i] * p_des;
      control_motor(i, (int)speed_cmd[i]);
      last_speed[i] = speed_cm_s[i];
    }
    uint8_t header_encod = 'E';  // Byte nhận diện cho dữ liệu encoder
                                 // uint8_t delta_encod_bytes[4];

    // // Sao chép dữ liệu từ delta_encod vào delta_encod_bytes
    // for (int i = 0; i < 2; i++) {
    //   delta_encod_bytes[2 * i] = delta_encod[i] & 0xFF;
    //   delta_encod_bytes[2 * i + 1] = (delta_encod[i] >> 8) & 0xFF;
    // }
    float yaw_now=ypr[0]*180.0/M_PI;
    static float last_yaw = yaw_now;
    yaw_send = yaw_now - last_yaw;
    last_yaw = yaw_now;
    uint8_t delta_encod_bytes[4];

    // Gửi mảng byte qua WebSocket
    memcpy(delta_encod_bytes, &yaw_send, sizeof(yaw_send));

    // Gửi mảng byte qua WebSocket
    webSocket.broadcastBIN(delta_encod_bytes, sizeof(delta_encod_bytes));
    // if (ff)
    // {
    //   Serial.println(delta_encod[M_LEFT]);
    //   Serial.println(speed_cmd[M_LEFT]);
    // }
    static uint8_t count_ = 0;
  }
}