#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid = "ESP32_test";          // Thay "your_SSID" bằng tên Wi-Fi của bạn
const char* password = "123456789";  // Thay "your_PASSWORD" bằng mật khẩu Wi-Fi của bạn

WebSocketsServer webSocket(80);  // Cổng WebSocket Server

// Khai báo các biến điều khiển động cơ hoặc lidar
int speed_left = 0;
int speed_right = 0;

// Hàm khởi tạo WebSocket Server và kết nối WiFi
void setup() {
  Serial.begin(115200);
 

  // Chờ kết nối Wi-Fi
   WiFi.softAP(ssid,password);



  IPAddress IP = WiFi.softAPIP();

  Serial.print("AP IP address: ");

  Serial.println(IP);

  Serial.println("Connected to WiFi");

  // Khởi động WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// Hàm xử lý các sự kiện của WebSocket
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_BIN) {
    // Kiểm tra kích thước dữ liệu
    if (length == 4) {  // 2 int16_t, mỗi int16_t là 2 byte
      speed_left = (int16_t)(payload[0] | (payload[1] << 8));
      speed_right = (int16_t)(payload[2] | (payload[3] << 8));
      Serial.printf("Received speed_left: %d, speed_right: %d\n", speed_left, speed_right);
    } else {
      Serial.println("Invalid binary payload length");
    }
  } else if (type == WStype_TEXT) {
    Serial.printf("Text message received: %s\n", payload);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("Client [%u] disconnected!\n", num);
  } else if (type == WStype_CONNECTED) {
    Serial.printf("Client [%u] connected!\n", num);
  }
}

// Task gửi dữ liệu lidar qua WebSocket
void taskLidar() {
  static unsigned long time_ = millis();
  if (millis() - time_ > 200) {
    time_ = millis();
    // Giả lập dữ liệu lidar với khoảng cách ngẫu nhiên
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
    webSocket.broadcastBIN(data, sizeof(data));\
    static uint8_t count_=0;
    count_++;
    if(count_>5)
     {
      Serial.println("Lidar data sent to client");
      count_=0;

     }     
  }

}

// Task nhận và xử lý lệnh điều khiển tốc độ từ ROS


void loop() {
  // Tạo hai task song song
  taskLidar();
  webSocket.loop();
}
