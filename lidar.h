#include <HardwareSerial.h>

HardwareSerial LiDARSerial(1);  // Sử dụng UART1 trên ESP32
const int RX_PIN = 16;          // RX của ESP32 (kết nối với TX của LDS-007)
const int TX_PIN = 17;          // TX của ESP32 (kết nối với RX của LDS-007)
const long BAUD_RATE = 115200;  // Tốc độ baud cho LDS-007

// Cấu hình các biến cho dữ liệu
float rpms = 0;  // Vòng quay trên phút
const float range_max_now = 5.5;
const float range_min_now = 0.05;
void setupLidar()
{
    LiDARSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // Gửi lệnh để khởi động LDS-007
    LiDARSerial.write('$');
    LiDARSerial.print("startlds$");
}
void loopLidar()
{
      uint8_t raw_bytes[1980];
  int start_count = 0;
  bool got_scan = false;

  while (!got_scan) {
    unsigned long time_=millis();
    // Đọc một byte dữ liệu và kiểm tra trình tự bắt đầu của gói dữ liệu
    if (LiDARSerial.available() > 0) {
      uint8_t temp_byte = LiDARSerial.read();

      // Tìm kiếm trình tự bắt đầu
      if (start_count == 0 && temp_byte == 0xFA) {
        start_count = 1;
      } else if (start_count == 1 && temp_byte == 0xA0) {
        start_count = 0;
        got_scan = true;

        // Đọc toàn bộ gói tin còn lại
        LiDARSerial.readBytes(&raw_bytes[2], 1978);

        float ranges[360];
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
              float range_float = range / 1000.0;
              if (range_float >= range_max_now || range_float <= range_min_now || intensity <= 5) {
                range_float = 0;
              }

              ranges[index] = range_float;
              intensities[index] = intensity;
            }
          }
        }

        // Serial.print("RPM: ");
        // Serial.println(rpms);
        // Serial.print("First distance: ");
        // Serial.println(ranges[200], 5);
        // Serial.print("First intensity: ");
        // Serial.println(intensities[200]);
      }
    }
    Serial.println(millis()-time_);
  }
}