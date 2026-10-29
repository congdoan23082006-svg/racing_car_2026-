#include "sensor.h"
#include <EEPROM.h>

volatile uint8_t sensor = 0;
unsigned int sensorValue[NUM_SENSORS];
unsigned int sensorPID[NUM_SENSORS];
unsigned int black_value[NUM_SENSORS];
unsigned int white_value[NUM_SENSORS];
unsigned int compare_value[NUM_SENSORS];
volatile uint8_t isCalib = 0;

// Hàm chọn kênh 0 -> 7 trên IC 74HC4067 (8 mắt)
static inline void selectMuxChannel(uint8_t channel) {
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
}

// Hàm đọc thô giá trị ADC của 1 mắt cụ thể
static inline unsigned int readRawAdcChannel(uint8_t channel) {
  selectMuxChannel(channel);
  delayMicroseconds(3); // Đợi kênh Mux ổn định điện áp
  return 4095 - analogRead(MUX_SIG);
}

void sensor_init() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);

  EEPROM.begin(EEPROM_SIZE);
  readSensorFromEEPROM();
}

void read_sensor() {
  uint8_t temp = 0;

  for (int j = 0; j < NUM_SENSORS; j++) {
    // 1. Đọc giá trị thực tế từ kênh thứ j của 74HC4067
    sensorValue[j] = readRawAdcChannel(j);

    // 2. Nếu đang bật chế độ Calib: Tự động bắt Min/Max và tính ngưỡng riêng cho từng mắt
    if (isCalib == 1) {
      if (sensorValue[j] < black_value[j]) black_value[j] = sensorValue[j];
      if (sensorValue[j] > white_value[j]) white_value[j] = sensorValue[j];
      compare_value[j] = (black_value[j] + white_value[j]) / 2;
    } 
    // 3. Nếu đang chạy bình thường: Quy đổi ra thang 1..1000 theo ngưỡng riêng của từng mắt
    else {
      unsigned int clampedVal = sensorValue[j];
      if (clampedVal < black_value[j]) clampedVal = black_value[j];
      if (clampedVal > white_value[j]) clampedVal = white_value[j];

      long lowBound = (long)black_value[j] * 95 / 100;
      long span = (long)white_value[j] - lowBound;
      if (span <= 0) span = 1;

      sensorPID[j] = map(clampedVal, lowBound, white_value[j], 1, 1000);
    }

    // 4. So sánh với ngưỡng riêng compare_value[j] của chính mắt đó
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01; // Đọc thấy vạch đen (1)
    } else {
      temp &= 0xFE; // Đọc thấy nền trắng (0)
    }
  }

  sensor = temp;
}

// Bắt đầu quá trình Calib: Lấy mẫu thực tế đầu tiên của từng mắt làm mốc gốc
void startCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    unsigned int currentVal = readRawAdcChannel(i);
    black_value[i] = currentVal;
    white_value[i] = currentVal;
    compare_value[i] = currentVal;
  }
  isCalib = 1;
}

// Cập nhật ngưỡng từng mắt trong khi lia giàn mắt qua vạch đen và nền trắng
void updateLineCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];

    // Tính ngưỡng trung bình riêng biệt cho từng mắt cảm biến
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
}

// Kết thúc Calib và lưu toàn bộ ngưỡng của 8 mắt vào EEPROM
void finishCalibration() {
  isCalib = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
  saveSensorToEEPROM();
}

void saveSensorToEEPROM() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.write(i * 2, (black_value[i] >> 8) & 0xFF);
    EEPROM.write(i * 2 + 1, black_value[i] & 0xFF);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.write(16 + (i * 2), (white_value[i] >> 8) & 0xFF);
    EEPROM.write(16 + (i * 2) + 1, white_value[i] & 0xFF);
  }
  EEPROM.commit();
}

void readSensorFromEEPROM() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    black_value[i] = (EEPROM.read(i * 2) << 8) | EEPROM.read(i * 2 + 1);
    white_value[i] = (EEPROM.read(16 + (i * 2)) << 8) | EEPROM.read(16 + (i * 2) + 1);

    // Tính lại compare_value[i] từ dữ liệu calib thực tế của từng mắt trong EEPROM
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
}
