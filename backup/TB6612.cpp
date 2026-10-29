#include "TB6612.h"

// Hàm phụ trợ xuất xung PWM tương thích cả ESP32 Core 2.x và 3.x
static inline void writePwmLeft(uint32_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(L_PWM, duty);
#else
  ledcWrite(L_PWM_CH, duty);
#endif
}

static inline void writePwmRight(uint32_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(R_PWM, duty);
#else
  ledcWrite(R_PWM_CH, duty);
#endif
}

void tb6612_init() {
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  if (TB6612_STBY >= 0) {
    pinMode(TB6612_STBY, OUTPUT);
    digitalWrite(TB6612_STBY, HIGH); // Bật IC hoạt động
  }

  // Đưa 2 cầu H về trạng thái nghỉ (Coast)
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);

// Khởi tạo PWM ở tần số 2000Hz (Giảm tổn hao đóng cắt sinh nhiệt)
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(L_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_PWM, PWM_FREQ, PWM_RES);
#else
  ledcSetup(L_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_PWM, L_PWM_CH);
  ledcSetup(R_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_PWM, R_PWM_CH);
#endif

  speed_run(0, 0);
}

// -------------------------------------------------------------------------
// HÀM ĐIỀU KHIỂN TỐC ĐỘ & CHIỀU QUAY CHO 2 IC TB6612 SONG SONG
// -------------------------------------------------------------------------
void speed_run(int speedDC_left, int speedDC_right) {
  // Giới hạn dải PWM an toàn [-255 .. 255]
  speedDC_left = constrain(speedDC_left, -255, 255);
  speedDC_right = constrain(speedDC_right, -255, 255);

  // 1. Điều khiển IC TB6612 số 1 (Động cơ TRÁI)
  if (speedDC_left > 0) {
    // Quay tiến
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    writePwmLeft(speedDC_left);
  } else if (speedDC_left < 0) {
    // Quay lùi (hãm cua)
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    writePwmLeft(-speedDC_left);
  } else {
    // Bằng 0 -> Chế độ Coast (IN1=LOW, IN2=LOW) để MOSFET nghỉ, không sinh nhiệt
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    writePwmLeft(0);
  }

  // 2. Điều khiển IC TB6612 số 2 (Động cơ PHẢI)
  if (speedDC_right > 0) {
    // Quay tiến
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    writePwmRight(speedDC_right);
  } else if (speedDC_right < 0) {
    // Quay lùi (hãm cua)
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    writePwmRight(-speedDC_right);
  } else {
    // Bằng 0 -> Chế độ Coast (IN1=LOW, IN2=LOW)
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    writePwmRight(0);
  }
}

// Phanh điện tử ghìm cứng trục động cơ (Short Brake: IN1 = HIGH, IN2 = HIGH)
void tb6612_short_brake() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, HIGH);
  writePwmLeft(255);

  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, HIGH);
  writePwmRight(255);
}

// Thả trôi tự do 2 bánh
void tb6612_coast() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  writePwmLeft(0);

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  writePwmRight(0);
}

// Bật/tắt chế độ ngủ tiết kiệm điện của TB6612
void tb6612_standby(bool enable) {
  if (TB6612_STBY >= 0) {
    digitalWrite(TB6612_STBY, enable ? LOW : HIGH);
  }
}
