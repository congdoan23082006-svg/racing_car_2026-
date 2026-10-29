#ifndef TB6612_H
#define TB6612_H

#include <Arduino.h>

// =========================================================================
// CHÂN TB6612 - IC TRÁI (AIN1 nối BIN1, AIN2 nối BIN2, PWMA nối PWMB)
// =========================================================================
#define L_PWM   25
#define L_IN1   26
#define L_IN2   27

// =========================================================================
// CHÂN TB6612 - IC PHẢI
// =========================================================================
#define R_PWM   14
#define R_IN1   12   // GPIO12 là chân strapping: không kéo HIGH lúc boot
#define R_IN2   13

// Chân STBY chung (nối cứng 3.3V → để -1)
#define TB6612_STBY -1

// Cấu hình PWM
#define PWM_FREQ    2000
#define PWM_RES     8        // 8-bit: 0..255
#define L_PWM_CH    0
#define R_PWM_CH    1

// Đặt 1 nếu bánh quay ngược chiều mong muốn
#define REVERSE_LEFT   0
#define REVERSE_RIGHT  0

// =========================================================================
// API
// =========================================================================
void tb6612_init();
void tb6612_coast();        // Thả trôi tự do
void tb6612_short_brake();  // Phanh ghìm cứng (dừng khẩn cấp)
void tb6612_standby(bool enable);

// speed_run: left/right thang -255..255 (dương=tiến, âm=lùi)
void speed_run(int left, int right);

// Hàm tiện ích cấp cao
void move(int left, int right);
void stopMotor();
void movePID(int base, int adjust);

// Tham số dùng chung
extern int speed_dk;  // tốc độ manual
extern int BASE;      // base speed PID

#endif // TB6612_H