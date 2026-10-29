#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <EEPROM.h>

// =========================================================================
// MUX CD74HC4067: S0=IO18, S1=IO19, S2=IO23, S3=GND, EN=GND, SIG=IO32
// L1..L8 nối vào kênh C0..C7
// =========================================================================
#define MUX_S0  18
#define MUX_S1  19
#define MUX_S2  23
#define MUX_SIG 32

#define NUM_SENSORS 8

// =========================================================================
// CẤU HÌNH ĐỌC CẢM BIẾN
// =========================================================================
// 1 = dùng thứ tự Gray code để giảm nhiễu khi chuyển kênh mux
#define QTR_USE_GRAY  1
#if QTR_USE_GRAY
  #define QTR_ORDER {0, 1, 3, 2, 6, 7, 5, 4}
#else
  #define QTR_ORDER {0, 1, 2, 3, 4, 5, 6, 7}
#endif

#define QTR_SETTLE_US  300   // µs chờ sau khi đổi kênh mux
#define QTR_DUMMY       3   // số lần đọc bỏ (xả tụ ký sinh)
#define QTR_SAMPLES     4   // số lần đọc lấy trung bình
#define QTR_MIN_RANGE  200  // khoảng min-max tối thiểu để coi kênh đã calib tốt

#define QTR_EEPROM_SIZE   64
#define QTR_EEPROM_ADDR    0
#define QTR_EEPROM_MAGIC  0x51A8

// Đặt 1 nếu cảm biến đọc NGƯỢC trái/phải
#define REVERSE_SENSOR 0
// true = đảo màu (line đen trên nền trắng, đọc ADC cao = trắng)
#define INVERT_COLOR   true

// Ngưỡng "thấy line" (thang 0..1000)
#define SENSOR_THRESHOLD 500

// Setpoint trung tâm (thang 0..7000, tâm = 3500)
#define SETPOINT 3500

// =========================================================================
// CLASS MyQTR8C - ĐỌC 8 CẢM BIẾN QUA MUX CD74HC4067
// =========================================================================
class MyQTR8C {
public:
    MyQTR8C(const uint8_t* selPins, uint8_t sigPin, bool invert = false);

    void begin();
    void resetCalibration();
    void calibrate();
    bool channelOk(uint8_t ch) const;
    void saveCalibration();
    bool loadCalibration();

    // Trả về vị trí tâm line 0..7000 (setpoint = 3500)
    // values[i] = 0..1000 (1000 = thấy line đậm nhất)
    uint16_t readLine(uint16_t* values);

    uint16_t raw(uint8_t ch);

private:
    uint8_t  _sel[3], _sig;
    bool     _invert;
    uint16_t _min[8], _max[8];
    uint16_t _last;
    uint8_t  _cur;
    const uint8_t GRAY[8] = QTR_ORDER;

    void     selectChannel(uint8_t ch);
    uint16_t readRaw(uint8_t ch);
    uint16_t normalize(uint8_t i, uint16_t raw);
};

// =========================================================================
// BIẾN TOÀN CỤC (dùng trong pid.cpp / main.cpp)
// =========================================================================
extern MyQTR8C       qtr;
extern uint16_t      sensorValues[NUM_SENSORS];
extern uint8_t       currentPattern;
extern unsigned int  lastLinePos;

extern bool calibrated;
extern bool calibrating;
extern unsigned long calEnd;
#define CAL_TIME 5000

// =========================================================================
// HÀM TIỆN ÍCH
// =========================================================================
// Đọc toàn bộ 8 mắt, cập nhật currentPattern và lastLinePos
// Trả về số mắt thấy line
int docSensor();

#endif // SENSOR_H