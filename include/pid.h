#ifndef PID_H
#define PID_H

#include <Arduino.h>

// =========================================================================
// THAM SỐ PID (điều chỉnh qua BLE: PID:kp,kd,base)
// =========================================================================
extern float KP;       // Hệ số tỉ lệ P
extern float KD;       // Hệ số vi phân D
extern int   BASE;     // Tốc độ cơ sở (0..255)

// Cua 90 độ: bánh ngoài = BASE + TURN_OUT, bánh trong = BASE - TURN_IN
extern int TURN_OUT;
extern int TURN_IN;

// Trạng thái chạy line
extern bool runLine;
extern int  lastError;

// Bộ đếm nét đứt & vạch kết thúc
extern int  cnt;
extern int  cntDen;
extern bool inGap;
extern bool inFullBlack;
extern int  fullBlackFrames;
extern unsigned long gapStart;

// =========================================================================
// CẤU HÌNH LOGIC LINE FOLLOW
// =========================================================================
#define GAP_STRAIGHT_MS 100   // ms chạy thẳng qua nét đứt
#define GAP_MAX_COUNT     5   // số lần mất line được phép
#define GAP_ERR_MAX    1500   // |error| tối đa để coi là mất line ngắn
#define END_BLACK_COUNT   1   // số lần thấy 0xFF để xác nhận vạch đích
#define END_CONFIRM       3   // số frame liên tiếp 0xFF để trigger endLine

// =========================================================================
// API
// =========================================================================
void pid_init();          // Khởi tạo giá trị mặc định
void lineFollowPID();     // Gọi trong loop() khi runLine = true
void endLine();           // Phanh + dừng khi cán vạch đích

#endif // PID_H
