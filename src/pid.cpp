#include "pid.h"
#include "sensor.h"     // docSensor(), currentPattern, lastLinePos
#include "TB6612.h"     // move(), movePID(), stopMotor(), tb6612_short_brake(), tb6612_coast()

// =========================================================================
// ĐỊNH NGHĨA BIẾN TOÀN CỤC PID
// =========================================================================
float KP      = 0.04f;
float KD      = 0.3f;
int   BASE    = 150;
int   TURN_OUT = 70;
int   TURN_IN  = 90;

bool runLine = false;
int  lastError = 0;

int  cnt           = 0;
int  cntDen        = 0;
bool inGap         = false;
bool inFullBlack   = false;
int  fullBlackFrames = 0;
unsigned long gapStart = 0;

// =========================================================================
// KHỞI TẠO GIÁ TRỊ MẶC ĐỊNH
// =========================================================================
void pid_init() {
    KP = 0.04f; KD = 0.3f; BASE = 150;
    TURN_OUT = 70; TURN_IN = 90;
    runLine = false; lastError = 0;
    cnt = 0; cntDen = 0;
    inGap = false; inFullBlack = false;
    fullBlackFrames = 0;
}

// =========================================================================
// KẾT THÚC LINE: PHANH GHÌM + THẢ TRÔI
// =========================================================================
void endLine() {
    tb6612_short_brake();
    delay(150);
    tb6612_coast();
    runLine = false;
    // currentModeName / currentCaseName được cập nhật trong main.cpp
}

// =========================================================================
// VÒNG LẶP DÒ LINE PID - GỌI LIÊN TỤC TRONG loop()
// =========================================================================
void lineFollowPID() {
    int blackCount = docSensor();

    // --- Kiểm tra vạch đích (0b11111111) ---
    if (currentPattern == 0b11111111) {
        fullBlackFrames++;
        if (fullBlackFrames >= END_CONFIRM && !inFullBlack) {
            inFullBlack = true;
            cntDen++;
            if (cntDen >= END_BLACK_COUNT) {
                endLine();
                return;
            }
        }
    } else {
        fullBlackFrames = 0;
        inFullBlack = false;
    }

    if (blackCount != 0) inGap = false;

    // --- Cua 90 độ PHẢI (mắt ngoài cùng phải sáng, blackCount < 5) ---
    if ((currentPattern & 0b00000001) && blackCount < 5) {
        move(BASE + TURN_OUT, BASE - TURN_IN);
        return;
    }

    // --- Cua 90 độ TRÁI (mắt ngoài cùng trái sáng, blackCount < 5) ---
    if ((currentPattern & 0b10000000) && blackCount < 5) {
        move(BASE - TURN_IN, BASE + TURN_OUT);
        return;
    }

    // --- Mất line hoàn toàn ---
    if (blackCount == 0) {
        if (!inGap && cnt < GAP_MAX_COUNT && abs(lastError) <= GAP_ERR_MAX) {
            inGap = true;
            gapStart = millis();
            cnt++;
        }
        // Nét đứt ngắn: chạy thẳng qua khoảng trống
        if (inGap && (millis() - gapStart < GAP_STRAIGHT_MS)) {
            move(BASE, BASE);
            return;
        }
        // Mất line thật: quay về phía đã thấy line lần cuối
        if (lastError > 0) move(BASE + 20, -BASE);
        else               move(-BASE, BASE + 20);
        return;
    }

    // --- PID BÌNH THƯỜNG ---
    int error  = (int)lastLinePos - SETPOINT;
    int adjust = (int)(error * KP) + (int)(KD * (error - lastError));
    lastError  = error;
    movePID(BASE, adjust);
}
