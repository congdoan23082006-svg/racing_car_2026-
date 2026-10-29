#include "pid.h"

// Biến lưu trữ cấu hình PID
static PID_Parameters pidParams = {
    .kp = 1.0f,
    .kd = 13.0f,
    .ki = 0.0f,
    .maxCorrection = 20000,
    .scaleDivisor = 35,
    .maxPwm = 255
};

// Biến trạng thái nội bộ của bộ điều khiển
static LineBiasMode currentBiasMode = LINE_BIAS_CENTER;
static long lastPos = 0;
static long lastError = 0;
static long integralSum = 0;
static long currentPosition = PID_CENTER_TARGET;
static int lastServoPwm = 0;
static bool isLineDetected = true;

// Biến trạng thái ghi nhớ hướng rẽ khi mất line (vitri từ code mẫu)
static int lineMemoryDirection = 0; 

// Biến trạng thái xử lý đoạn nét đứt (Dashed Line Gap Crossing)
static unsigned long lostLineStartTime = 0;
static bool isLostLineActive = false;
static unsigned long dashedGapTimeoutMs = 350;  // Cho phép vượt qua khoảng trống nét đứt trong 350ms
static unsigned long maxSearchTimeoutMs = 1500; // Thời gian quay tìm line tối đa trước khi dừng khẩn cấp
static bool isInDashedGap = false;

void pid_init(float kp, float kd, float ki, int maxPwm) {
    pidParams.kp = kp;
    pidParams.kd = kd;
    pidParams.ki = ki;
    pidParams.maxPwm = maxPwm;
    
    // Tự động căn chỉnh scaleDivisor theo chuẩn độ phân giải PWM
    // Nếu là 8-bit (TB6612, 0..255): scaleDivisor = 35 cho góc cua cực nhạy, bẻ lái dứt khoát và ôm sát vạch
    // Nếu là 12-bit (ESP32 ledc, 0..4095): scaleDivisor = 5 theo code gốc
    if (maxPwm <= 255) {
        pidParams.scaleDivisor = 35;
    } else {
        pidParams.scaleDivisor = 5;
    }
    
    pidParams.maxCorrection = maxPwm * pidParams.scaleDivisor;
    
    lastPos = 0;
    lastError = 0;
    integralSum = 0;
    currentPosition = PID_CENTER_TARGET;
    lastServoPwm = 0;
    lineMemoryDirection = 0;
    isLineDetected = true;
    lostLineStartTime = 0;
    isLostLineActive = false;
    isInDashedGap = false;
}

void pid_set_bias_mode(LineBiasMode mode) {
    currentBiasMode = mode;
}

void pid_set_tunings(float kp, float kd, float ki) {
    pidParams.kp = kp;
    pidParams.kd = kd;
    pidParams.ki = ki;
}

// ========================================================
// HÀM TÍNH TOÁN PID CHÍNH
// ========================================================
int pid_calculate(const unsigned int sensorPID[PID_NUM_SENSORS]) {
    long sum = 0;
    long avg = 0;
    bool hasLine = false;

    // 1. Kiểm tra xem có mắt nào thực sự thấy vạch đen hay không
    // Tránh bẫy toán học: Khi cả 8 mắt đều đọc nền trắng (ví dụ giá trị ~1),
    // avg / sum vẫn cho ra 10500 (tâm giả), làm xe tưởng đang ở giữa đường và lao thẳng!
    for (int j = 0; j < PID_NUM_SENSORS; j++) {
        if (sensorPID[j] > 350) { // Ngưỡng nhận diện có vạch đen
            hasLine = true;
        }
        avg += (long)sensorPID[j] * (j * PID_SENSOR_WEIGHT_GAP);
        sum += sensorPID[j];
    }

    // 2. KHẮC PHỤC LỖI NGUY HIỂM 1: Chia cho 0 hoặc mất line hoàn toàn
    if (sum == 0 || !hasLine) {
        isLineDetected = false;
        // Đóng băng khâu tích phân I tránh hiện tượng bão hòa (Anti-Windup)
        return lastServoPwm;
    }

    isLineDetected = true;

    // Vị trí tâm line thực tế (từ 0 đến 21000)
    currentPosition = avg / sum;

    // 3. Tính sai số theo chế độ bám (Bình thường / Lệch Trái / Lệch Phải)
    long target = PID_CENTER_TARGET;
    if (currentBiasMode == LINE_BIAS_LEFT) {
        target = PID_LEFT_BIAS_TARGET;
    } else if (currentBiasMode == LINE_BIAS_RIGHT) {
        target = PID_RIGHT_BIAS_TARGET;
    }

    long error = currentPosition - target;
    lastError = error;

    // 4. Khâu Tỉ lệ (Proportional)
    long iP = (long)(pidParams.kp * error);

    // 5. Khâu Vi phân (Derivative)
    long iD = (long)(pidParams.kd * (error - lastPos));

    // 6. Khâu Tích phân (Integral) kèm chống bão hòa (Anti-windup)
    integralSum += error;
    integralSum = constrain(integralSum, -30000, 30000);
    long iI = (long)(pidParams.ki * integralSum);

    // 7. Tổng hợp ngõ ra hiệu chỉnh
    long iRet = iP + iD + iI;

    // 8. KHẮC PHỤC LỖI NGUY HIỂM 2: Sửa đoạn "if (iRet < -12000) iRet = 0"
    // Giới hạn bão hòa đối xứng cả 2 bên [-maxCorrection .. +maxCorrection]
    iRet = constrain(iRet, -pidParams.maxCorrection, pidParams.maxCorrection);

    // 9. Chia tỷ lệ ra xung điều khiển động cơ
    int servoPwm = (int)(iRet / pidParams.scaleDivisor);

    // Giới hạn trong phạm vi motor
    servoPwm = constrain(servoPwm, -pidParams.maxPwm, pidParams.maxPwm);

    lastPos = error;
    lastServoPwm = servoPwm;

    return servoPwm;
}

// ========================================================
// HÀM TRỘN TỐC ĐỘ 2 BÁNH XE (DIFFERENTIAL SPEED MIXER)
// ========================================================
void pid_mix_motor_speeds(int baseSpeed, int angle, int *outSpeedLeft, int *outSpeedRight) {
    if (!outSpeedLeft || !outSpeedRight) return;

    int maxPwm = pidParams.maxPwm; // 255 đối với TB6612

    int left = baseSpeed + angle;
    int right = baseSpeed - angle;

    // Duy trì lực kéo tối đa (High-Torque Differential Drive):
    // Nếu một bên vượt quá maxPwm (255), giữ bên đó ở 255 và trừ phần dư vào bên đối diện.
    // Điều này giúp xe không bao giờ bị sụt giảm tổng lực kéo khi đang bám cua!
    if (left > maxPwm) {
        int excess = left - maxPwm;
        left = maxPwm;
        right -= excess;
    } else if (right > maxPwm) {
        int excess = right - maxPwm;
        right = maxPwm;
        left -= excess;
    }

    *outSpeedLeft = constrain(left, -maxPwm, maxPwm);
    *outSpeedRight = constrain(right, -maxPwm, maxPwm);
}

// ========================================================
// BỘ ĐIỀU KHIỂN LAI: KẾT HỢP PID VÀ MÁY TRẠNG THÁI GÓC CUA GẮT
// ========================================================
void pid_run_hybrid_controller(unsigned char sensorPattern,
                               const unsigned int sensorPID[PID_NUM_SENSORS],
                               int baseSpeed,
                               int curveSpeed,
                               int *outSpeedLeft,
                               int *outSpeedRight) {
    if (!outSpeedLeft || !outSpeedRight) return;

    // Bước 1: Tính góc lái PID
    int servoPwm = pid_calculate(sensorPID);

    // Nếu đang bắt được vạch (không phải 0b00000000) -> Reset bộ đếm khoảng đứt
    if (sensorPattern != 0b00000000) {
        isLostLineActive = false;
        lostLineStartTime = 0;
        isInDashedGap = false;
    }

    // Bước 2: Phân loại trạng thái theo cấu hình bitmask
    switch (sensorPattern) {
        // === VẠCH Ở CHÍNH GIỮA: Chạy thẳng tốc độ tối đa ===
        case 0b00011000:
        case 0b00111100:
            lineMemoryDirection = 0;
            pid_mix_motor_speeds(baseSpeed, servoPwm, outSpeedLeft, outSpeedRight);
            break;

        case 0b00011100:
        case 0b00001000:
            lineMemoryDirection = 1;
            pid_mix_motor_speeds(baseSpeed, servoPwm, outSpeedLeft, outSpeedRight);
            break;

        case 0b00010000:
        case 0b00111000:
            lineMemoryDirection = -1;
            pid_mix_motor_speeds(baseSpeed, servoPwm, outSpeedLeft, outSpeedRight);
            break;

        // === VẠCH NGANG NGÃ TƯ (CROSSROAD): 6-8 mắt đều nhận vạch đen ===
        // Chuẩn thi đấu quốc tế: Khóa góc lái, giữ thẳng băng lướt qua ngã tư, không bị bẻ lái bậy
        case 0b11111111:
        case 0b01111110:
        case 0b11111110:
        case 0b01111111:
        case 0b11111100:
        case 0b00111111:
            lineMemoryDirection = 0;
            pid_mix_motor_speeds(baseSpeed, 0, outSpeedLeft, outSpeedRight);
            break;

        // === CUA NHẸ: Giữ ga 92% baseSpeed để xe luôn có đà bám cua, không bị ì ===
        case 0b00001100:
        case 0b00011110:
            lineMemoryDirection = 2;
            pid_mix_motor_speeds(baseSpeed * 92 / 100, servoPwm, outSpeedLeft, outSpeedRight);
            break;

        case 0b00110000:
        case 0b01111000:
            lineMemoryDirection = -2;
            pid_mix_motor_speeds(baseSpeed * 92 / 100, servoPwm, outSpeedLeft, outSpeedRight);
            break;

        // === CUA VỪA: Giữ ga 85% baseSpeed, lực kéo đủ khỏe vượt ma sát bánh ===
        case 0b00000100:
        case 0b00001110:
            lineMemoryDirection = 3;
            pid_mix_motor_speeds(baseSpeed * 85 / 100, servoPwm, outSpeedLeft, outSpeedRight);
            break;

        case 0b00100000:
        case 0b01110000:
            lineMemoryDirection = -3;
            pid_mix_motor_speeds(baseSpeed * 85 / 100, servoPwm, outSpeedLeft, outSpeedRight);
            break;

        // === CUA GẮT 90 ĐỘ BÊN PHẢI (Mắt mép 5, 6, 7 nhận vạch) ===
        // Bánh ngoài kéo mạnh (Forward), bánh trong đảo chiều quay lùi (Counter-Brake) để xe ôm gọn góc vuông 90°
        case 0b00000110:
        case 0b00001111:
        case 0b00000010:
        case 0b00000111:
        case 0b00000011:
        case 0b00000001:
            lineMemoryDirection = 7;
            {
                int turnForward = constrain(baseSpeed + 35, 160, 255);
                int turnCounter = constrain(baseSpeed * 70 / 100, 80, 150);
                *outSpeedLeft = turnForward;
                *outSpeedRight = -turnCounter;
            }
            break;

        // === CUA GẮT 90 ĐỘ BÊN TRÁI (Mắt mép 0, 1, 2 nhận vạch) ===
        case 0b01100000:
        case 0b11110000:
        case 0b01000000:
        case 0b11100000:
        case 0b11000000:
        case 0b10000000:
            lineMemoryDirection = -7;
            {
                int turnForward = constrain(baseSpeed + 35, 160, 255);
                int turnCounter = constrain(baseSpeed * 70 / 100, 80, 150);
                *outSpeedLeft = -turnCounter;
                *outSpeedRight = turnForward;
            }
            break;

        // === MẤT VẠCH / ĐOẠN NÉT ĐỨT (GAP HANDLING) ===
        case 0b00000000:
            if (!isLostLineActive) {
                isLostLineActive = true;
                lostLineStartTime = millis();
            }
            {
                unsigned long lostDuration = millis() - lostLineStartTime;

                // TRƯỜNG HỢP 1: TRƯỚC ĐÓ XE ĐANG GẦN TÂM (|lineMemoryDirection| <= 1)
                // -> ĐÂY LÀ VẠCH NÉT ĐỨT: Duy trì tốc độ thẳng để lướt qua khoảng ngắt quãng!
                if (abs(lineMemoryDirection) <= 1) {
                    if (lostDuration <= dashedGapTimeoutMs) {
                        isInDashedGap = true;
                        *outSpeedLeft = baseSpeed;
                        *outSpeedRight = baseSpeed;
                    } else if (lostDuration <= maxSearchTimeoutMs) {
                        isInDashedGap = false;
                        int searchSpeed = constrain(baseSpeed * 70 / 100, 110, 180);
                        if (lineMemoryDirection < 0) {
                            *outSpeedLeft = -searchSpeed / 2;
                            *outSpeedRight = searchSpeed;
                        } else {
                            *outSpeedLeft = searchSpeed;
                            *outSpeedRight = -searchSpeed / 2;
                        }
                    } else {
                        isInDashedGap = false;
                        *outSpeedLeft = 0;
                        *outSpeedRight = 0;
                    }
                }
                // TRƯỜNG HỢP 2: TRƯỚC ĐÓ XE ĐANG Ở MÉP BÊN NGOÀI (|lineMemoryDirection| >= 2)
                // -> ĐÂY LÀ GÓC CUA 90 ĐỘ HOẶC CUA GẮT BỊ VĂNG KHỎI VẠCH!
                // TUYỆT ĐỐI KHÔNG ĐƯỢC CHẠY THẲNG! PHẢI BẺ LÁI GẤP THEO HƯỚNG GHI NHỚ ĐỂ BẮT LẠI VẠCH!
                else {
                    isInDashedGap = false;
                    int pivotForward = constrain(baseSpeed + 35, 160, 255);
                    int pivotReverse = constrain(baseSpeed * 70 / 100, 80, 150);

                    if (lostDuration <= maxSearchTimeoutMs) {
                        if (lineMemoryDirection > 0) {
                            // Trước đó vạch ở bên PHẢI -> Ép quay gấp sang PHẢI
                            *outSpeedLeft = pivotForward;
                            *outSpeedRight = -pivotReverse;
                        } else {
                            // Trước đó vạch ở bên TRÁI -> Ép quay gấp sang TRÁI
                            *outSpeedLeft = -pivotReverse;
                            *outSpeedRight = pivotForward;
                        }
                    } else {
                        *outSpeedLeft = 0;
                        *outSpeedRight = 0;
                    }
                }
            }
            break;

        default:
            lineMemoryDirection = (servoPwm > 0) ? 2 : (servoPwm < 0 ? -2 : 0);
            pid_mix_motor_speeds(baseSpeed * 88 / 100, servoPwm, outSpeedLeft, outSpeedRight);
            break;
    }
}

long pid_get_current_position(void) {
    return currentPosition;
}

long pid_get_last_error(void) {
    return lastError;
}

bool pid_is_line_detected(void) {
    return isLineDetected;
}

void pid_set_dashed_gap_timeout(unsigned long gapTimeoutMs) {
    dashedGapTimeoutMs = gapTimeoutMs;
}

bool pid_is_in_dashed_gap(void) {
    return isInDashedGap;
}
