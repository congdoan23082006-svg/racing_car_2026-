#include "sensor.h"

// =========================================================================
// ĐỊNH NGHĨA BIẾN TOÀN CỤC
// =========================================================================
const uint8_t S_PINS[] = {MUX_S0, MUX_S1, MUX_S2};

MyQTR8C      qtr(S_PINS, MUX_SIG, INVERT_COLOR);
uint16_t     sensorValues[NUM_SENSORS];
uint8_t      currentPattern  = 0;
unsigned int lastLinePos      = SETPOINT;

bool         calibrated  = false;
bool         calibrating = false;
unsigned long calEnd     = 0;

// =========================================================================
// MyQTR8C: CONSTRUCTOR
// =========================================================================
MyQTR8C::MyQTR8C(const uint8_t* selPins, uint8_t sigPin, bool invert)
    : _sig(sigPin), _invert(invert), _last(3500), _cur(0) {
    for (int i = 0; i < 3; i++) _sel[i] = selPins[i];
    resetCalibration();
}

// =========================================================================
// BEGIN: KHỞI TẠO CHÂN VÀ ADC
// =========================================================================
void MyQTR8C::begin() {
    for (int i = 0; i < 3; i++) {
        pinMode(_sel[i], OUTPUT);
        digitalWrite(_sel[i], LOW);
    }
    pinMode(_sig, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(_sig, ADC_11db);
    _cur = 0;
    EEPROM.begin(QTR_EEPROM_SIZE);
}

// =========================================================================
// RESET CALIBRATION
// =========================================================================
void MyQTR8C::resetCalibration() {
    for (int i = 0; i < 8; i++) { _min[i] = 4095; _max[i] = 0; }
}

// =========================================================================
// CALIBRATE: GỌI LIÊN TỤC TRONG VÒNG LẶP KHI LIA QUA VẠCH
// =========================================================================
void MyQTR8C::calibrate() {
    for (int k = 0; k < 8; k++) {
        uint8_t i = GRAY[k];
        uint16_t v = readRaw(i);
        if (v < _min[i]) _min[i] = v;
        if (v > _max[i]) _max[i] = v;
    }
}

// =========================================================================
// KIỂM TRA KÊNH ĐÃ CALIB HỢP LỆ
// =========================================================================
bool MyQTR8C::channelOk(uint8_t ch) const {
    return (_max[ch] > _min[ch]) && ((_max[ch] - _min[ch]) >= QTR_MIN_RANGE);
}

// =========================================================================
// LƯU CALIB VÀO EEPROM
// =========================================================================
void MyQTR8C::saveCalibration() {
    int addr = QTR_EEPROM_ADDR;
    EEPROM.put(addr, (uint16_t)QTR_EEPROM_MAGIC); addr += sizeof(uint16_t);
    for (int i = 0; i < 8; i++) { EEPROM.put(addr, _min[i]); addr += sizeof(uint16_t); }
    for (int i = 0; i < 8; i++) { EEPROM.put(addr, _max[i]); addr += sizeof(uint16_t); }
    EEPROM.commit();
}

// =========================================================================
// NẠP CALIB TỪ EEPROM
// =========================================================================
bool MyQTR8C::loadCalibration() {
    int addr = QTR_EEPROM_ADDR;
    uint16_t magic = 0;
    EEPROM.get(addr, magic); addr += sizeof(uint16_t);
    if (magic != QTR_EEPROM_MAGIC) return false;
    uint16_t mn[8], mx[8];
    for (int i = 0; i < 8; i++) { EEPROM.get(addr, mn[i]); addr += sizeof(uint16_t); }
    for (int i = 0; i < 8; i++) { EEPROM.get(addr, mx[i]); addr += sizeof(uint16_t); }
    for (int i = 0; i < 8; i++) { _min[i] = mn[i]; _max[i] = mx[i]; }
    return true;
}

// =========================================================================
// ĐỌC VỊ TRÍ TÂMLINE (0..7000, setpoint = 3500)
// values[i] = 0..1000 (1000 = thấy line)
// =========================================================================
uint16_t MyQTR8C::readLine(uint16_t* values) {
    for (int k = 0; k < 8; k++) {
        uint8_t i = GRAY[k];
        values[i] = normalize(i, readRaw(i));
    }
    long sum = 0, weighted = 0;
    bool seen = false;
    for (int i = 0; i < 8; i++) {
        uint16_t v = values[i];
        if (v > 200) seen = true;
        if (v > 50)  { weighted += (long)v * i * 1000; sum += v; }
    }
    if (!seen || sum == 0) {
        // Nhớ lệch về phía cuối cùng thấy line
        if (_last < 2000)      _last = 0;
        else if (_last > 5000) _last = 7000;
    } else {
        _last = weighted / sum;
    }
    return _last;
}

// =========================================================================
// ĐỌC GIÁ TRỊ RAW 1 KÊNH (KHÔNG NORMALIZE)
// =========================================================================
uint16_t MyQTR8C::raw(uint8_t ch) { return readRaw(ch); }

// =========================================================================
// PRIVATE: CHỌN KÊNH MUX (GRAY CODE ĐỂ GIẢM NHIỄU)
// =========================================================================
void MyQTR8C::selectChannel(uint8_t ch) {
    uint8_t diff = ch ^ _cur;
    for (int b = 0; b < 3; b++)
        if (diff & (1 << b)) digitalWrite(_sel[b], (ch >> b) & 1);
    _cur = ch;
}

// =========================================================================
// PRIVATE: ĐỌC ADC VỚI DUMMY READS VÀ OVERSAMPLING
// =========================================================================
uint16_t MyQTR8C::readRaw(uint8_t ch) {
    selectChannel(ch);
    delayMicroseconds(QTR_SETTLE_US);
    for (int d = 0; d < QTR_DUMMY; d++) analogRead(_sig);
    uint32_t acc = 0;
    for (int k = 0; k < QTR_SAMPLES; k++) acc += analogRead(_sig);
    return acc / QTR_SAMPLES;
}

// =========================================================================
// PRIVATE: NORMALIZE VỀ THANG 0..1000
// =========================================================================
uint16_t MyQTR8C::normalize(uint8_t i, uint16_t rawVal) {
    if (!channelOk(i)) return 0;
    uint16_t range = _max[i] - _min[i];
    uint16_t subVal = (rawVal < _min[i]) ? rawVal : _min[i];
    long v = ((long)(rawVal - subVal) * 1000) / range;
    v = constrain(v, 0, 1000);
    return _invert ? (uint16_t)(1000 - v) : (uint16_t)v;
}

// =========================================================================
// docSensor: ĐỌC 8 MẮT, CẬP NHẬT currentPattern + lastLinePos
// Trả về số mắt thấy line
// =========================================================================
int docSensor() {
    unsigned int pos = qtr.readLine(sensorValues);
#if REVERSE_SENSOR
    lastLinePos = 7000 - pos;
#else
    lastLinePos = pos;
#endif
    currentPattern = 0;
    int black = 0;
    for (int i = 0; i < 8; i++) {
        if (sensorValues[i] > SENSOR_THRESHOLD) {
#if REVERSE_SENSOR
            currentPattern |= (1 << i);
#else
            currentPattern |= (1 << (7 - i));
#endif
            black++;
        }
    }
    return black;
}
