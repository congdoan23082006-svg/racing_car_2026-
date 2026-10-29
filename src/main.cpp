// ================================================================
// ROBOT BLE - DÒ LINE BÁM PID
// Phần cứng: ESP32 + 2x TB6612 + 8 mắt QTR qua mux CD74HC4067
// Lệnh BLE: CAL → calibrate | AUTO → chạy | MANUAL → dừng
//           PID:kp,kd,base → chỉnh PID | GETPID → đọc PID
//           TURN:out,in → chỉnh tốc độ cua 90 độ
//           SPD:xxx / V:xxx → chỉnh tốc độ manual
// ================================================================
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "TB6612.h"   // include/dongco/TB6612.h
#include "sensor.h"   // include/mat/sensor.h  (MyQTR8C, docSensor)
#include "pid.h"      // include/pid.h         (lineFollowPID, KP, KD...)

// ================================================================
// BLE UUIDs (Nordic UART Service)
// ================================================================
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer*         pServer           = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected    = false;
bool oldDeviceConnected = false;

// ================================================================
// TRẠNG THÁI & TELEMETRY
// ================================================================
int  speed_dk = 120;    // tốc độ manual (lệnh SPD:/V:)

String currentModeName = "IDLE";
String currentCaseName = "-";
unsigned long lastStatusTime = 0;
#define STATUS_INTERVAL 150   // ms

// Hàng chờ lệnh BLE gửi ra ngoài callback (tránh delay trong ISR)
#define BLE_QUEUE_SIZE 4
static char   bleQueue[BLE_QUEUE_SIZE][128];
static int    bleQHead = 0, bleQTail = 0;

#define DEBUG_RAW 0
#if DEBUG_RAW
bool testMode = false;
unsigned long lastRawTime = 0;
#endif

// ================================================================
// BLE SEND
// ================================================================
// Gửi BLE ngay (chỉ gọi từ loop() - KHÔNG gọi từ BLE callback)
void bleSendDirect(const char* msg) {
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
        pTxCharacteristic->notify();
    }
    Serial.println(msg);
}

// Đẩy vào hàng chờ - an toàn từ bất kỳ ngữ cảnh nào (callback, ISR)
void bleSend(const char* msg) {
    Serial.println(msg);  // Luôn in Serial ngay
    int next = (bleQTail + 1) % BLE_QUEUE_SIZE;
    if (next != bleQHead) {  // còn chỗ trong queue
        strncpy(bleQueue[bleQTail], msg, 127);
        bleQueue[bleQTail][127] = '\0';
        bleQTail = next;
    }
}

// Flush hàng chờ - gọi trong loop()
void bleFlushQueue() {
    if (!deviceConnected || !pTxCharacteristic) { bleQHead = bleQTail = 0; return; }
    while (bleQHead != bleQTail) {
        pTxCharacteristic->setValue((uint8_t*)bleQueue[bleQHead], strlen(bleQueue[bleQHead]));
        pTxCharacteristic->notify();
        bleQHead = (bleQHead + 1) % BLE_QUEUE_SIZE;
        delay(3);  // Cho BLE stack thở - an toàn trong loop()
    }
}

// JSON status 150ms/lần
void sendStatus() {
    if (!deviceConnected) return;
    if (millis() - lastStatusTime < STATUS_INTERVAL) return;
    lastStatusTime = millis();

    // Xây dựng chuỗi pattern 8 bit (MSB trước)
    char patStr[9];
    for (int i = 7; i >= 0; i--)
        patStr[7 - i] = (currentPattern & (1 << i)) ? '1' : '0';
    patStr[8] = '\0';

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"mode\":\"%s\",\"case\":\"%s\","
        "\"pos\":%u,\"pat\":\"%s\","
        "\"spd\":%d,\"err\":%d,\"cal\":%d}",
        currentModeName.c_str(), currentCaseName.c_str(),
        lastLinePos, patStr,
        speed_dk, lastError,
        calibrating ? 2 : (calibrated ? 1 : 0)
    );
    pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
    pTxCharacteristic->notify();
}

// ================================================================
// XỬ LÝ LỆNH BLE
// ================================================================
void handleBLECommand(String cmd) {
    cmd.trim();

    // ---- SPD: / V: ---- tốc độ manual (KHÔNG override BASE khi đang AUTO)
    if (cmd.startsWith("SPD:") || cmd.startsWith("spd:") ||
        cmd.startsWith("V:")   || cmd.startsWith("v:")) {
        int idx = cmd.indexOf(':');
        speed_dk = constrain(cmd.substring(idx + 1).toInt(), 30, 255);
        if (!runLine) BASE = speed_dk;  // Chỉ đồng bộ BASE khi ở manual
        char buf[30]; snprintf(buf, sizeof(buf), "LOG:SPD=%d", speed_dk);
        bleSend(buf); return;
    }

#if DEBUG_RAW
    if (cmd == "TEST") {
        testMode = !testMode; runLine = false; stopMotor();
        bleSend(testMode ? "LOG:TEST_ON" : "LOG:TEST_OFF"); return;
    }
#endif

    // ---- CAL / C ---- hiệu chuẩn cảm biến
    if (cmd == "CAL" || cmd == "cal" || cmd == "C" || cmd == "c") {
        runLine = false; stopMotor();
        calibrated = false; calibrating = true;
        calEnd = millis() + CAL_TIME;
        qtr.resetCalibration(); currentPattern = 0;
        currentModeName = "CALIB"; currentCaseName = "CALIBRATING";
        bleSend("LOG:CAL_START"); return;
    }

    // ---- PID:kp,kd,base ----
    if (cmd.startsWith("PID:") || cmd.startsWith("pid:")) {
        float kp, kd; int base;
        if (sscanf(cmd.c_str() + 4, "%f,%f,%d", &kp, &kd, &base) == 3) {
            KP = kp; KD = kd; BASE = constrain(base, 0, 255);
            bleSend("LOG:PID_OK");
        } else bleSend("LOG:PID_ERR");
        return;
    }

    // ---- KP: / KD: ---- đơn lẻ
    if (cmd.startsWith("KP:") || cmd.startsWith("kp:")) {
        KP = cmd.substring(3).toFloat();
        bleSend("LOG:KP_SET"); return;
    }
    if (cmd.startsWith("KD:") || cmd.startsWith("kd:")) {
        KD = cmd.substring(3).toFloat();
        bleSend("LOG:KD_SET"); return;
    }

    // ---- TURN:out,in ---- cua 90 độ
    if (cmd.startsWith("TURN:") || cmd.startsWith("turn:")) {
        int o, i;
        if (sscanf(cmd.c_str() + 5, "%d,%d", &o, &i) == 2) {
            TURN_OUT = constrain(o, 0, 255);
            TURN_IN  = constrain(i, -255, 400);
            bleSend("LOG:TURN_OK");
        } else bleSend("LOG:TURN_ERR");
        return;
    }

    // ---- GETPID / PID ---- đọc thông số hiện tại
    if (cmd == "GETPID" || cmd == "PID" || cmd == "pid") {
        char buf[80];
        snprintf(buf, sizeof(buf), "PID:%.4f,%.4f,%d,%d,%d",
            KP, KD, BASE, TURN_OUT, TURN_IN);
        bleSend(buf); return;
    }

    // ---- AUTO / A ---- bật chạy tự động
    if (cmd == "AUTO" || cmd == "auto" || cmd == "A" || cmd == "a") {
        if (!calibrated) { bleSend("LOG:NEED_CAL"); return; }
        runLine = true;
        lastError = 0; lastLinePos = SETPOINT;
        cnt = 0; cntDen = 0;
        inGap = false; gapStart = millis();  // ← reset gapStart tránh false-positive
        fullBlackFrames = 0; inFullBlack = false;
        BASE = speed_dk;  // Đồng bộ BASE = tốc độ hiện tại lúc bắt đầu AUTO
        currentModeName = "AUTO"; currentCaseName = "PID_FOLLOW";
        bleSend("LOG:AUTO_START"); return;
    }

    // ---- MANUAL / M / STP / S ---- dừng về tay
    if (cmd == "MANUAL" || cmd == "manual" ||
        cmd == "M"      || cmd == "m"      ||
        cmd == "STP"    || cmd == "S"      || cmd == "s") {
        runLine = false; stopMotor();
        currentModeName = "MANUAL"; currentCaseName = "IDLE";
        bleSend("LOG:MODE_MANUAL"); return;
    }

    // ---- P ---- đọc pattern 8 mắt ngay lập tức
    if (cmd == "P" || cmd == "p") {
        docSensor();
        char patStr[9];
        for (int i = 7; i >= 0; i--)
            patStr[7 - i] = (currentPattern & (1 << i)) ? '1' : '0';
        patStr[8] = '\0';
        char buf[48]; snprintf(buf, sizeof(buf), "SENSORS: %s\n", patStr);
        bleSend(buf); return;
    }

    // ---- Điều khiển tay (chỉ khi không chạy Auto) ----
    if (!runLine) {
        if      (cmd == "FWD" || cmd == "F" || cmd == "f") move(speed_dk, speed_dk);
        else if (cmd == "BWD" || cmd == "B" || cmd == "b") move(-speed_dk, -speed_dk);
        else if (cmd == "LFT" || cmd == "L" || cmd == "l") move(-speed_dk, speed_dk);
        else if (cmd == "RGT" || cmd == "R" || cmd == "r") move(speed_dk, -speed_dk);
        else if (cmd == "STP" || cmd == "S" || cmd == "s") stopMotor();
    }
}

// ================================================================
// BLE CALLBACKS
// ================================================================
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        std::string v = pChar->getValue();
        if (v.length() > 0) handleBLECommand(String(v.c_str()));
    }
};

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* p) override {
        deviceConnected = true;
        bleSend("LOG:BLE_CONNECTED");
    }
    void onDisconnect(BLEServer* p) override {
        deviceConnected = false;
        stopMotor(); runLine = false;
        BLEDevice::startAdvertising();
    }
};

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);

    // Khởi tạo động cơ
    tb6612_init();

    // Khởi tạo BLE
    BLEDevice::init("ROBOT_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pSvc = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pSvc->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic* pRx = pSvc->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pRx->setCallbacks(new MyCallbacks());

    pSvc->start();
    BLEAdvertising* pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising... ROBOT_BLE");

    // Khởi tạo cảm biến, nạp calib từ EEPROM
    qtr.begin();
    if (qtr.loadCalibration()) {
        calibrated = true;
        Serial.println("Da nap calib tu EEPROM");
    }

    // Khởi tạo PID
    pid_init();

    Serial.println("READY! Gui CAL de calibrate, sau do AUTO de chay");
    bleSend("LOG:ROBOT_READY");
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    // Quản lý kết nối BLE
    if (!deviceConnected && oldDeviceConnected) {
        delay(300);
        BLEDevice::startAdvertising();
        oldDeviceConnected = false;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true;
    }

    // ---- Chế độ CALIB ----
    if (calibrating) {
        qtr.calibrate();
        if (millis() >= calEnd) {
            calibrating = false; calibrated = true;
            currentModeName = "IDLE"; currentCaseName = "-";

            // Báo cáo kênh calib kém
            char bad[48] = "LOG:CAL_WARN mat:";
            bool anyBad = false;
            for (int c = 0; c < 8; c++) {
                if (!qtr.channelOk(c)) {
                    char t[4]; snprintf(t, sizeof(t), " %d", c + 1);
                    strncat(bad, t, sizeof(bad) - strlen(bad) - 1);
                    anyBad = true;
                }
            }
            if (anyBad) bleSend(bad);
            qtr.saveCalibration();
            bleSend("LOG:CAL_DONE");
        }
    }

    // ---- Chế độ AUTO (dò line PID) ----
    if (runLine) {
        bool wasRunning = runLine;         // snapshot trước khi gọi
        lineFollowPID();
        if (!runLine && wasRunning) {      // endLine() vừa gọi bên trong
            currentModeName = "IDLE";
            currentCaseName = "END_LINE";
        } else if (runLine) {              // vẫn đang chạy → cập nhật case
            currentModeName = "AUTO";
            if      (currentPattern == 0b11111111)   currentCaseName = "END_LINE";
            else if (currentPattern & 0b00000001)    currentCaseName = "TURN_RIGHT";
            else if (currentPattern & 0b10000000)    currentCaseName = "TURN_LEFT";
            else if (inGap)                          currentCaseName = "GAP_STRAIGHT";
            else if (currentPattern == 0)            currentCaseName = "LOST_LINE";
            else                                     currentCaseName = "PID_FOLLOW";
        }
    }
    // ---- Ở chế độ idle nhưng đã calib: vẫn đọc sensor để hiển thị ----
    else if (calibrated && !calibrating) {
        docSensor();
    }
    else {
        currentPattern = 0;
    }

#if DEBUG_RAW
    if (testMode && !calibrating && !runLine) {
        docSensor();
        if (deviceConnected && (millis() - lastRawTime > 200)) {
            lastRawTime = millis();
            char raw[72];
            snprintf(raw, sizeof(raw), "RAW:%u,%u,%u,%u,%u,%u,%u,%u",
                sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
                sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);
            bleSend(raw);
        }
    }
#endif

    // Flush hàng chờ BLE (an toàn ngoài callback)
    bleFlushQueue();

    // Gửi JSON telemetry 150ms/lần
    sendStatus();
}
