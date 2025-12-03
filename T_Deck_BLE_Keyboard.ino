#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "utilities.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDKeyboardTypes.h>
#include <Wire.h>
#include "keymap.h"
#include "hid_descriptor.h"

#define LILYGO_KB_SLAVE_ADDRESS 0x55
#define LILYGO_KB_MODE_RAW_CMD 0x03
#define BOARD_POWERON 10
#define BOARD_I2C_SDA 18
#define BOARD_I2C_SCL 8

TFT_eSPI tft;
String message = "Hello World!";

BLEServer *pServer;
BLEHIDDevice *hid;
BLECharacteristic *input;
BLECharacteristic *output;
BLEAdvertising *pAdvertising;
bool bleConnected = false;
bool symbolMode = false;

// 扫描状态
bool curState[colCount][rowCount] = {0};
bool lastState[colCount][rowCount] = {0};

// 上一份已发送的 HID 输入报告（8 字节键盘）
uint8_t lastReport[8] = {0};

// 构造当前 HID 报告（根据矩阵扫描）
void buildCurrentReport(uint8_t report[8])
{
    // 8 字节键盘内容
    uint8_t kb[8] = {0};

    uint8_t modifier = 0;
    uint8_t keys[6] = {0};
    int idx = 0;

    for (int col = 0; col < colCount; col++)
    {
        for (int row = 0; row < rowCount; row++)
        {
            if (!curState[col][row])
                continue;
            const Key &k = symbolMode ? keymap_symbol_flat[col * rowCount + row]
                                      : keymap_flat[col * rowCount + row];
            modifier |= k.mod;
            if (k.hid && idx < 6)
                keys[idx++] = k.hid;
        }
    }

    kb[0] = modifier; // 修饰键
    kb[1] = 0x00;     // 保留
    for (int i = 0; i < 6; i++)
        kb[2 + i] = keys[i];

    // 拷贝到带 ID 的报告缓冲
    memcpy(&report[0], kb, 8);
}

bool reportChanged(const uint8_t a[8], const uint8_t b[8])
{
    for (int i = 0; i < 8; i++)
        if (a[i] != b[i])
            return true;
    return false;
}

void sendCurrentReport(const uint8_t report[8])
{
    input->setValue((uint8_t *)report, 8);
    input->notify();
    Serial.printf("mod=0x%02X keys=[%02X %02X %02X %02X %02X %02X]\n",
                  report[0], report[2], report[3], report[4], report[5], report[6], report[7]);
}

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) override
    {
        bleConnected = true;
        Serial.println("BLE: connected");
        // hid->setBatteryLevel(100);

        uint8_t report[8] = {0};
        sendCurrentReport(report);
        memcpy(lastReport, report, 8);
    }
    void onDisconnect(BLEServer *pServer) override
    {
        bleConnected = false;
        Serial.println("BLE: disconnected");

        uint8_t zero[8] = {0};
        sendCurrentReport(zero);
        memcpy(lastReport, zero, 8);

        if (pAdvertising)
            pAdvertising->start();
        else
            BLEDevice::getAdvertising()->start();
    }
};

class MyOutputCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override
    {
        std::string value = characteristic->getValue();
        if (!value.empty())
        {
            uint8_t leds = value[0];
            Serial.printf("LED state changed: NumLock=%d CapsLock=%d ScrollLock=%d\n",
                          leds & 0x01, (leds >> 1) & 0x01, (leds >> 2) & 0x01);
        }
    }
};

void setup()
{
    Serial.begin(115200);
    Serial.println("T-Deck HID Keyboard");

    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    pinMode(BOARD_SDCARD_CS, OUTPUT);
    pinMode(RADIO_CS_PIN, OUTPUT);
    pinMode(BOARD_TFT_CS, OUTPUT);
    digitalWrite(BOARD_SDCARD_CS, HIGH);
    digitalWrite(RADIO_CS_PIN, HIGH);
    digitalWrite(BOARD_TFT_CS, HIGH);

    pinMode(BOARD_SPI_MISO, INPUT_PULLUP);
    SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI);

    delay(500);
    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    // 切换到 Raw Mode
    Wire.beginTransmission(LILYGO_KB_SLAVE_ADDRESS);
    Wire.write(LILYGO_KB_MODE_RAW_CMD);
    Wire.endTransmission();

    // 初始化 BLE HID
    BLEDevice::init("T-Deck Keyboard");

    BLESecurity *pSecurity = new BLESecurity();
    // 设置安全参数
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_MITM | ESP_LE_AUTH_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setKeySize(16); // 配对密钥长度
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    hid = new BLEHIDDevice(pServer);
    input = hid->inputReport(0);
    output = hid->outputReport(0);
    output->setCallbacks(new MyOutputCallbacks());

    hid->manufacturer()->setValue("LILYGO");
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00, 0x01);

    // 设置 HID 报告描述符并启动服务
    hid->reportMap((uint8_t *)hidReportDescriptor, sizeof(hidReportDescriptor));
    hid->startServices();

    // 配置并启动广播
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAppearance(HID_KEYBOARD);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();

    // TFT 显示初始化
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setFreeFont(&FreeSansOblique12pt7b);
    // tft.drawString(String(pin), TFT_WIDTH / 2, TFT_HEIGHT / 2);

    pinMode(BOARD_BL_PIN, OUTPUT);
    digitalWrite(BOARD_BL_PIN, HIGH);

    // 初始化 lastReport 为全零
    memset(lastReport, 0, sizeof(lastReport));
}

void loop()
{
    // 读取列状态
    Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, colCount);
    uint8_t cols[colCount] = {0};
    int got = 0;
    while (Wire.available() && got < colCount)
    {
        cols[got++] = Wire.read();
    }

    // 更新当前矩阵状态
    for (int col = 0; col < colCount; col++)
    {
        uint8_t val = cols[col];
        for (int row = 0; row < rowCount; row++)
        {
            bool pressed = ((val >> row) & 0x01) != 0;
            curState[col][row] = pressed;
        }
    }

    if (curState[0][2])
    {
        symbolMode = true;
    }
    else
    {
        symbolMode = false;
    }

    // 构造并发送当前报告（仅在变化时）
    if (bleConnected)
    {
        uint8_t report[8];
        buildCurrentReport(report);
        // sendCurrentReport(report);
        if (reportChanged(report, lastReport))
        {
            sendCurrentReport(report);
            memcpy(lastReport, report, 8);
        }
    }

    static bool lastSymbolMode = false;
    if (symbolMode != lastSymbolMode)
    {
        tft.fillScreen(TFT_BLACK);
        if (symbolMode)
            tft.drawString("Sym", TFT_HEIGHT / 2, TFT_WIDTH / 2);
        lastSymbolMode = symbolMode;
    }

    delay(15); // 扫描间隔
}