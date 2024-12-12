#ifndef ESP32_BLE_MULTI_HID_H
#define ESP32_BLE_MULTI_HID_H
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#include "nimconfig.h"
#if defined(CONFIG_BT_NIMBLE_ROLE_PERIPHERAL)

#include "NimBLEHIDDevice.h"
#include "NimBLECharacteristic.h"

#include "BLEHostConfiguration.h"
#include "BaseCompositeDevice.h"

#include <vector>
#include "SafeQueue.hpp"

class BleConnectionStatus;

class BleCompositeHID
{
public:
    BleCompositeHID(std::string deviceName = "ESP32 BLE Composite HID", std::string deviceManufacturer = "Espressif", uint8_t batteryLevel = 100);
    ~BleCompositeHID();
    void begin();
    void begin(const BLEHostConfiguration& config);
    void end();

    void setBatteryLevel(uint8_t level);
    void addDevice(BaseCompositeDevice* device);
    bool isConnected();
    void startAdvertising(bool useBlackList);
    void clearPairedAddresses();
    NimBLEServer* getServer() { return this->_pServer; }
    NimBLEHIDDevice* getHIDDevice() { return this->_hid; }

    uint8_t batteryLevel;
    std::string deviceManufacturer;
    std::string deviceName;

private:
    void startServer();
    void onAdvComplete(NimBLEAdvertising *pAdvertising);

    NimBLEServer *_pServer;
    BLEHostConfiguration _configuration;
    BleConnectionStatus* _connectionStatus;
    NimBLEHIDDevice* _hid;

    std::vector<BaseCompositeDevice*> _devices;
};

#endif // CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#endif // CONFIG_BT_ENABLED
#endif // ESP32_BLE_MULTI_HID_H
