#include "WString.h"
#include "BleCompositeHID.h"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>
#include <esp_gap_ble_api.h>
#include <Preferences.h>

#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
#endif

#define LOG_TAG "BleCompositeHID"

#define SERVICE_UUID_DEVICE_INFORMATION        "180A"      // Service - Device information

#define CHARACTERISTIC_UUID_SYSTEM_ID          "2A23"      // Characteristic - System ID 0x2A23
#define CHARACTERISTIC_UUID_MODEL_NUMBER       "2A24"      // Characteristic - Model Number String - 0x2A24
#define CHARACTERISTIC_UUID_SOFTWARE_REVISION  "2A28"      // Characteristic - Software Revision String - 0x2A28
#define CHARACTERISTIC_UUID_SERIAL_NUMBER      "2A25"      // Characteristic - Serial Number String - 0x2A25
#define CHARACTERISTIC_UUID_FIRMWARE_REVISION  "2A26"      // Characteristic - Firmware Revision String - 0x2A26
#define CHARACTERISTIC_UUID_HARDWARE_REVISION  "2A27"      // Characteristic - Hardware Revision String - 0x2A27

// Set MAX_CLIENTS to 1-4 to allow N computers/phones/quests to connect at once
//    If set to 1, uses blacklist logic to avoid reconnecting to the same device for 15 sec after a 
//    reconect, to allow another device to connect
#define MAX_CLIENTS 2


uint16_t vidSource;
uint16_t vid;
uint16_t pid;
uint16_t guidVersion;

std::string modelNumber;
std::string softwareRevision;
std::string serialNumber;
std::string firmwareRevision;
std::string hardwareRevision;
std::string systemID;


static Preferences preferences;
std::vector<uint64_t> pairedAddresses;
#define MAX_ADDRESSES 4

void loadPairedAddresses()
{
    if (MAX_CLIENTS == 1) {
        preferences.begin(LOG_TAG);
        int numBytes = preferences.getBytesLength("pairedAddresses");
        if (numBytes) {
            uint8_t *bytes = (uint8_t*)malloc(numBytes);
            if (bytes) {
                preferences.getBytes("pairedAddresses", bytes, numBytes);
                int pos = 0;
                while (pos < numBytes) {
                    uint64_t *addrInt = (uint64_t*)(bytes+pos);
                    pairedAddresses.push_back(*addrInt);
                    pos += sizeof(uint64_t);
                }
                free(bytes);
            }
        }
    }
}

void savePairedAddresses()
{
    if (MAX_CLIENTS == 1) {
        // Keep within MAX_ADDRESSES addresses
        while (pairedAddresses.size() > MAX_ADDRESSES) {
            pairedAddresses.erase(pairedAddresses.begin());
        }

        int numBytes = pairedAddresses.size() * sizeof(uint64_t);
        uint8_t *bytes = (uint8_t*)malloc(numBytes);
        if (bytes) {
            int pos = 0;
            for (uint64_t addrInt : pairedAddresses) {
                ((uint64_t*)bytes)[pos++] = addrInt;
            }
            preferences.putBytes("pairedAddresses", bytes, numBytes);
            free(bytes);
        }
    }
}


class BleConnectionStatus : public NimBLEServerCallbacks
{
public:
    bool is_connected = false;
    int num_connected = 0;
    uint64_t lastAddr = 0;
    BleCompositeHID *hid;

    void onConnect(NimBLEServer *pServer, ble_gap_conn_desc* desc)
    {
        if (MAX_CLIENTS == 1) {
            NimBLEAddress addr = NimBLEAddress(desc->peer_ota_addr);
            uint64_t addrInt = uint64_t(addr);
            if (this->lastAddr && addrInt == this->lastAddr) {
                printf("Reject lastAdr\n");
                NimBLEDevice::getServer()->disconnect(desc->conn_handle);
                return;
            }
        }

        pServer->updateConnParams(desc->conn_handle, 6, 24, 0, 120);
        NimBLEDevice::startSecurity(desc->conn_handle);
        this->num_connected++;
        printf("Connected\n");
    }

    void onAuthenticationComplete(ble_gap_conn_desc* desc)
    {
        if(!desc->sec_state.encrypted) {
            printf("Auth failed\n");
            NimBLEDevice::getServer()->disconnect(desc->conn_handle);
            return;
        }

        if (MAX_CLIENTS == 1) {
            NimBLEAddress addr = NimBLEAddress(desc->peer_ota_addr);
            uint64_t addrInt = uint64_t(addr);
            if (std::find(pairedAddresses.begin(), pairedAddresses.end(), addrInt) == pairedAddresses.end()) {
                pairedAddresses.push_back(addrInt);
                savePairedAddresses();
                printf("Added paired address\n");
            }
        }

        this->is_connected = true;
        printf("Auth complete\n");

        if (this->num_connected < MAX_CLIENTS) {
            this->hid->startAdvertising(false);
        }
    }

    void onDisconnect(NimBLEServer *pServer, ble_gap_conn_desc* desc)
    {
        NimBLEAddress addr = NimBLEAddress(desc->peer_ota_addr);
        uint64_t addrInt = uint64_t(addr);
        this->lastAddr = addrInt;

        this->num_connected--;
        if (this->num_connected == 0) {
            this->is_connected = false;
        }
        printf("Disconnected\n");

        this->hid->startAdvertising(true);
    }
};


std::string uint8_to_hex_string(const uint8_t *v, const size_t s) {
  std::stringstream ss;

  ss << std::hex << std::setfill('0');

  for (int i = 0; i < s; i++) {
    ss << std::hex << std::setw(2) << static_cast<int>(v[i]);
  }

  return ss.str();
}

BleCompositeHID::BleCompositeHID(std::string deviceName, std::string deviceManufacturer, uint8_t batteryLevel) : _hid(nullptr)
{
    this->deviceName = deviceName.substr(0, CONFIG_BT_NIMBLE_GAP_DEVICE_NAME_MAX_LEN - 1);
    this->deviceManufacturer = deviceManufacturer;
    this->batteryLevel = batteryLevel;
    this->_connectionStatus = new BleConnectionStatus();
    this->_connectionStatus->hid = this;
}

BleCompositeHID::~BleCompositeHID()
{
    delete this->_connectionStatus;
}

void BleCompositeHID::begin()
{
    this->begin(BLEHostConfiguration());
}

void BleCompositeHID::begin(const BLEHostConfiguration& config)
{
    _configuration = config; // we make a copy, so the user can't change actual values midway through operation, without calling the begin function again

    modelNumber = _configuration.getModelNumber();
    softwareRevision = _configuration.getSoftwareRevision();
    serialNumber = _configuration.getSerialNumber();
    firmwareRevision = _configuration.getFirmwareRevision();
    hardwareRevision = _configuration.getHardwareRevision();
    systemID = _configuration.getSystemID();

    vidSource = _configuration.getVidSource();
    vid = _configuration.getVid();
    pid = _configuration.getPid();
    guidVersion = _configuration.getGuidVersion();

    uint8_t high = highByte(vid);
    uint8_t low = lowByte(vid);

    vid = low << 8 | high;

    high = highByte(pid);
    low = lowByte(pid);

    pid = low << 8 | high;
    
    high = highByte(guidVersion);
    low = lowByte(guidVersion);
    guidVersion = low << 8 | high;

    loadPairedAddresses();

    // Start BLE server
    this->startServer();
}

void BleCompositeHID::addDevice(BaseCompositeDevice *device)
{
    device->_parent = this;
    _devices.push_back(device);
}

bool BleCompositeHID::isConnected()
{
    return this->_connectionStatus->is_connected;
}

void BleCompositeHID::setBatteryLevel(uint8_t level)
{
    this->batteryLevel = level;
    if (this->_hid)
    {
        this->_hid->setBatteryLevel(this->batteryLevel);

        if (this->isConnected())
        {
            this->_hid->batteryLevel()->notify();
        }
    }
}

void BleCompositeHID::clearPairedAddresses()
{
    pairedAddresses.clear();
    preferences.remove(LOG_TAG);
}

void BleCompositeHID::startAdvertising(bool useBlackList)
{
    // Start BLE advertisement
    NimBLEAdvertising *pAdvertising = this->_pServer->getAdvertising();
    
    if (MAX_CLIENTS == 1 && useBlackList && this->_connectionStatus->lastAddr) {
        if (this->_connectionStatus->lastAddr && pairedAddresses.size() >= 2) {
            printf("Start Advertising - Use blacklist\n");
            pAdvertising->start(15, [this](NimBLEAdvertising *pAdvertising){ this->onAdvComplete(pAdvertising); });
            return;
        }
    }

    printf("Start Advertising - No blacklist\n");
    this->_connectionStatus->lastAddr = 0;
    pAdvertising->start();
}

void BleCompositeHID::onAdvComplete(NimBLEAdvertising *pAdvertising) {
    // printf("Advertising stopped\n");
    if (this->isConnected()) {
        return;
    }
    // If advertising timed out without connection start advertising without whitelist filter
    this->startAdvertising(false);
}


void BleCompositeHID::startServer()
{
    NimBLEDevice::init(this->deviceName);
    NimBLEDevice::setSecurityAuth(true, false, false);
    // NimBLEDevice::setPower(ESP_PWR_LVL_P15);  // +15db

    this->_pServer = NimBLEDevice::createServer();
    this->_pServer->setCallbacks(this->_connectionStatus);
    this->_pServer->advertiseOnDisconnect(false);

    this->_hid = new NimBLEHIDDevice(this->_pServer);
    this->_hid->manufacturer()->setValue(this->deviceManufacturer);
    this->_hid->pnp(vidSource, vid, pid, guidVersion);
    this->_hid->hidInfo(0x00, 0x01);

    // Setup the HID descriptor buffers
    size_t totalBufferSize = 2048;
    uint8_t tempHidReportDescriptor[totalBufferSize];
    int hidReportDescriptorSize = 0;
    ESP_LOGD(LOG_TAG, "About to init devices");
    
    // Setup child devices to build the HID report descriptor
    for (auto device : this->_devices) {
        ESP_LOGD(LOG_TAG, "Before device %s init", device->getDeviceConfig()->getDeviceName());
        device->init(this->_hid);
        ESP_LOGD(LOG_TAG, "After device %s init", device->getDeviceConfig()->getDeviceName());
        
        auto config = device->getDeviceConfig();
        size_t reportSize = config->makeDeviceReport(tempHidReportDescriptor + hidReportDescriptorSize, totalBufferSize - hidReportDescriptorSize);
        
        if (reportSize >= BLE_ATT_ATTR_MAX_LEN) {
            ESP_LOGE(LOG_TAG, "Device report size %d is larger than max buffer size %d", reportSize, BLE_ATT_ATTR_MAX_LEN);
            return;
        } else if (reportSize == 0) {
            ESP_LOGE(LOG_TAG, "Device report size is 0");
            return;
        } else if (reportSize < 0) {
            ESP_LOGE(LOG_TAG, "Error creating report for device %s", config->getDeviceName());
            return;
        } else {
            ESP_LOGD(LOG_TAG, "Created device %s with report size %d", config->getDeviceName(), reportSize);
        }

        hidReportDescriptorSize += reportSize;
    }
    ESP_LOGD(LOG_TAG, "Final hidReportDescriptorSize: %d", hidReportDescriptorSize);

    // Set the report map
    uint8_t *customHidReportDescriptor = (uint8_t*)malloc(hidReportDescriptorSize);
    memcpy(customHidReportDescriptor, tempHidReportDescriptor, hidReportDescriptorSize);
    this->_hid->reportMap(customHidReportDescriptor, hidReportDescriptorSize);

    // Create device UUID
    NimBLEService *pService = this->_pServer->getServiceByUUID(SERVICE_UUID_DEVICE_INFORMATION);
  
    // Create characteristics
    BLECharacteristic* pCharacteristic_Model_Number = pService->createCharacteristic(
      CHARACTERISTIC_UUID_MODEL_NUMBER,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pCharacteristic_Model_Number->setValue(modelNumber);
  
    // BLECharacteristic* pCharacteristic_Software_Revision = pService->createCharacteristic(
    //   CHARACTERISTIC_UUID_SOFTWARE_REVISION,
    //   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    // );
    // pCharacteristic_Software_Revision->setValue(softwareRevision);
  
    BLECharacteristic* pCharacteristic_Serial_Number = pService->createCharacteristic(
      CHARACTERISTIC_UUID_SERIAL_NUMBER,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pCharacteristic_Serial_Number->setValue(serialNumber);
  
    // BLECharacteristic* pCharacteristic_Firmware_Revision = pService->createCharacteristic(
    //   CHARACTERISTIC_UUID_FIRMWARE_REVISION,
    //   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    // );
    // pCharacteristic_Firmware_Revision->setValue(firmwareRevision);
  
    // BLECharacteristic* pCharacteristic_Hardware_Revision = pService->createCharacteristic(
    //   CHARACTERISTIC_UUID_HARDWARE_REVISION,
    //   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    // );
    // pCharacteristic_Hardware_Revision->setValue(hardwareRevision);

    // Start BLE server
    this->_hid->startServices();

    // Start BLE advertisement
    NimBLEAdvertising *pAdvertising = this->_pServer->getAdvertising();
    pAdvertising->setAppearance(HID_GAMEPAD);
    pAdvertising->addServiceUUID(this->_hid->deviceInfo()->getUUID());
    pAdvertising->addServiceUUID(this->_hid->hidService()->getUUID());
    pAdvertising->addServiceUUID(this->_hid->batteryService()->getUUID());
    pAdvertising->setScanResponse(true);

    // Update battery
    this->setBatteryLevel(this->batteryLevel);

    this->startAdvertising(true);

    ESP_LOGD(LOG_TAG, "Advertising started!");
}
