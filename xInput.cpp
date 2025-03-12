#include "xInput.h"
#include <Preferences.h>
#include <NimBLEDevice.h>

#define PREFS_NAME "XInput_BLE"

// Set MAX_CLIENTS to 1-4 to allow N computers/phones/quests to connect at once
//    If set to 1, uses blacklist logic to avoid reconnecting to the same device for 15 sec after a 
//    reconect, to allow another device to connect
#define MAX_CLIENTS 4

static Preferences preferences;
std::vector<uint64_t> pairedAddresses;
#define MAX_ADDRESSES 4

void loadPairedAddresses()
{
    if (MAX_CLIENTS == 1) {
        preferences.begin(PREFS_NAME);
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

static uint8_t dPadDirectionToValue(XboxDpadFlags direction){
    if(direction == XboxDpadFlags::NORTH)
        return XBOX_BUTTON_DPAD_NORTH;
    else if(direction == (XboxDpadFlags::EAST | XboxDpadFlags::NORTH))
        return XBOX_BUTTON_DPAD_NORTHEAST;
    else if(direction == XboxDpadFlags::EAST)
        return XBOX_BUTTON_DPAD_EAST;
    else if(direction == (XboxDpadFlags::EAST | XboxDpadFlags::SOUTH))
        return XBOX_BUTTON_DPAD_SOUTHEAST;
    else if(direction == XboxDpadFlags::SOUTH)
        return XBOX_BUTTON_DPAD_SOUTH;
    else if(direction == (XboxDpadFlags::WEST | XboxDpadFlags::SOUTH))
        return XBOX_BUTTON_DPAD_SOUTHWEST;
    else if(direction == XboxDpadFlags::WEST)
        return XBOX_BUTTON_DPAD_WEST;
    else if(direction == (XboxDpadFlags::WEST | XboxDpadFlags::NORTH))
        return XBOX_BUTTON_DPAD_NORTHWEST;
    
    return XBOX_BUTTON_DPAD_NONE;
}


class HIDOutputCallbacks : public NimBLECharacteristicCallbacks
{
public:
    HIDOutputCallbacks(XInput* xInput) : _xInput(xInput) {}

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override
    {
        // An example packet we might receive from XInput might look like 0x0300002500ff00ff
        XboxGamepadOutputReportData vibrationData = pCharacteristic->getValue<uint64_t>();
        this->_xInput->onVibrate.fire(vibrationData);
    }

private:
    XInput* _xInput;
};


class ServerCallbacks : public NimBLEServerCallbacks
{
public:
    bool is_connected = false;
    uint64_t lastAddr = 0;

    ServerCallbacks(XInput* xInput) : _xInput(xInput) {}

    void onConnect(NimBLEServer *server, NimBLEConnInfo& connInfo) override
    {
        // Detect old connections from the same peer address, and disconnect them
        // This section works around issue #886  https://github.com/h2zero/NimBLE-Arduino/issues/886
        std::vector<uint16_t> peers = server->getPeerDevices();
        for (const uint16_t& peerHandle : peers) {
            NimBLEConnInfo peer = server->getPeerInfoByHandle(peerHandle);
            if (peer.getConnHandle() != connInfo.getConnHandle() && peer.getAddress() == connInfo.getAddress()) {
                printf("--- Removing duplicate peer\n");
                server->disconnect(peerHandle);
            }
        }
        // End workaround

        if (MAX_CLIENTS == 1) {
            NimBLEAddress addr = NimBLEAddress(connInfo.getAddress());
            uint64_t addrInt = uint64_t(addr);
            if (this->lastAddr && addrInt == this->lastAddr) {
                printf("Reject lastAdr\n");
                server->disconnect(connInfo.getConnHandle());
                return;
            }
        }

        server->updateConnParams(connInfo.getConnHandle(), 6, 12, 0, 160);
        NimBLEDevice::startSecurity(connInfo.getConnHandle());
        printf("Connected %d                  (%s)\n", server->getConnectedCount(),
          connInfo.getAddress().toString().c_str());

        if (NimBLEDevice::getServer()->getConnectedCount() < MAX_CLIENTS) {
            this->_xInput->startAdvertising(false);
        }
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override
    {
        if (!connInfo.isEncrypted()) {
            printf("Auth failed                  (%s)\n", connInfo.getAddress().toString().c_str());
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            return;
        }

        if (MAX_CLIENTS == 1) {
            NimBLEAddress addr = NimBLEAddress(connInfo.getAddress());
            uint64_t addrInt = uint64_t(addr);
            if (std::find(pairedAddresses.begin(), pairedAddresses.end(), addrInt) == pairedAddresses.end()) {
                pairedAddresses.push_back(addrInt);
                savePairedAddresses();
                printf("Added paired address\n");
            }
        }

        this->is_connected = true;
        printf("Auth complete                (%s)\n", connInfo.getAddress().toString().c_str());
    }

    // void onConnParamsUpdate(NimBLEConnInfo& connInfo) override
    // {
    //     printf("Conn Params: %d, %d, %d, %d (%s)\n", connInfo.getConnInterval(),
    //         connInfo.getConnTimeout(), connInfo.getConnLatency(), connInfo.getMTU(),
    //         connInfo.getAddress().toString().c_str());
    // }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo& connInfo, int reason) override
    {
        NimBLEAddress addr = NimBLEAddress(connInfo.getAddress());
        uint64_t addrInt = uint64_t(addr);
        this->lastAddr = addrInt;

        if (server->getConnectedCount() == 0) {
            this->is_connected = false;
        }
        printf("Disconnected %d %d           (%s)\n", server->getConnectedCount(), 
            reason, connInfo.getAddress().toString().c_str());
            
        this->_xInput->startAdvertising(false);
    }

private:
    XInput *_xInput = NULL;
};


void XInput::startServer(const char *device_name, const char *manufacturer)
{
    NimBLEDevice::init(device_name);
    NimBLEDevice::setSecurityAuth(true, false, false);

    this->_server = NimBLEDevice::createServer();
    this->_serverCallbacks = new ServerCallbacks(this);
    this->_server->setCallbacks(this->_serverCallbacks);

    this->_hid = new NimBLEHIDDevice(this->_server);
    this->_hid->setManufacturer(manufacturer);
    this->_hid->setPnp(VENDOR_USB_SOURCE, XBOX_VENDOR_ID, XBOX_PRODUCT_ID, XBOX_BCD_DEVICE_ID);
    this->_hid->setHidInfo(0x00, 0x01);
    this->_hid->setReportMap((uint8_t*)Xbox_HIDDescriptor, sizeof(Xbox_HIDDescriptor));
    // printf("HID report size: %d", sizeof(Xbox_HIDDescriptor));

    this->_input = this->_hid->getInputReport(XBOX_INPUT_REPORT_ID);
    this->_input->setValue((uint8_t*)&_inputReport, sizeof(_inputReport));

    this->_output = this->_hid->getOutputReport(XBOX_OUTPUT_REPORT_ID);
    this->_hidOutputCallbacks = new HIDOutputCallbacks(this);
    this->_output->setCallbacks(this->_hidOutputCallbacks);

    // Create device UUID
    NimBLEService *service = this->_server->getServiceByUUID(SERVICE_UUID_DEVICE_INFORMATION);
  
    // Create characteristics
    BLECharacteristic* characteristic_Model_Number = service->createCharacteristic(
      CHARACTERISTIC_UUID_MODEL_NUMBER,
      NIMBLE_PROPERTY::READ);
    static char* modelNumber = XBOX_MODEL;
    characteristic_Model_Number->setValue((const uint8_t*)modelNumber, strlen(modelNumber));
  
    BLECharacteristic* characteristic_Serial_Number = service->createCharacteristic(
      CHARACTERISTIC_UUID_SERIAL_NUMBER,
      NIMBLE_PROPERTY::READ);
    static char* serialNumber = XBOX_SERIAL;
    characteristic_Serial_Number->setValue((const uint8_t*)serialNumber, strlen(serialNumber));
  
    BLECharacteristic* characteristic_Firmware_Revision = service->createCharacteristic(
      CHARACTERISTIC_UUID_FIRMWARE_REVISION,
      NIMBLE_PROPERTY::READ);
    static char* firmwareRevision = XBOX_FW_VER;
    characteristic_Firmware_Revision->setValue((const uint8_t*)firmwareRevision, strlen(firmwareRevision));
  
    this->_hid->setBatteryLevel(100);

    this->_hid->startServices();

    // Start BLE advertisement
    this->_advertising = this->_server->getAdvertising();
    this->_advertising->setAppearance(HID_GAMEPAD);
    this->_advertising->addServiceUUID(this->_hid->getHidService()->getUUID());
    this->_advertising->enableScanResponse(true);
    this->_advertising->start();
}

bool XInput::isConnected()
{
    return this->_serverCallbacks->is_connected;
}

bool XInput::isAdvertising()
{
    return this->_advertising->isAdvertising();
}

void XInput::startAdvertising(bool useBlackList)
{
    // Start BLE advertisement
    if (MAX_CLIENTS == 1 && useBlackList && this->_serverCallbacks->lastAddr) {
        if (this->_serverCallbacks->lastAddr && pairedAddresses.size() >= 2) {
            printf("Start Advertising - Use blacklist\n");
            this->_advertising->start(15000);
            this->_advertising->setAdvertisingCompleteCallback([this](NimBLEAdvertising *pAdvertising){
               this->onAdvComplete(pAdvertising);
            });
            return;
        }
    }

    if (this->_server->getConnectedCount() >= MAX_CLIENTS) {
        printf("Skip Starting Advertising - already connected to MAX_CLIENTS\n");
        return;
    }

    printf("Start Advertising\n");
    this->_serverCallbacks->lastAddr = 0;
    if (this->_server->getConnectedCount() > 0) {
        this->_advertising->start();
        this->_advertising->setAdvertisingCompleteCallback([this](NimBLEAdvertising *pAdvertising){
            this->onAdvComplete(pAdvertising);
        });
    }
    else {
        // No connections yet, so advertise indefinitely
        this->_advertising->start();
        this->_advertising->setAdvertisingCompleteCallback([this](NimBLEAdvertising *pAdvertising){
            this->onAdvComplete(pAdvertising);
        });
    }
}

void XInput::onAdvComplete(NimBLEAdvertising *advertising) {
    printf("Advertising stopped\n");
    if (this->isConnected()) {
        return;
    }
    // If advertising timed out without connection start advertising without whitelist filter
    this->startAdvertising(false);
}

void XInput::clearPairedAddresses()
{
    if (MAX_CLIENTS == 1) {
        pairedAddresses.clear();
        preferences.remove("pairedAddresses");
    }
}


///////////////////////////////////////////
// Handle button presses / stick movements

void XInput::press(uint16_t button) {
    // Avoid double presses
    if (!isPressed(button)) {
        _inputReport.buttons |= button;
    }
}

void XInput::release(uint16_t button) {
    // Avoid double presses
    if (isPressed(button)) {   
        _inputReport.buttons ^= button;
    }
}

bool XInput::isPressed(uint16_t button) {
    return (bool)((_inputReport.buttons & button) == button);
}

void XInput::setLeftThumb(int16_t x, int16_t y) {
    x = constrain(x, XBOX_STICK_MIN, XBOX_STICK_MAX);
    y = constrain(y, XBOX_STICK_MIN, XBOX_STICK_MAX);

    if(_inputReport.x != x || _inputReport.y != y) {
        _inputReport.x = (uint16_t)(x + 0x8000);
        _inputReport.y = (uint16_t)(y + 0x8000);
    }
}

void XInput::setRightThumb(int16_t z, int16_t rZ) {
    z = constrain(z, XBOX_STICK_MIN, XBOX_STICK_MAX);
    rZ = constrain(rZ, XBOX_STICK_MIN, XBOX_STICK_MAX);

    if(_inputReport.z != z || _inputReport.rz != rZ){
        _inputReport.z = (uint16_t)(z + 0x8000);
        _inputReport.rz = (uint16_t)(rZ + 0x8000);
    }
}

void XInput::setLeftTrigger(uint16_t value) {
    value = constrain(value, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);

    if (_inputReport.brake != value) {
        _inputReport.brake = value;
    }
}

void XInput::setRightTrigger(uint16_t value) {
    value = constrain(value, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);

    if (_inputReport.accelerator != value) {
        _inputReport.accelerator = value;
    }
}

void XInput::setTriggers(uint16_t left, uint16_t right) {
    left = constrain(left, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);
    right = constrain(right, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);

    if (_inputReport.brake != left || _inputReport.accelerator != right) {
        _inputReport.brake = left;
        _inputReport.accelerator = right;
    }
}

void XInput::pressDPadDirectionInternal(uint8_t direction) {

    // Avoid double presses
    if (!isDPadPressedInternal(direction))
    {
        _inputReport.hat = direction;
    }
}

void XInput::pressDPadDirection(XboxDpadFlags direction) {
    // Filter opposite button presses
    if((direction & (XboxDpadFlags::NORTH | XboxDpadFlags::SOUTH)) == (XboxDpadFlags::NORTH | XboxDpadFlags::SOUTH)){
        ESP_LOGD(LOG_TAG, "Filtering opposite button presses - up down");
        direction = (XboxDpadFlags)(direction ^ (uint8_t)(XboxDpadFlags::NORTH | XboxDpadFlags::SOUTH));
    }
    if((direction & (XboxDpadFlags::EAST | XboxDpadFlags::WEST)) == (XboxDpadFlags::EAST | XboxDpadFlags::WEST)){
        ESP_LOGD(LOG_TAG, "Filtering opposite button presses - left right");
        direction = (XboxDpadFlags)(direction ^ (uint8_t)(XboxDpadFlags::EAST | XboxDpadFlags::WEST));
    }

    pressDPadDirectionInternal(dPadDirectionToValue(direction));
}

void XInput::releaseDPad() {
    pressDPadDirectionInternal(XBOX_BUTTON_DPAD_NONE);
}

bool XInput::isDPadPressedInternal(uint8_t direction) {
    return _inputReport.hat == direction;
    //return (bool)((_inputReport.hat & direction) == direction);
}

bool XInput::isDPadPressed(XboxDpadFlags direction) {
    if (direction == XboxDpadFlags::NORTH) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_NORTH;
    } else if(direction == (XboxDpadFlags::NORTH & XboxDpadFlags::EAST)) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_NORTHEAST;
    } else if(direction == XboxDpadFlags::EAST) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_EAST;
    } else if(direction == (XboxDpadFlags::SOUTH & XboxDpadFlags::EAST)) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_SOUTHEAST;
    } else if(direction == XboxDpadFlags::SOUTH) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_SOUTH;
    } else if(direction == (XboxDpadFlags::SOUTH & XboxDpadFlags::WEST)) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_SOUTHWEST;
    } else if(direction == XboxDpadFlags::WEST) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_WEST;
    } else if(direction == (XboxDpadFlags::NORTH & XboxDpadFlags::WEST)) {
        return _inputReport.hat == XBOX_BUTTON_DPAD_NORTHWEST;
    }
    return false;
}


void XInput::pressShare() {
    // Avoid double presses
    if (!(_inputReport.share & XBOX_BUTTON_SHARE)) {
        _inputReport.share |= XBOX_BUTTON_SHARE;
    }
}

void XInput::releaseShare() {
    if (_inputReport.share & XBOX_BUTTON_SHARE) {
        _inputReport.share ^= XBOX_BUTTON_SHARE;
    }
}

void XInput::sendGamepadReport() {
    if (!this->_input || !this->isConnected()) {
        return;
    }

    this->_input->setValue((uint8_t*)&_inputReport, sizeof(_inputReport));
    this->_input->notify();
}