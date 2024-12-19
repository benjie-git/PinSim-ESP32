#include "xInput.h"
#include <Preferences.h>
#include <NimBLEDevice.h>

#define PREFS_NAME "XInput_BLE"

// Set MAX_CLIENTS to 1-4 to allow N computers/phones/quests to connect at once
//    If set to 1, uses blacklist logic to avoid reconnecting to the same device for 15 sec after a 
//    reconect, to allow another device to connect
#define MAX_CLIENTS 3

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

    void onWrite(NimBLECharacteristic* pCharacteristic) override
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
    int num_connected = 0;
    uint64_t lastAddr = 0;

    ServerCallbacks(XInput* xInput) : _xInput(xInput) {}

    void onConnect(NimBLEServer *server, ble_gap_conn_desc* desc)
    {
        if (MAX_CLIENTS == 1) {
            NimBLEAddress addr = NimBLEAddress(desc->peer_ota_addr);
            uint64_t addrInt = uint64_t(addr);
            if (this->lastAddr && addrInt == this->lastAddr) {
                printf("Reject lastAdr\n");
                server->disconnect(desc->conn_handle);
                return;
            }
        }

        server->updateConnParams(desc->conn_handle, 12, 24, 0, 80);
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
            this->_xInput->startAdvertising(false);
        }
    }

    void onDisconnect(NimBLEServer *server, ble_gap_conn_desc* desc)
    {
        NimBLEAddress addr = NimBLEAddress(desc->peer_ota_addr);
        uint64_t addrInt = uint64_t(addr);
        this->lastAddr = addrInt;

        this->num_connected--;
        if (this->num_connected == 0) {
            this->is_connected = false;
        }
        printf("Disconnected\n");

        this->_xInput->startAdvertising(true);
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
    this->_hid->manufacturer()->setValue(manufacturer);
    this->_hid->pnp(VENDOR_USB_SOURCE, XBOX_VENDOR_ID, XBOX_PRODUCT_ID, XBOX_BCD_DEVICE_ID);
    this->_hid->hidInfo(0x00, 0x01);
    this->_hid->reportMap((uint8_t*)Xbox_HIDDescriptor, sizeof(Xbox_HIDDescriptor));
    // printf("HID report size: %d", sizeof(Xbox_HIDDescriptor));

    this->_input = this->_hid->inputReport(XBOX_INPUT_REPORT_ID);
    this->_output = this->_hid->outputReport(XBOX_OUTPUT_REPORT_ID);
    this->_hidOutputCallbacks = new HIDOutputCallbacks(this);
    this->_output->setCallbacks(this->_hidOutputCallbacks);

    // Create device UUID
    NimBLEService *service = this->_server->getServiceByUUID(SERVICE_UUID_DEVICE_INFORMATION);
  
    // Create characteristics
    BLECharacteristic* characteristic_Model_Number = service->createCharacteristic(
      CHARACTERISTIC_UUID_MODEL_NUMBER,
      NIMBLE_PROPERTY::READ);
    static char* modelNumber = XBOX_MODEL;
    characteristic_Model_Number->setValue(modelNumber);
  
    BLECharacteristic* characteristic_Serial_Number = service->createCharacteristic(
      CHARACTERISTIC_UUID_SERIAL_NUMBER,
      NIMBLE_PROPERTY::READ);
    static char* serialNumber = XBOX_SERIAL;
    characteristic_Serial_Number->setValue(serialNumber);
  
    BLECharacteristic* characteristic_Firmware_Revision = service->createCharacteristic(
      CHARACTERISTIC_UUID_FIRMWARE_REVISION,
      NIMBLE_PROPERTY::READ);
    static char* firmwareRevision = XBOX_FW_VER;
    characteristic_Firmware_Revision->setValue(firmwareRevision);
  
    this->_hid->startServices();

    // Start BLE advertisement
    this->_advertising = this->_server->getAdvertising();
    this->_advertising->setAppearance(HID_GAMEPAD);
    this->_advertising->addServiceUUID(this->_hid->deviceInfo()->getUUID());
    this->_advertising->addServiceUUID(this->_hid->hidService()->getUUID());
    this->_advertising->addServiceUUID(this->_hid->batteryService()->getUUID());
    this->_advertising->setScanResponse(true);
    this->_advertising->start();

    this->_hid->setBatteryLevel(100);
    this->_hid->batteryLevel()->notify();
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
            this->_advertising->start(15, [this](NimBLEAdvertising *pAdvertising){ this->onAdvComplete(this->_advertising); });
            return;
        }
    }

    if (this->_serverCallbacks->num_connected >= MAX_CLIENTS) {
        printf("Skip Starting Advertising - already connected to MAX_CLIENTS\n");
        return;
    }

    printf("Start Advertising - No blacklist\n");
    this->_serverCallbacks->lastAddr = 0;
    if (this->_serverCallbacks->num_connected > 0) {
        // We already have a connection, so only advertise for 30s
        this->_advertising->start(30, [this](NimBLEAdvertising *advertising){ this->onAdvComplete(advertising); });
    }
    else {
        // No connections yet, so advertise indefinitely
        this->_advertising->start();
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