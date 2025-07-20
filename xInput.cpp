#include <string>
#include "xInput.h"
#include <Preferences.h>
#include <NimBLEDevice.h>

#define MAX_CLIENTS 4



///////////////////////////////////////////
// Manage persistent Whitelist

#define PREFS_NAME "XInput_BLE"
static Preferences preferences;
#define MAX_ADDRESSES 4
std::vector<uint64_t> pairedAddresses;

void loadWhitelist()
{
    pairedAddresses.reserve(MAX_ADDRESSES+1);
    
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
                NimBLEDevice::whiteListAdd(NimBLEAddress(*addrInt, BLE_ADDR_PUBLIC));
                pos += sizeof(uint64_t);
            }
            free(bytes);
        }
    }
}

void saveWhitelist()
{
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

void clearWhitelistInternal()
{
    printf("--- Clear whitelist and delete all bonds.\n");
    NimBLEDevice::deleteAllBonds();

    // Iterate over all whitelist addresses, removing each one
    for (int i=0; i< NimBLEDevice::getWhiteListCount(); i++) {
        NimBLEDevice::whiteListRemove(NimBLEDevice::getWhiteListAddress(0));
    }

    pairedAddresses.clear();
    saveWhitelist();
    preferences.remove("pairedAddresses");
}



///////////////////////////////////////////
// BLE Callbacks / Event Handlers

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

    ServerCallbacks(XInput* xInput) : _xInput(xInput) {}

    void onConnect(NimBLEServer *server, NimBLEConnInfo& connInfo) override
    {
        server->updateConnParams(connInfo.getConnHandle(), 6, 12, 0, 160);
        NimBLEDevice::startSecurity(connInfo.getConnHandle());
        printf("Connected %d                  (%s)\n", server->getConnectedCount(),
                                                       connInfo.getIdAddress().toString().c_str());
    }

    void saveIdentity(NimBLEConnInfo& connInfo)
    {
        NimBLEAddress addr = NimBLEAddress(connInfo.getIdAddress());
        uint64_t addrInt = uint64_t(addr);
        if (std::find(pairedAddresses.begin(), pairedAddresses.end(), addrInt) == pairedAddresses.end()) {
            pairedAddresses.push_back(addrInt);
            NimBLEDevice::whiteListAdd(connInfo.getIdAddress());
            // this->_xInput->allowNewConnections(false);
            saveWhitelist();
            printf("Added paired address         (%s) (%s)\n", connInfo.getAddress().toString().c_str(), connInfo.getIdAddress().toString().c_str());
        }
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override
    {
        if (!connInfo.isEncrypted()) {
            printf("Auth failed                  (%s)\n", connInfo.getIdAddress().toString().c_str());
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            this->_xInput->startAdvertising();
            return;
        }

        uint8_t t = connInfo.getIdAddress().getType();

        if ((t & 0x01) == 0) {
            // If this is not a random address, save it now
            // If it is random, save it later, when onIdentity() is called
            saveIdentity(connInfo);
        }

        this->is_connected = true;
        printf("Auth complete                (%s)\n", connInfo.getIdAddress().toString().c_str());
        
        this->_xInput->startAdvertising();
    }

    void onIdentity(NimBLEConnInfo& connInfo) override
    {
        saveIdentity(connInfo);
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo& connInfo, int reason) override
    {
        if (server->getConnectedCount() == 0) {
            this->is_connected = false;
        }
        printf("Disconnected %d               (%s)\n", server->getConnectedCount(), 
            connInfo.getIdAddress().toString().c_str());
        this->_xInput->startAdvertising();
    }

private:
    XInput *_xInput = NULL;
};



///////////////////////////////////////////
// Manage the BLE Server / Advertising

void XInput::startServer(const char *device_name, const char *manufacturer)
{
    NimBLEDevice::init(device_name);
    NimBLEDevice::setSecurityAuth(true, false, false);

    this->_inputReportDirty = true;
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
    NimBLEAdvertisementData ad = this->_advertising->getAdvertisementData();
    ad.setName(std::string(device_name));
    this->_advertising->setAdvertisementData(ad);
    this->_advertising->setAppearance(HID_GAMEPAD);
    this->_advertising->addServiceUUID(this->_hid->getHidService()->getUUID());
    this->_advertising->enableScanResponse(true);
    this->_advertising->setAdvertisingCompleteCallback([&](NimBLEAdvertising *advertising) { this->onAdvComplete(advertising); });
    this->_advertising->setScanFilter(true, true);
    this->_allowNewConnections = false;

    loadWhitelist();
}

bool XInput::isConnected()
{
    return this->_serverCallbacks->is_connected;
}

bool XInput::isAdvertising()
{
    return this->_advertising->isAdvertising();
}

bool XInput::isAdvertisingNewDevices()
{
    return this->_advertising->isAdvertising() && this->_allowNewConnections;
}

void XInput::startAdvertising()
{
    if (this->_server->getConnectedCount() >= MAX_CLIENTS) {
        printf("Skip Starting Advertising - already connected to MAX_CLIENTS\n");
        return;
    }
    
    int cnt = pairedAddresses.size();
    if (this->_allowNewConnections == false && cnt == 0) {
        this->_allowNewConnections = true;
    }

    if (this->_allowNewConnections == false) {
        printf("Start Advertising (whitelist only)\n");
        this->_advertising->stop();
        this->_advertising->setScanFilter(true, true);
        this->_advertising->start();
    }
    else {
        printf("Start Advertising (allow all - 30sec)\n");
        this->_advertising->stop();  // 30 sec
        this->_advertising->setScanFilter(false, false);
        this->_advertising->start(30000);  // 30 sec
    }
}

void XInput::allowNewConnections(bool allow)
{
    this->_allowNewConnections = allow;
}

void XInput::onAdvComplete(NimBLEAdvertising *advertising)
{
    printf("Advertising stopped\n");

    // Restart advertising with whitelist filter
    this->startAdvertising();
}

void XInput::clearWhitelist()
{
    clearWhitelistInternal();
}




///////////////////////////////////////////
// Handle button presses / stick movements

void XInput::press(uint16_t button)
{
    // Avoid double presses
    if (!isPressed(button)) {
        _inputReport.buttons |= button;
        _inputReportDirty = true;
    }
}

void XInput::release(uint16_t button)
{
    // Avoid double presses
    if (isPressed(button)) {   
        _inputReport.buttons ^= button;
        _inputReportDirty = true;
    }
}

bool XInput::isPressed(uint16_t button)
{
    return (bool)((_inputReport.buttons & button) == button);
}

void XInput::setLeftThumb(int16_t x, int16_t y)
{
    x = constrain(x, XBOX_STICK_MIN, XBOX_STICK_MAX);
    y = constrain(y, XBOX_STICK_MIN, XBOX_STICK_MAX);
    
    uint16_t ux = (uint16_t)(x + 0x8000);
    uint16_t uy = (uint16_t)(y + 0x8000);

    if(_inputReport.x != ux || _inputReport.y != uy) {
        _inputReport.x = ux;
        _inputReport.y = uy;
        _inputReportDirty = true;
    }
}

void XInput::setRightThumb(int16_t z, int16_t rZ)
{
    z = constrain(z, XBOX_STICK_MIN, XBOX_STICK_MAX);
    rZ = constrain(rZ, XBOX_STICK_MIN, XBOX_STICK_MAX);
    
    uint16_t uz = (uint16_t)(z + 0x8000);
    uint16_t urZ = (uint16_t)(rZ + 0x8000);

    if(_inputReport.z != uz || _inputReport.rz != urZ){
        _inputReport.z = uz;
        _inputReport.rz = urZ;
        _inputReportDirty = true;
    }
}

void XInput::setLeftTrigger(uint16_t value)
{
    value = constrain(value, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);

    if (_inputReport.brake != value) {
        _inputReport.brake = value;
        _inputReportDirty = true;
    }
}

void XInput::setRightTrigger(uint16_t value)
{
    value = constrain(value, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);

    if (_inputReport.accelerator != value) {
        _inputReport.accelerator = value;
        _inputReportDirty = true;
    }
}

void XInput::setTriggers(uint16_t left, uint16_t right)
{
    left = constrain(left, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);
    right = constrain(right, XBOX_TRIGGER_MIN, XBOX_TRIGGER_MAX);

    if (_inputReport.brake != left || _inputReport.accelerator != right) {
        _inputReport.brake = left;
        _inputReport.accelerator = right;
        _inputReportDirty = true;
    }
}

static uint8_t dPadDirectionToValue(XboxDpadFlags direction)
{
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

void XInput::pressDPadDirectionInternal(uint8_t direction)
{

    // Avoid double presses
    if (!isDPadPressedInternal(direction))
    {
        _inputReport.hat = direction;
        _inputReportDirty = true;
    }
}

void XInput::pressDPadDirection(XboxDpadFlags direction)
{
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

void XInput::releaseDPad() 
{
    pressDPadDirectionInternal(XBOX_BUTTON_DPAD_NONE);
}

bool XInput::isDPadPressedInternal(uint8_t direction)
{
    return _inputReport.hat == direction;
    //return (bool)((_inputReport.hat & direction) == direction);
}

bool XInput::isDPadPressed(XboxDpadFlags direction)
{
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

void XInput::pressShare()
{
    // Avoid double presses
    if (!(_inputReport.share & XBOX_BUTTON_SHARE)) {
        _inputReport.share |= XBOX_BUTTON_SHARE;
        _inputReportDirty = true;
    }
}

void XInput::releaseShare()
{
    if (_inputReport.share & XBOX_BUTTON_SHARE) {
        _inputReport.share ^= XBOX_BUTTON_SHARE;
        _inputReportDirty = true;
    }
}

void XInput::sendGamepadReport()
{
    if (!this->_input || !this->isConnected()) {
        return;
    }

    if (_inputReportDirty) {
        this->_input->setValue((uint8_t*)&_inputReport, sizeof(_inputReport));
        this->_input->notify();
        _inputReportDirty = false;
    }
}
