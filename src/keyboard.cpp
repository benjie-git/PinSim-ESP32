#include "keyboard.h"
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include <Preferences.h>
#include "HIDTypes.h"

// Changed NimBLE-Arduino/src/nimble/nimble/host/store/config/src/ble_store_nvs.c
// to make this an external string, defined here, so we can change it!
// After reinstalling nimble, need to replace first NIMBLE_NVS_NAMESPACE line in ble_store_nvs.c with:
// const char* NIMBLE_NVS_NAMESPACE = "nimble_bond";
extern const char* NIMBLE_NVS_NAMESPACE;

#define MAX_CLIENTS 3

#define PREFS_NAME "Keyboard_BLE"
static Preferences preferences;
#define MAX_ADDRESSES 4
static std::vector<uint64_t> pairedAddresses;

// Report IDs:
#define KEYBOARD_ID 0x01

static const uint8_t _hidKBReportDescriptor[] = {
    USAGE_PAGE(1),      0x01,       // USAGE_PAGE (Generic Desktop Ctrls)
    USAGE(1),           0x06,       // USAGE (Keyboard)
    COLLECTION(1),      0x01,       // COLLECTION (Application)
    // -------------------------------------------------
    REPORT_ID(1),       KEYBOARD_ID,// REPORT_ID (1)

    // All INPUT items are grouped here
    // -------------------------------------------------
    USAGE_PAGE(1),      0x07,       // USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1),   0xE0,       // USAGE_MINIMUM (0xE0) - Left Control
    USAGE_MAXIMUM(1),   0xE7,       // USAGE_MAXIMUM (0xE7) - Right GUI
    LOGICAL_MINIMUM(1), 0x00,       // LOGICAL_MINIMUM (0)
    LOGICAL_MAXIMUM(1), 0x01,       // LOGICAL_MAXIMUM (1)
    REPORT_SIZE(1),     0x01,       // REPORT_SIZE (1)
    REPORT_COUNT(1),    0x08,       // REPORT_COUNT (8)
    HIDINPUT(1),        0x02,       // INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    REPORT_COUNT(1),    0x01,       // REPORT_COUNT (1) ; 1 byte (Reserved)
    REPORT_SIZE(1),     0x08,       // REPORT_SIZE (8)
    HIDINPUT(1),        0x01,       // INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)

    REPORT_COUNT(1),    0x06,       // REPORT_COUNT (6) ; 6 bytes (Keys)
    REPORT_SIZE(1),     0x08,       // REPORT_SIZE(8)
    LOGICAL_MINIMUM(1), 0x00,       // LOGICAL_MINIMUM(0)
    LOGICAL_MAXIMUM(1), 0x65,       // LOGICAL_MAXIMUM(0x65) ; 101 keys
    USAGE_PAGE(1),      0x07,       // USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1),   0x00,       // USAGE_MINIMUM (0)
    USAGE_MAXIMUM(1),   0x65,       // USAGE_MAXIMUM (0x65)
    HIDINPUT(1),        0x00,       // INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)

    // All OUTPUT items are grouped here
    // -------------------------------------------------
    REPORT_COUNT(1),    0x05,       // REPORT_COUNT (5) ; 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1),     0x01,       // REPORT_SIZE (1)
    USAGE_PAGE(1),      0x08,       // USAGE_PAGE (LEDs)
    USAGE_MINIMUM(1),   0x01,       // USAGE_MINIMUM (0x01) ; Num Lock
    USAGE_MAXIMUM(1),   0x05,       // USAGE_MAXIMUM (0x05) ; Kana
    HIDOUTPUT(1),       0x02,       // OUTPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    REPORT_COUNT(1),    0x01,       // REPORT_COUNT (1) ; 3 bits (Padding)
    REPORT_SIZE(1),     0x03,       // REPORT_SIZE (3)
    HIDOUTPUT(1),       0x01,       // OUTPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    END_COLLECTION(0)               // END_COLLECTION
};

class KBServerCallbacks : public NimBLEServerCallbacks
{
public:
    bool is_connected = false;

    KBServerCallbacks(BLEKeyboard *k) : _keyboard(k) {}

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
            this->_keyboard->saveWhitelist();
            printf("Added paired address         (%s) (%s)\n", connInfo.getAddress().toString().c_str(), connInfo.getIdAddress().toString().c_str());
        }
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override
    {
        if (!connInfo.isEncrypted()) {
            printf("Auth failed                  (%s)\n", connInfo.getIdAddress().toString().c_str());
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            this->_keyboard->startAdvertising();
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

        this->_keyboard->startAdvertising();
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
        this->_keyboard->startAdvertising();
    }

private:
    BLEKeyboard* _keyboard = nullptr;
};


void BLEKeyboard::loadWhitelist()
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

void BLEKeyboard::saveWhitelist()
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

void BLEKeyboard::clearWhitelistInternal()
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

void BLEKeyboard::begin(const std::string& deviceName,
						const std::string& manufacturer,
						CommandCallback_t commandCallback)
{
	NIMBLE_NVS_NAMESPACE = "nim_bond_kb";

    NimBLEDevice::init(deviceName);
    NimBLEDevice::setSecurityAuth(true, true, true);

    this->_inputReportDirty = true;
    this->_server = NimBLEDevice::createServer();
    this->_serverCallbacks = new KBServerCallbacks(this);
    this->_server->setCallbacks(this->_serverCallbacks);

    this->_hid = new NimBLEHIDDevice(this->_server);
    this->_hid->setManufacturer(manufacturer);
    this->_hid->setPnp(0x02, 0x05ac, 0x820a, 0x0210);
    this->_hid->setHidInfo(0x00, 0x01);
    this->_hid->setReportMap((uint8_t*)_hidKBReportDescriptor, sizeof(_hidKBReportDescriptor));

    this->_inputKeyboard = this->_hid->getInputReport(KEYBOARD_ID);
    this->_inputKeyboard->setValue((uint8_t*)&_keyReport, sizeof(_keyReport));

	this->_commandHandler = new CommandHandler(this->_server, commandCallback);

    this->_hid->setBatteryLevel(100);

    this->_hid->startServices();

    // Start BLE advertisement
    this->_advertising = this->_server->getAdvertising();
    NimBLEAdvertisementData ad = this->_advertising->getAdvertisementData();
    ad.setName(std::string(deviceName));
    this->_advertising->setAdvertisementData(ad);
    this->_advertising->setAppearance(HID_KEYBOARD);

    NimBLEAdvertisementData sr = this->_advertising->getScanData();
    sr.addServiceUUID(this->_hid->getHidService()->getUUID());
    sr.addServiceUUID(COMMAND_SERVICE_ID);
    this->_advertising->setScanResponseData(sr);

    this->_advertising->enableScanResponse(true);
    this->_advertising->setAdvertisingCompleteCallback([&](NimBLEAdvertising *advertising) { this->onAdvComplete(advertising); });
    this->_allowNewConnections = false;

    this->loadWhitelist();
}

bool BLEKeyboard::isConnected()
{
    return this->_serverCallbacks->is_connected;
}

bool BLEKeyboard::isAdvertising()
{
    return this->_advertising->isAdvertising();
}

bool BLEKeyboard::isAdvertisingNewDevices()
{
    return this->_advertising->isAdvertising() && this->_allowNewConnections;
}

void BLEKeyboard::startAdvertising()
{
    if (this->_server->getConnectedCount() >= MAX_CLIENTS) {
        printf("Skip Starting Advertising - already connected to MAX_CLIENTS\n");
        return;
    }

    uint cnt = pairedAddresses.size();
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
        this->_advertising->stop();
        this->_advertising->setScanFilter(false, false);
        this->_advertising->start(30000);  // 30 sec
    }
}

void BLEKeyboard::allowNewConnections(bool allow)
{
    this->_allowNewConnections = allow;
}

void BLEKeyboard::onAdvComplete(NimBLEAdvertising *advertising)
{
    printf("Advertising stopped\n");

    // Restart advertising with whitelist filter
    this->_allowNewConnections = false;
    this->startAdvertising();
}

uint BLEKeyboard::getPairCount()
{
    return pairedAddresses.size();
}

void BLEKeyboard::clearWhitelist()
{
    this->clearWhitelistInternal();
}

void BLEKeyboard::send_command(const uint8_t* data, const uint8_t length)
{
    this->_commandHandler->send_command(data, length);
}

extern
const uint8_t _asciimap[128] PROGMEM;

#define SHIFT 0x80
const uint8_t _asciimap[128] =
{
	0x00,             // NUL
	0x00,             // SOH
	0x00,             // STX
	0x00,             // ETX
	0x00,             // EOT
	0x00,             // ENQ
	0x00,             // ACK
	0x00,             // BEL
	0x2a,			// BS	Backspace
	0x2b,			// TAB	Tab
	0x28,			// LF	Enter
	0x00,             // VT
	0x00,             // FF
	0x00,             // CR
	0x00,             // SO
	0x00,             // SI
	0x00,             // DEL
	0x00,             // DC1
	0x00,             // DC2
	0x00,             // DC3
	0x00,             // DC4
	0x00,             // NAK
	0x00,             // SYN
	0x00,             // ETB
	0x00,             // CAN
	0x00,             // EM
	0x00,             // SUB
	0x00,             // ESC
	0x00,             // FS
	0x00,             // GS
	0x00,             // RS
	0x00,             // US

	0x2c,		   //  ' '
	0x1e|SHIFT,	   // !
	0x34|SHIFT,	   // "
	0x20|SHIFT,    // #
	0x21|SHIFT,    // $
	0x22|SHIFT,    // %
	0x24|SHIFT,    // &
	0x34,          // '
	0x26|SHIFT,    // (
	0x27|SHIFT,    // )
	0x25|SHIFT,    // *
	0x2e|SHIFT,    // +
	0x36,          // ,
	0x2d,          // -
	0x37,          // .
	0x38,          // /
	0x27,          // 0
	0x1e,          // 1
	0x1f,          // 2
	0x20,          // 3
	0x21,          // 4
	0x22,          // 5
	0x23,          // 6
	0x24,          // 7
	0x25,          // 8
	0x26,          // 9
	0x33|SHIFT,      // :
	0x33,          // ;
	0x36|SHIFT,      // <
	0x2e,          // =
	0x37|SHIFT,      // >
	0x38|SHIFT,      // ?
	0x1f|SHIFT,      // @
	0x04|SHIFT,      // A
	0x05|SHIFT,      // B
	0x06|SHIFT,      // C
	0x07|SHIFT,      // D
	0x08|SHIFT,      // E
	0x09|SHIFT,      // F
	0x0a|SHIFT,      // G
	0x0b|SHIFT,      // H
	0x0c|SHIFT,      // I
	0x0d|SHIFT,      // J
	0x0e|SHIFT,      // K
	0x0f|SHIFT,      // L
	0x10|SHIFT,      // M
	0x11|SHIFT,      // N
	0x12|SHIFT,      // O
	0x13|SHIFT,      // P
	0x14|SHIFT,      // Q
	0x15|SHIFT,      // R
	0x16|SHIFT,      // S
	0x17|SHIFT,      // T
	0x18|SHIFT,      // U
	0x19|SHIFT,      // V
	0x1a|SHIFT,      // W
	0x1b|SHIFT,      // X
	0x1c|SHIFT,      // Y
	0x1d|SHIFT,      // Z
	0x2f,          // [
	0x31,          // bslash
	0x30,          // ]
	0x23|SHIFT,    // ^
	0x2d|SHIFT,    // _
	0x35,          // `
	0x04,          // a
	0x05,          // b
	0x06,          // c
	0x07,          // d
	0x08,          // e
	0x09,          // f
	0x0a,          // g
	0x0b,          // h
	0x0c,          // i
	0x0d,          // j
	0x0e,          // k
	0x0f,          // l
	0x10,          // m
	0x11,          // n
	0x12,          // o
	0x13,          // p
	0x14,          // q
	0x15,          // r
	0x16,          // s
	0x17,          // t
	0x18,          // u
	0x19,          // v
	0x1a,          // w
	0x1b,          // x
	0x1c,          // y
	0x1d,          // z
	0x2f|SHIFT,    // {
	0x31|SHIFT,    // |
	0x30|SHIFT,    // }
	0x35|SHIFT,    // ~
	0				// DEL
};

// press() adds the specified key (printing, non-printing, or modifier)
// to the persistent key report and sends the report.  Because of the way
// USB HID works, the host acts like the key remains pressed until we
// call release(), releaseAll(), or otherwise clear the report and resend.
size_t BLEKeyboard::press(uint8_t k)
{
	uint8_t i;
	if (k >= 136) {			// it's a non-printing key (not a modifier)
		k = k - 136;
	} else if (k >= 128) {	// it's a modifier key
		_keyReport.modifiers |= (1<<(k-128));
		_inputReportDirty = true;
		k = 0;
	} else {				// it's a printing key
		k = pgm_read_byte(_asciimap + k);
		if (!k) {
			return 0;
		}
		if (k & 0x80) {						// it's a capital letter or other character reached with shift
			_keyReport.modifiers |= 0x02;	// the left shift modifier
			_inputReportDirty = true;
			k &= 0x7F;
		}
	}

	// Add k to the key report only if it's not already present
	// and if there is an empty slot.
	if (_keyReport.keys[0] != k && _keyReport.keys[1] != k &&
		_keyReport.keys[2] != k && _keyReport.keys[3] != k &&
		_keyReport.keys[4] != k && _keyReport.keys[5] != k) {

		for (i=0; i<6; i++) {
			if (_keyReport.keys[i] == 0x00) {
				_keyReport.keys[i] = k;
				_inputReportDirty = true;
				break;
			}
		}
		if (i == 6) {
			return 0;
		}
	}
	return 1;
}

// release() takes the specified key out of the persistent key report and
// sends the report.  This tells the OS the key is no longer pressed and that
// it shouldn't be repeated any more.
size_t BLEKeyboard::release(uint8_t k)
{
	uint8_t i;
	if (k >= 136) {			// it's a non-printing key (not a modifier)
		k = k - 136;
	} else if (k >= 128) {	// it's a modifier key
		_keyReport.modifiers &= ~(1<<(k-128));
		_inputReportDirty = true;
		k = 0;
	} else {				// it's a printing key
		k = pgm_read_byte(_asciimap + k);
		if (!k) {
			return 0;
		}
		if (k & 0x80) {							// it's a capital letter or other character reached with shift
			_keyReport.modifiers &= ~(0x02);	// the left shift modifier
			_inputReportDirty = true;
			k &= 0x7F;
		}
	}

	// Test the key report to see if k is present.  Clear it if it exists.
	// Check all positions in case the key is present more than once (which it shouldn't be)
	for (i=0; i<6; i++) {
		if (0 != k && _keyReport.keys[i] == k) {
			_keyReport.keys[i] = 0x00;
			_inputReportDirty = true;
		}
	}

	return 1;
}


size_t BLEKeyboard::set(uint8_t k, bool pressed)
{
	if (pressed)
		return press(k);
	else
		return release(k);
}


void BLEKeyboard::releaseAll(void)
{
	_keyReport.keys[0] = 0;
	_keyReport.keys[1] = 0;
	_keyReport.keys[2] = 0;
	_keyReport.keys[3] = 0;
	_keyReport.keys[4] = 0;
	_keyReport.keys[5] = 0;
	_keyReport.modifiers = 0;
	_inputReportDirty = true;
}


void BLEKeyboard::sendReport()
{
	if (this->isConnected() && _inputReportDirty) {
		this->_inputKeyboard->setValue((uint8_t*)&_keyReport, sizeof(KeyReport));
		this->_inputKeyboard->notify();
		_inputReportDirty = false;
	}
}
