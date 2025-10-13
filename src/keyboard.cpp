#include "keyboard.h"
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include "HIDTypes.h"

// Report IDs:
#define KEYBOARD_ID 0x01

static const uint8_t _hidKBReportDescriptor[] = {
	USAGE_PAGE(1),      0x01,          // USAGE_PAGE (Generic Desktop Ctrls)
	USAGE(1),           0x06,          // USAGE (Keyboard)
	COLLECTION(1),      0x01,          // COLLECTION (Application)
	// ------------------------------------------------- Keyboard
	REPORT_ID(1),       KEYBOARD_ID,   //   REPORT_ID (1)
	USAGE_PAGE(1),      0x07,          //   USAGE_PAGE (Kbrd/Keypad)
	USAGE_MINIMUM(1),   0xE0,          //   USAGE_MINIMUM (0xE0)
	USAGE_MAXIMUM(1),   0xE7,          //   USAGE_MAXIMUM (0xE7)
	LOGICAL_MINIMUM(1), 0x00,          //   LOGICAL_MINIMUM (0)
	LOGICAL_MAXIMUM(1), 0x01,          //   Logical Maximum (1)
	REPORT_SIZE(1),     0x01,          //   REPORT_SIZE (1)
	REPORT_COUNT(1),    0x08,          //   REPORT_COUNT (8)
	HIDINPUT(1),        0x02,          //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	REPORT_COUNT(1),    0x01,          //   REPORT_COUNT (1) ; 1 byte (Reserved)
	REPORT_SIZE(1),     0x08,          //   REPORT_SIZE (8)
	HIDINPUT(1),        0x01,          //   INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
	REPORT_COUNT(1),    0x05,          //   REPORT_COUNT (5) ; 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
	REPORT_SIZE(1),     0x01,          //   REPORT_SIZE (1)
	USAGE_PAGE(1),      0x08,          //   USAGE_PAGE (LEDs)
	USAGE_MINIMUM(1),   0x01,          //   USAGE_MINIMUM (0x01) ; Num Lock
	USAGE_MAXIMUM(1),   0x05,          //   USAGE_MAXIMUM (0x05) ; Kana
	HIDOUTPUT(1),       0x02,          //   OUTPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	REPORT_COUNT(1),    0x01,          //   REPORT_COUNT (1) ; 3 bits (Padding)
	REPORT_SIZE(1),     0x03,          //   REPORT_SIZE (3)
	HIDOUTPUT(1),       0x01,          //   OUTPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	REPORT_COUNT(1),    0x06,          //   REPORT_COUNT (6) ; 6 bytes (Keys)
	REPORT_SIZE(1),     0x08,          //   REPORT_SIZE(8)
	LOGICAL_MINIMUM(1), 0x00,          //   LOGICAL_MINIMUM(0)
	LOGICAL_MAXIMUM(1), 0x65,          //   LOGICAL_MAXIMUM(0x65) ; 101 keys
	USAGE_PAGE(1),      0x07,          //   USAGE_PAGE (Kbrd/Keypad)
	USAGE_MINIMUM(1),   0x00,          //   USAGE_MINIMUM (0)
	USAGE_MAXIMUM(1),   0x65,          //   USAGE_MAXIMUM (0x65)
	HIDINPUT(1),        0x00,          //   INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
	END_COLLECTION(0),                 // END_COLLECTION
};


class HIDKBOutputCallbacks : public NimBLECharacteristicCallbacks
{
public:
    HIDKBOutputCallbacks(BLEKeyboard *k) : _keyboard(k) {}

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override
    {
    	// HID Report characteristic (for LED status)
        uint8_t ledData = pCharacteristic->getValue<uint8_t>();
        _keyboard->sendLedStatus(ledData);
    }
private:
    BLEKeyboard* _keyboard;
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

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override
    {
        if (!connInfo.isEncrypted()) {
            printf("Auth failed                  (%s)\n", connInfo.getIdAddress().toString().c_str());
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            this->_keyboard->startAdvertising();
            return;
        }

        this->is_connected = true;
        printf("Auth complete                (%s)\n", connInfo.getIdAddress().toString().c_str());
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
    BLEKeyboard* _keyboard;
};


BLEKeyboard::BLEKeyboard() : _server(nullptr),
							 _advertising(nullptr),
							 _hid(nullptr),
                             _inputKeyboard(nullptr),
                             _outputKeyboard(nullptr),
							 _keyReport(),
                             _serverCallbacks(nullptr),
                             _hidOutputCallbacks(nullptr),
                             _inputReportDirty(false),
							 _ledStatusCallback(nullptr) {}

void BLEKeyboard::begin(const std::string& deviceName,
						const std::string& manufacturer,
						void (*ledCallback)(uint8_t))
{
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


    this->_outputKeyboard = this->_hid->getOutputReport(KEYBOARD_ID);
    this->_hidOutputCallbacks = new HIDKBOutputCallbacks(this);
    this->_outputKeyboard->setCallbacks(this->_hidOutputCallbacks);
    this->_ledStatusCallback = ledCallback;

    this->_hid->setBatteryLevel(100);

    this->_hid->startServices();

    // Start BLE advertisement
    this->_advertising = this->_server->getAdvertising();
    NimBLEAdvertisementData ad = this->_advertising->getAdvertisementData();
    ad.setName(std::string(deviceName));
    this->_advertising->setAdvertisementData(ad);
    this->_advertising->setAppearance(HID_KEYBOARD);
    this->_advertising->addServiceUUID(this->_hid->getHidService()->getUUID());
    this->_advertising->enableScanResponse(true);
	printf("KB is set up\n");
}

bool BLEKeyboard::isAdvertising()
{
	return this->_advertising->isAdvertising();
}

void BLEKeyboard::startAdvertising()
{
	printf("KB is advertising\n");
	this->_advertising->start();
}

void BLEKeyboard::sendLedStatus(uint8_t ledStatus) {
    if (_ledStatusCallback) {
        _ledStatusCallback(ledStatus);
    }
}

bool BLEKeyboard::isConnected()
{
    return this->_serverCallbacks->is_connected;
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
