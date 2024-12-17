#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include <Callback.h>

class HIDOutputCallbacks;
class ServerCallbacks;


#define XBOX_BUTTON_A 0x01
#define XBOX_BUTTON_B 0x02
// UNUSED - 0x04
#define XBOX_BUTTON_X 0x08 
#define XBOX_BUTTON_Y 0x10
// UNUSED - 0x20
#define XBOX_BUTTON_LB 0x40
#define XBOX_BUTTON_RB 0x80
// UNUSED - 0x100
// UNUSED - 0x200
// UNUSED - 0x400
#define XBOX_BUTTON_SELECT 0x400
#define XBOX_BUTTON_START 0x800
#define XBOX_BUTTON_HOME 0x1000
#define XBOX_BUTTON_LS 0x2000
#define XBOX_BUTTON_RS 0x4000

// Select bitmask
// The share button lives in its own byte at the end of the input report
#define XBOX_BUTTON_SHARE 0x01

// Dpad values
#define XBOX_BUTTON_DPAD_NONE 0x00
#define XBOX_BUTTON_DPAD_NORTH 0x01
#define XBOX_BUTTON_DPAD_NORTHEAST 0x02
#define XBOX_BUTTON_DPAD_EAST 0x03
#define XBOX_BUTTON_DPAD_SOUTHEAST 0x04
#define XBOX_BUTTON_DPAD_SOUTH 0x05
#define XBOX_BUTTON_DPAD_SOUTHWEST 0x06
#define XBOX_BUTTON_DPAD_WEST 0x07
#define XBOX_BUTTON_DPAD_NORTHWEST 0x08

// Dpad bitflags
enum XboxDpadFlags : uint8_t {
    NONE = 0x00,
    NORTH = 0x01,
    EAST = 0x02,
    SOUTH = 0x04,
    WEST = 0x08
};

// Trigger range
#define XBOX_TRIGGER_MIN 0
#define XBOX_TRIGGER_MAX 1023

// Thumbstick range
#define XBOX_STICK_MIN -32768
#define XBOX_STICK_MAX 32767


#pragma pack(push, 1)
struct XboxGamepadInputReportData {
    uint16_t x = 0;             // Left joystick X
    uint16_t y = 0;             // Left joystick Y
    uint16_t z = 0;             // Right jostick X
    uint16_t rz = 0;            // Right joystick Y
    uint16_t brake = 0;         // 10 bits for brake (left trigger) + 6 bit padding (2 bytes)
    uint16_t accelerator = 0;   // 10 bits for accelerator (right trigger) + 6bit padding
    uint8_t  hat = 0x00;        // 4bits for hat switch (Dpad) + 4 bit padding (1 byte) 
    uint16_t buttons = 0x00;    // 15 * 1bit for buttons + 1 bit padding (2 bytes)
    uint8_t  share = 0x00;      // 1 bits for share/menu button + 7 bit padding (1 byte)
};
#pragma pack(pop)


struct XboxGamepadOutputReportData {
    uint8_t dcEnableActuators = 0x00;   // 4bits for DC Enable Actuators, 4bits padding
    uint8_t leftTriggerMagnitude = 0;
    uint8_t rightTriggerMagnitude = 0; 
    uint8_t weakMotorMagnitude = 0;
    uint8_t strongMotorMagnitude = 0; 
    uint8_t duration = 0;               // UNUSED
    uint8_t startDelay = 0;             // UNUSED
    uint8_t loopCount = 0;              // UNUSED

    constexpr XboxGamepadOutputReportData(uint64_t value = 0) noexcept : 
        dcEnableActuators((value & 0xFF)),
        leftTriggerMagnitude((value >> 8) & 0xFF),
        rightTriggerMagnitude((value >> 16) & 0xFF),
        weakMotorMagnitude((value >> 24) & 0xFF),
        strongMotorMagnitude((value >> 32) & 0xFF),
        duration((value >> 40) & 0xFF),
        startDelay((value >> 48) & 0xFF),
        loopCount((value >> 56) & 0xFF)
    {}
};


class XInput
{
public:
    void startServer(const char *device_name, const char *manufacturer);
    void startAdvertising(bool useBlackList);
    void onAdvComplete(NimBLEAdvertising *advertising);
    bool isConnected();
    bool isAdvertising();
    void clearPairedAddresses();

    Signal<XboxGamepadOutputReportData> onVibrate;

    void press(uint16_t button = XBOX_BUTTON_A);    
    void release(uint16_t button = XBOX_BUTTON_A); 
    bool isPressed(uint16_t button = XBOX_BUTTON_A);
    void setLeftThumb(int16_t x = 0, int16_t y = 0);
    void setRightThumb(int16_t z = 0, int16_t rZ = 0);
    void setLeftTrigger(uint16_t rX = 0);
    void setRightTrigger(uint16_t rY = 0);
    void setTriggers(uint16_t rX = 0, uint16_t rY = 0);
    void pressDPadDirection(XboxDpadFlags direction = XboxDpadFlags::NONE);
    void releaseDPad();
    bool isDPadPressed(XboxDpadFlags direction);
    void pressShare();
    void releaseShare();
    
    void sendGamepadReport();

private:
    NimBLEServer *_server;
    NimBLEAdvertising *_advertising;
    ServerCallbacks *_serverCallbacks;
    NimBLEHIDDevice *_hid;
    NimBLECharacteristic *_input;
    NimBLECharacteristic *_output;
    XboxGamepadInputReportData _inputReport;
    HIDOutputCallbacks *_hidOutputCallbacks;

    void pressDPadDirectionInternal(uint8_t direction = 0);
    bool isDPadPressedInternal(uint8_t direction = 0);
};
