#include "xInput.h"
#include <Preferences.h>
#include <NimBLEDevice.h>


#define SERVICE_UUID_DEVICE_INFORMATION        "180A"      // Service - Device information

#define CHARACTERISTIC_UUID_SYSTEM_ID          "2A23"      // Characteristic - System ID 0x2A23
#define CHARACTERISTIC_UUID_MODEL_NUMBER       "2A24"      // Characteristic - Model Number String - 0x2A24
#define CHARACTERISTIC_UUID_SOFTWARE_REVISION  "2A28"      // Characteristic - Software Revision String - 0x2A28
#define CHARACTERISTIC_UUID_SERIAL_NUMBER      "2A25"      // Characteristic - Serial Number String - 0x2A25
#define CHARACTERISTIC_UUID_FIRMWARE_REVISION  "2A26"      // Characteristic - Firmware Revision String - 0x2A26
#define CHARACTERISTIC_UUID_HARDWARE_REVISION  "2A27"      // Characteristic - Hardware Revision String - 0x2A27

// Product: latest Xbox series X wireless controller
#define VENDOR_USB_SOURCE 0x02
#define XBOX_VENDOR_ID 0x045E
#define XBOX_1914_PRODUCT_ID 0x0B13
#define XBOX_1914_BCD_DEVICE_ID 0x0503
#define XBOX_INPUT_REPORT_ID 0x01
#define XBOX_OUTPUT_REPORT_ID 0x03

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

// Select bitmask
// The share button lives in its own byte at the end of the input report
#define XBOX_BUTTON_SHARE 0x01


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


static const uint8_t XboxOneS_1914_HIDDescriptor[] 
{
    0x05, 0x01,                 //(GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
    0x09, 0x05,                 //(LOCAL)  USAGE              0x00010005 Game Pad (Application Collection)  
    0xA1, 0x01,                 //(MAIN)   COLLECTION         0x01 Application (Usage=0x00010005: Page=Generic Desktop Page, Usage=Game Pad, Type=Application Collection)
    0x85, XBOX_INPUT_REPORT_ID, //  (GLOBAL) REPORT_ID          0x01 (1)  
    0x09, 0x01,                 //  (LOCAL)  USAGE              0x00010001 Pointer (Physical Collection)  
    0xA1, 0x00,                 //  (MAIN)   COLLECTION         0x00 Physical (Usage=0x00010001: Page=Generic Desktop Page, Usage=Pointer, Type=Physical Collection)
    0x09, 0x30,                 //    (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
    0x09, 0x31,                 //    (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0)  <-- Info: Consider replacing 15 00 with 14
    0x27, 0xFF, 0xFF, 0x00, 0x00,//    (GLOBAL) LOGICAL_MAXIMUM    0x0000FFFF (65535)  
    0x95, 0x02,                 //    (GLOBAL) REPORT_COUNT       0x02 (2) Number of fields  
    0x75, 0x10,                 //    (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
    0x81, 0x02,                 //    (MAIN)   INPUT              0x00000002 (2 fields x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0xC0,                       //  (MAIN)   END_COLLECTION     Physical 
    0x09, 0x01,                 //  (LOCAL)  USAGE              0x00010001 Pointer (Physical Collection)  
    0xA1, 0x00,                 //  (MAIN)   COLLECTION         0x00 Physical (Usage=0x00010001: Page=Generic Desktop Page, Usage=Pointer, Type=Physical Collection)
    0x09, 0x32,                 //    (LOCAL)  USAGE              0x00010032 Z (Dynamic Value)  
    0x09, 0x35,                 //    (LOCAL)  USAGE              0x00010035 Rz (Dynamic Value)  
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x27, 0xFF, 0xFF, 0x00, 0x00,//    (GLOBAL) LOGICAL_MAXIMUM    0x0000FFFF (65535) <-- Redundant: LOGICAL_MAXIMUM is already 65535 
    0x95, 0x02,                 //    (GLOBAL) REPORT_COUNT       0x02 (2) Number of fields <-- Redundant: REPORT_COUNT is already 2 
    0x75, 0x10,                 //    (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field <-- Redundant: REPORT_SIZE is already 16 
    0x81, 0x02,                 //    (MAIN)   INPUT              0x00000002 (2 fields x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0xC0,                       //  (MAIN)   END_COLLECTION     Physical 
    0x05, 0x02,                 //  (GLOBAL) USAGE_PAGE         0x0002 Simulation Controls Page 
    0x09, 0xC5,                 //  (LOCAL)  USAGE              0x000200C5 Brake (Dynamic Value)  
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x03,           //  (GLOBAL) LOGICAL_MAXIMUM    0x03FF (1023)  
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
    0x75, 0x0A,                 //  (GLOBAL) REPORT_SIZE        0x0A (10) Number of bits per field  
    0x81, 0x02,                 //  (MAIN)   INPUT              0x00000002 (1 field x 10 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x06,                 //  (GLOBAL) REPORT_SIZE        0x06 (6) Number of bits per field  
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x81, 0x03,                 //  (MAIN)   INPUT              0x00000003 (1 field x 6 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x05, 0x02,                 //  (GLOBAL) USAGE_PAGE         0x0002 Simulation Controls Page <-- Redundant: USAGE_PAGE is already 0x0002
    0x09, 0xC4,                 //  (LOCAL)  USAGE              0x000200C4 Accelerator (Dynamic Value)  
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x03,           //  (GLOBAL) LOGICAL_MAXIMUM    0x03FF (1023)  
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x75, 0x0A,                 //  (GLOBAL) REPORT_SIZE        0x0A (10) Number of bits per field  
    0x81, 0x02,                 //  (MAIN)   INPUT              0x00000002 (1 field x 10 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x06,                 //  (GLOBAL) REPORT_SIZE        0x06 (6) Number of bits per field  
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x81, 0x03,                 //  (MAIN)   INPUT              0x00000003 (1 field x 6 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x05, 0x01,                 //  (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
    0x09, 0x39,                 //  (LOCAL)  USAGE              0x00010039 Hat switch (Dynamic Value)  
    0x15, 0x01,                 //  (GLOBAL) LOGICAL_MINIMUM    0x01 (1)  
    0x25, 0x08,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x08 (8)  
    0x35, 0x00,                 //  (GLOBAL) PHYSICAL_MINIMUM   0x00 (0)  <-- Info: Consider replacing 35 00 with 34
    0x46, 0x3B, 0x01,           //  (GLOBAL) PHYSICAL_MAXIMUM   0x013B (315)  
    0x66, 0x14, 0x00,           //  (GLOBAL) UNIT               0x0014 Rotation in degrees [1° units] (4=System=English Rotation, 1=Rotation=Degrees)  <-- Info: Consider replacing 66 1400 with 65 14
    0x75, 0x04,                 //  (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field  
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x81, 0x42,                 //  (MAIN)   INPUT              0x00000042 (1 field x 4 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 1=Null 0=NonVolatile 0=Bitmap 
    0x75, 0x04,                 //  (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field <-- Redundant: REPORT_SIZE is already 4 
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0)  <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x35, 0x00,                 //  (GLOBAL) PHYSICAL_MINIMUM   0x00 (0) <-- Redundant: PHYSICAL_MINIMUM is already 0 <-- Info: Consider replacing 35 00 with 34
    0x45, 0x00,                 //  (GLOBAL) PHYSICAL_MAXIMUM   0x00 (0)  <-- Info: Consider replacing 45 00 with 44
    0x65, 0x00,                 //  (GLOBAL) UNIT               0x00 No unit (0=None)  <-- Info: Consider replacing 65 00 with 64
    0x81, 0x03,                 //  (MAIN)   INPUT              0x00000003 (1 field x 4 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x05, 0x09,                 //  (GLOBAL) USAGE_PAGE         0x0009 Button Page 
    0x19, 0x01,                 //  (LOCAL)  USAGE_MINIMUM      0x00090001 Button 1 Primary/trigger (Selector, On/Off Control, Momentary Control, or One Shot Control)  
    0x29, 0x0F,                 //  (LOCAL)  USAGE_MAXIMUM      0x0009000F Button 15 (Selector, On/Off Control, Momentary Control, or One Shot Control)  
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x01,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
    0x75, 0x01,                 //  (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
    0x95, 0x0F,                 //  (GLOBAL) REPORT_COUNT       0x0F (15) Number of fields  
    0x81, 0x02,                 //  (MAIN)   INPUT              0x00000002 (15 fields x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x01,                 //  (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field <-- Redundant: REPORT_SIZE is already 1 
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
    0x81, 0x03,                 //  (MAIN)   INPUT              0x00000003 (1 field x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x05, 0x0C,                 //  (GLOBAL) USAGE_PAGE         0x000C Consumer Device Page 
    0x0A, 0xB2, 0x00,           //  (LOCAL)  USAGE              0x000C00B2 Record (On/Off Control)  <-- Info: Consider replacing 0A B200 with 09 B2
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x01,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x75, 0x01,                 //  (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field <-- Redundant: REPORT_SIZE is already 1 
    0x81, 0x02,                 //  (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x15, 0x00,                 //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                 //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x07,                 //  (GLOBAL) REPORT_SIZE        0x07 (7) Number of bits per field  
    0x95, 0x01,                 //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x81, 0x03,                 //  (MAIN)   INPUT              0x00000003 (1 field x 7 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x05, 0x0F,                 //  (GLOBAL) USAGE_PAGE         0x000F Physical Interface Device Page 
    0x09, 0x21,                 //  (LOCAL)  USAGE              0x000F0021 Set Effect Report (Logical Collection)  
    0x85, XBOX_OUTPUT_REPORT_ID,//  (GLOBAL) REPORT_ID          0x03 (3)  
    0xA1, 0x02,                 //  (MAIN)   COLLECTION         0x02 Logical (Usage=0x000F0021: Page=Physical Interface Device Page, Usage=Set Effect Report, Type=Logical Collection)
    0x09, 0x97,                 //    (LOCAL)  USAGE              0x000F0097 DC Enable Actuators (Selector)  
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x01,                 //    (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
    0x75, 0x04,                 //    (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field  
    0x95, 0x01,                 //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x91, 0x02,                 //    (MAIN)   OUTPUT             0x00000002 (1 field x 4 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                 //    (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x04,                 //    (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field <-- Redundant: REPORT_SIZE is already 4 
    0x95, 0x01,                 //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x91, 0x03,                 //    (MAIN)   OUTPUT             0x00000003 (1 field x 4 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x09, 0x70,                 //    (LOCAL)  USAGE              0x000F0070 Magnitude (Dynamic Value)  
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x64,                 //    (GLOBAL) LOGICAL_MAXIMUM    0x64 (100)  
    0x75, 0x08,                 //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
    0x95, 0x04,                 //    (GLOBAL) REPORT_COUNT       0x04 (4) Number of fields  
    0x91, 0x02,                 //    (MAIN)   OUTPUT             0x00000002 (4 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x09, 0x50,                 //    (LOCAL)  USAGE              0x000F0050 Duration (Dynamic Value)  
    0x66, 0x01, 0x10,           //    (GLOBAL) UNIT               0x1001 Time in seconds [1 s units] (1=System=SI Linear, 1=Time=Seconds)  
    0x55, 0x0E,                 //    (GLOBAL) UNIT_EXPONENT      0x0E (Unit Value x 10⁻²)  
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x00,           //    (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
    0x75, 0x08,                 //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field <-- Redundant: REPORT_SIZE is already 8 
    0x95, 0x01,                 //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
    0x91, 0x02,                 //    (MAIN)   OUTPUT             0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x09, 0xA7,                 //    (LOCAL)  USAGE              0x000F00A7 Start Delay (Dynamic Value)  
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x00,           //    (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255) <-- Redundant: LOGICAL_MAXIMUM is already 255 
    0x75, 0x08,                 //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field <-- Redundant: REPORT_SIZE is already 8 
    0x95, 0x01,                 //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x91, 0x02,                 //    (MAIN)   OUTPUT             0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0x65, 0x00,                 //    (GLOBAL) UNIT               0x00 No unit (0=None)  <-- Info: Consider replacing 65 00 with 64
    0x55, 0x00,                 //    (GLOBAL) UNIT_EXPONENT      0x00 (Unit Value x 10⁰)  <-- Info: Consider replacing 55 00 with 54
    0x09, 0x7C,                 //    (LOCAL)  USAGE              0x000F007C Loop Count (Dynamic Value)  
    0x15, 0x00,                 //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x00,           //    (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255) <-- Redundant: LOGICAL_MAXIMUM is already 255 
    0x75, 0x08,                 //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field <-- Redundant: REPORT_SIZE is already 8 
    0x95, 0x01,                 //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1 
    0x91, 0x02,                 //    (MAIN)   OUTPUT             0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
    0xC0,                       //(MAIN)   END_COLLECTION     Logical 
    0xC0                    //(MAIN)   END_COLLECTION     Application 
}; 


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
    // void onRead(NimBLECharacteristic* pCharacteristic) override {}
    // void onNotify(NimBLECharacteristic* pCharacteristic) override {}
    // void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) override {}

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
    this->_hid->pnp(0x01, XBOX_VENDOR_ID, XBOX_1914_PRODUCT_ID, XBOX_1914_BCD_DEVICE_ID);
    this->_hid->hidInfo(0x00, 0x01);
    this->_hid->reportMap((uint8_t*)XboxOneS_1914_HIDDescriptor, sizeof(XboxOneS_1914_HIDDescriptor));

    this->_input = this->_hid->inputReport(XBOX_INPUT_REPORT_ID);
    this->_output = this->_hid->outputReport(XBOX_OUTPUT_REPORT_ID);
    this->_hidOutputCallbacks = new HIDOutputCallbacks(this);
    this->_output->setCallbacks(this->_hidOutputCallbacks);
    this->_hid->startServices();

    // Start BLE advertisement
    this->_advertising = this->_server->getAdvertising();
    this->_advertising->setAppearance(HID_GAMEPAD);
    this->_advertising->addServiceUUID(this->_hid->deviceInfo()->getUUID());
    this->_advertising->addServiceUUID(this->_hid->hidService()->getUUID());
    this->_advertising->addServiceUUID(this->_hid->batteryService()->getUUID());
    this->_advertising->setScanResponse(false);
    this->_advertising->start();

    this->_hid->setBatteryLevel(100);
    this->_hid->batteryLevel()->notify();

    printf("\n\nXInput Server Started\n");
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