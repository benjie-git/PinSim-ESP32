//
// Created by Ben Levitt on 10/14/25.
//

#ifndef PINSIM_ESP32_COMMAND_H
#define PINSIM_ESP32_COMMAND_H

#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>

#define COMMAND_SERVICE_ID        "4a761e27-a006-4b89-8d76-1c4b9e2402d4"
#define COMMAND_CHARACTERISTIC_ID "4a761e27-a006-4b89-8d76-1c4b9e2402d5"


// Command num goes in commandData[0]
#define COMMAND_ACCEL_CAL           3
#define COMMAND_PLUNGER_SET_MIN     4
#define COMMAND_PLUNGER_SET_MAX     5
#define COMMAND_PLUNGER_SET_ZERO    6
#define COMMAND_TOGGLE_CONTROL_SWAP 7
#define COMMAND_TOGGLE_SOLENOIDS    8
#define COMMAND_PAIR_CLEAR          9
#define COMMAND_PAIR_START          10
#define COMMAND_TOGGLE_KEYBOARD     11
#define COMMAND_SEND_STATUS         12
#define COMMAND_SET_PINSIM_ID       13  // commandData[1] contains id
#define COMMAND_SET_KEY_MAPPING     14
#define COMMAND_RESET_KEY_MAPPING   15

// Command response num in commandData[0]
#define COMMAND_RESPONSE_STATUS     2
#define COMMAND_RESPONSE_KEYMAP     3

#define COMMAND_STATUS_PLUNGER_CONTROL_RIGHT    1
#define COMMAND_STATUS_SOLENOIDS_ENABLED        2
#define COMMAND_STATUS_KEYBOARD_MODE            4


typedef void (*CommandCallback_t)(const uint8_t *args, uint8_t length);


class CommandHandler : public NimBLECharacteristicCallbacks
{
public:
    CommandHandler(NimBLEServer *server, CommandCallback_t callback);
    void send_command(const uint8_t* data, uint8_t length=4);

private:
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    NimBLECharacteristic *_commandCharacteristic = nullptr;
    CommandCallback_t _commandCallback = nullptr;
};


#endif //PINSIM_ESP32_COMMAND_H