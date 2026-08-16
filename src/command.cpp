//
// Created by Ben Levitt on 10/14/25.
//

#include "command.h"
#include <NimBLEService.h>

CommandHandler::CommandHandler(NimBLEServer *server, CommandCallback_t callback, NimBLECharacteristic **reportCharacteristic)
{
    NimBLEService *cmdService = server->createService(COMMAND_SERVICE_ID);

    _commandCharacteristic = cmdService->createCharacteristic(
        COMMAND_CHARACTERISTIC_ID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
        18);
    _commandCharacteristic->setCallbacks(this);
    uint8_t commandData[4] = {0, 0, 0, 0};
    _commandCharacteristic->setValue((uint8_t*)commandData, 4);
    _commandCallback = callback;

    _versionCharacteristic = cmdService->createCharacteristic(
        COMMAND_VERSION_CHARACTERISTIC_ID,
        NIMBLE_PROPERTY::READ,
        2);
    uint8_t versionData[2] = COMMAND_VERSION;
    _versionCharacteristic->setValue((uint8_t*)versionData, 2);

    char fwVersionData[12] = COMMAND_FW_VERSION;
    _fwVersionCharacteristic = cmdService->createCharacteristic(
        COMMAND_FW_VERSION_CHARACTERISTIC_ID,
        NIMBLE_PROPERTY::READ,
        strlen(fwVersionData));
    _fwVersionCharacteristic->setValue((uint8_t*)fwVersionData, strlen(fwVersionData));
    
    // Only used for gamepad mode.  Sends button and axis states.
    if (reportCharacteristic) {
        *reportCharacteristic = cmdService->createCharacteristic(
            COMMAND_REPORT_CHARACTERISTIC_ID,
            NIMBLE_PROPERTY::NOTIFY,
            8);
    }
}

void CommandHandler::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo)
{
    NimBLEAttValue val = pCharacteristic->getValue();
    _commandCallback(val.data(), val.size());
}

void CommandHandler::send_command(const uint8_t* commandData, const uint8_t length)
{
    _commandCharacteristic->setValue(commandData, length);
    _commandCharacteristic->notify();
}
