//
// Created by Ben Levitt on 10/14/25.
//

#include "command.h"
#include <NimBLEService.h>

CommandHandler::CommandHandler(NimBLEServer *server, CommandCallback_t callback)
{
    NimBLEService *cmdService = server->createService(COMMAND_SERVICE_ID);
    // server->addService(cmdService);

    _commandCharacteristic = cmdService->createCharacteristic(
        COMMAND_CHARACTERISTIC_ID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
        18);
    _commandCharacteristic->setCallbacks(this);
    uint8_t commandData[4] = {0, 0, 0, 0};
    _commandCharacteristic->setValue((uint8_t*)commandData, 4);
    _commandCallback = callback;
    // cmdService->addCharacteristic(_commandCharacteristic);

    _versionCharacteristic = cmdService->createCharacteristic(
        COMMAND_VERSION_CHARACTERISTIC_ID,
        NIMBLE_PROPERTY::READ,
        2);
    uint8_t versionData[2] = COMMAND_VERSION;
    _versionCharacteristic->setValue((uint8_t*)versionData, 2);
    // cmdService->addCharacteristic(_versionCharacteristic);

    cmdService->start();
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
