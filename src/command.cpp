//
// Created by Ben Levitt on 10/14/25.
//

#include "command.h"
#include <NimBLEService.h>

CommandHandler::CommandHandler(NimBLEServer *server, CommandCallback_t callback)
{
    NimBLEService *cmdService = server->createService(COMMAND_SERVICE_ID);
    _commandCharacteristic = cmdService->createCharacteristic(
        COMMAND_CHARACTERISTIC_ID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
        4);
    _commandCharacteristic->setCallbacks(this);
    server->addService(cmdService);

    uint8_t commandData[4] = {0, 0, 0, 0};
    _commandCharacteristic->setValue((uint8_t*)commandData, 4);
    _commandCallback = callback;
    cmdService->addCharacteristic(_commandCharacteristic);
    cmdService->start();
}

void CommandHandler::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo)
{
    uint32_t commandData32 = pCharacteristic->getValue<uint32_t>();
    uint8_t *commandData = (uint8_t*)&commandData32;
    printf("Got Command: %d (%d)\n", commandData[0], commandData[1]);
    _commandCallback(commandData);
}

void CommandHandler::send_command(uint8_t* commandData)
{
    printf("Sending Response: %d (%d, %d, %d)\n", commandData[0], commandData[1], commandData[2], commandData[3]);
    _commandCharacteristic->setValue(commandData, 4);
    _commandCharacteristic->notify();
}
