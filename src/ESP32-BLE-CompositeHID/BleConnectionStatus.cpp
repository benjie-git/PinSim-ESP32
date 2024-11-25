#include "BleConnectionStatus.h"
#include <NimBLEAdvertising.h>


BleConnectionStatus::BleConnectionStatus(void)
{
}

void BleConnectionStatus::onConnect(NimBLEServer *pServer, ble_gap_conn_desc* desc)
{
    pServer->updateConnParams(desc->conn_handle, 6, 15, 1, 300);
    this->connected++;
    if (!pServer->getAdvertising()->isAdvertising()) {
        pServer->startAdvertising(); // restart advertising
    }
}

void BleConnectionStatus::onDisconnect(NimBLEServer *pServer)
{
    this->connected--;
    if (!pServer->getAdvertising()->isAdvertising()) {
        pServer->startAdvertising(); // restart advertising
    }
}

