
/* Replaces Classic Bluetooth with BLE (NUS).
 *
 * Service UUID:
 *     6e400001-b5a3-f393-e0a9-e50e24dcca9e
 * Characteristics UUIDs:
 *   - Write Characteristic (RX):
 *     6e400002-b5a3-f393-e0a9-e50e24dcca9e
 *   - Notify Characteristic (TX):
 *     6e400003-b5a3-f393-e0a9-e50e24dcca9e
 *
 * Sends notifications for temperature/status data.
 * Expects commands via the write characteristic.*/

#include "ble.h"
#include "dlog.h"
#include "model.h"
#include "state_request_queue.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// -----------------------------------------------------------------------------
// BLE UUIDs for Nordic UART Service
// -----------------------------------------------------------------------------
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // NUS service
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Write
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Notify

// -----------------------------------------------------------------------------
// BLE Globals
// -----------------------------------------------------------------------------
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
extern String firmWareVersion;
extern String sketchName;

StateRequestT _currentRequest = {255, 255, 255, 255};
StateDataT _currentData = {0};

// Mutex for thread-safe BLE state access
static SemaphoreHandle_t bleMutex = nullptr;

StateRequestT bleTick(StateDataT data) {
  StateRequestT response = {255, 255, 255, 255};

  // Protect state access with mutex
  if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    _currentData = data;
    response = _currentRequest;
    _currentRequest = {255, 255, 255, 255};
    xSemaphoreGive(bleMutex);
  }

  return response;
}

void notifyBLEClient(const String &message);

// -----------------------------------------------------------------------------
// BLE Server Callbacks (static to prevent memory leaks)
// -----------------------------------------------------------------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, ble_gap_conn_desc *desc) override {
    deviceConnected = true;

    // Change BLE connection parameters per apple ble guidelines
    // (for this client, min interval 15ms (/1.25), max 30ms (/1.25), latency 4
    // frames, timeout 5sec(/10ms)
    // https://docs.silabs.com/bluetooth/4.0/bluetooth-miscellaneous-mobile/selecting-suitable-connection-parameters-for-apple-devices
    // pServer->updateConnParams(param->connect.remote_bda, 12, 24, 4, 500);
    // For NimBLE:
    // pServer->updateConnParams(desc->conn_handle, 12, 24, 4, 500);

    D_println("BLE: Client connected.");
  }
  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    D_println("BLE: Client disconnected. Restarting advertising...");
    pServer->getAdvertising()->start();
  }
};

// -----------------------------------------------------------------------------
// BLE Characteristic Callbacks (static to prevent memory leaks)
// -----------------------------------------------------------------------------
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();

    // Validate buffer size to prevent overflow
    if (rxValue.length() > 128) {
      D_println("BLE: Command too long, ignoring");
      return;
    }

    if (rxValue.length() > 0) {
      String input = rxValue;

      D_print("BLE Write Received: ");
      D_println(input);

      CommandTypeT type = classifyCommandType(input);
      if (type == CMDType_READ) {
        // Use snprintf instead of String concatenation to prevent heap
        // fragmentation
        char readMsg[64];
        if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          snprintf(readMsg, sizeof(readMsg), "0,%.1f,%.1f,%d,%d\r\n",
                   _currentData.temp, _currentData.temp,
                   _currentData.request.heater, _currentData.request.fan);
          xSemaphoreGive(bleMutex);
          notifyBLEClient(String(readMsg));
        }
        return;
      }
      if (type == CMDType_CHAN) {
        // TC4 protocol: parse channel mask from CHAN;xxxx and echo it back
        String channelMask = "2100"; // Default
        int splitPos = input.indexOf(';');
        if (splitPos >= 0 && splitPos < input.length() - 1) {
          channelMask = input.substring(splitPos + 1);
          channelMask.trim();
        }
        String response = "\r# Active channels set to " + channelMask + "\r\n";
        notifyBLEClient(response); // Only send ONCE
        return;
      }

      // Parse and enqueue command
      StateRequestT req = parseCommandToStateRequest(input);
      if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _currentRequest = req;
        xSemaphoreGive(bleMutex);
      }
      enqueueStateRequest(req, SOURCE_BLE);
    }
  }
};

void notifyBLEClient(const String &message) {
  if (!deviceConnected || !pTxCharacteristic) {
    return;
  }

  // BLE overhead is 3 bytes, so effective MTU is MTU-3
  size_t maxLen = 185 - 3; // 182 bytes max payload
  size_t msgLen = min(message.length(), maxLen);

  // Protect notification with mutex
  if (!bleMutex || xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    D_println("BLE: Failed to acquire mutex for notify");
    return;
  }

  pTxCharacteristic->setValue((uint8_t *)message.c_str(), msgLen);
  pTxCharacteristic->notify();
  xSemaphoreGive(bleMutex);

#ifdef DEBUG
  D_printf("BLE TX: %s", message.c_str());
#endif
}

// Static callback objects to prevent memory leaks
static MyServerCallbacks serverCallbacks;
static MyCallbacks rxCallbacks;

void initBLE(String sketchName, String firmWareVersion, String boardID) {
  // Create mutex for thread-safe state access
  bleMutex = xSemaphoreCreateMutex();
  if (!bleMutex) {
    D_println("BLE: Failed to create mutex!");
  }

  BLEDevice::init(boardID);
  BLEDevice::setMTU(185);
  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks); // Use static object, not new

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Roaster notifes to HiBean
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  // pTxCharacteristic->addDescriptor(new BLE2902());

  // Hibean commands to Roaster
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(&rxCallbacks); // Use static object, not new
  // pRxCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  // esp32 information to HiBean for support/debug purposes
  BLEService *devInfoService = pServer->createService("180A");
  BLECharacteristic *boardCharacteristic = devInfoService->createCharacteristic(
      "2A29", BLECharacteristic::PROPERTY_READ);
  boardCharacteristic->setValue(boardID);
  // boardCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *sketchNameCharacteristic =
      devInfoService->createCharacteristic("2A28",
                                           BLECharacteristic::PROPERTY_READ);
  sketchNameCharacteristic->setValue(sketchName);
  // sketchNameCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *firmwareCharacteristic =
      devInfoService->createCharacteristic("2A26",
                                           BLECharacteristic::PROPERTY_READ);
  firmwareCharacteristic->setValue(sketchName + ", " + firmWareVersion);
  // firmwareCharacteristic->addDescriptor(new BLE2902());

  devInfoService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  D_println("BLE Advertising started...");
}
