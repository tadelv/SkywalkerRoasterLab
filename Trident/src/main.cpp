#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <PID_v1.h>
#include <WiFi.h>

#include "CommandLoop.h"
#include "SkiComms.h"
#include "SkiCMD.h"
#include "api.h"
#include "ble.h"
#include "display.h"
#include "freertos/idf_additions.h"
#include "model.h"
#include "pindef.h"
#include "state_request_queue.h"
#include "wifi_setup.h"
#include "wifi_manager.h"
#include <esp_wifi.h>

// -----------------------------------------------------------------------------
// Global Bean Temperature Variable
// -----------------------------------------------------------------------------
double temp = 0.0; // Filtered temperature

// -----------------------------------------------------------------------------
// Define PID variables
// -----------------------------------------------------------------------------
double pInput, pOutput;
double pSetpoint = 0.0; // Desired temperature (adjustable on the fly)
int pMode = P_ON_M;
// was 20,1,3 at 1sec
// pid calibrations (adjustable on the fly)
double Kp = 20.0, Ki = 0.5, Kd = 4.0;
int pSampleTime = 1000; // ms (adjustable on the fly)
int manualHeatLevel = 0;
// pid instance with our default values
PID myPID(&pInput, &pOutput, &pSetpoint, Kp, Ki, Kd, pMode, DIRECT);

const unsigned int LED_BLUE[3] = {0, 0, 32};
const unsigned int LED_GREEN[3] = {0, 128, 0};
const unsigned int LED_RED[3] = {128, 0, 0};
const unsigned int LED_BLACK[3] = {0, 0, 0};
const unsigned int LED_YELLOW[3] = {0, 128, 128};

typedef enum { booting = 0, connected, disconnected } BloodhoundStateT;

AsyncWebServer server(80);
AsyncWebSocket
    wsConsole("/console"); // WebSocket for console (replaces WebSerial)
const char rgbLedPin = RGB_PIN;
// const char ledPin = 15;
bool isOn = false;
BloodhoundStateT m_state = booting;
void webSerialLoop(void *params);
void ledControl();

// Function to handle incoming console WebSocket messages
void handleConsoleMessage(AsyncWebSocketClient *client, uint8_t *data,
                          size_t len) {
  String input = String((char *)data, len);
  input.trim();

  // Echo the command back to the sender
  client->text("> " + input);

  // Parse and execute the command
  parseAndExecuteCommands(input);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(rgbLedPin, OUTPUT);

  rgbLedWrite(rgbLedPin, LED_RED[0], LED_RED[1], LED_RED[2]);

  initStateQueue();
  setupWifi();

  // Start WiFi monitoring for automatic reconnection
  startWiFiMonitor();

  // Setup console WebSocket (replaces WebSerial)
  wsConsole.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                       AwsEventType type, void *arg, uint8_t *data,
                       size_t len) {
    if (type == WS_EVT_CONNECT) {
      String welcome = "Console WebSocket Connected\n";
      welcome += "Type commands to control the system\n";
      welcome += "Use 'help' for available commands\n";
      welcome += "IP: " + WiFi.localIP().toString();
      client->text(welcome);
    } else if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->opcode == WS_TEXT) {
        handleConsoleMessage(client, data, len);
      }
    } else if (type == WS_EVT_DISCONNECT) {
      // Client disconnected
    }
  });
  server.addHandler(&wsConsole);

  setupMainLoop(&server);
  setupApi(&server);
  server.begin();
  // Increase stack size from 8192 to 12288 and priority from 1 to 2
  xTaskCreate(webSerialLoop, "WebSerialTask", 12288,
              NULL, 2, NULL);
  displayInit();
  myPID.SetOutputLimits(0, 95);

  // Configure WiFi/BLE coexistence for better stability
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // Minimum power save for better coexistence
  // Note: esp_coex_set_preference may not be available in all ESP32 framework versions
  // The WiFi power save setting above provides basic coexistence support

  // Let WiFi stabilize before starting BLE
  delay(2000);
  initBLE("Trident", "1.0.3", "Skywalker-Trident");

  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);
  pinMode(RX_PIN, INPUT);

  attachInterrupt(RX_PIN, watchRoasterStart, FALLING);

  shutdown();
}

StateRequestT _currentState = {0};
void webSocketLoop() {
  handleREAD();
  StateDataT data = {temp, _currentState};
  StateRequestT req = socketTick(data);
}

void bleLoop() {

  StateDataT data = {temp, _currentState};
  StateRequestT req = bleTick(data);
}

void serialLoop() {
  if (Serial.available() <= 0) {
    return;
  }
  String command = Serial.readStringUntil('\n');
  command.trim();
  CommandTypeT type = classifyCommandType(command);
  if (type == CMDType_READ) {
    // Use snprintf instead of String concatenation to prevent heap fragmentation
    char readMsg[64];
    snprintf(readMsg, sizeof(readMsg), "0,%.1f,%.1f,%d,%d\r\n",
             temp, temp, _currentState.heater, _currentState.fan);
    Serial.println(readMsg);
  } else if (type == CMDType_STATE_REQUEST) {
    StateRequestT req = parseCommandToStateRequest(command);
    enqueueStateRequest(req, SOURCE_USB);
  } else {
    parseAndExecuteCommands(command);
  }
}

void webSerialLoop(void *params) {
  static unsigned long lastStatusTime = 0;
  static unsigned long lastCleanupTime = 0;
  static unsigned long lastStackCheck = 0;
  static unsigned long lastHeapCheck = 0;
  const unsigned long STATUS_INTERVAL = 1000;  // Send status every 1 second
  const unsigned long CLEANUP_INTERVAL = 5000; // Cleanup every 5 seconds
  const unsigned long STACK_CHECK_INTERVAL = 60000; // Check stack every minute
  const unsigned long HEAP_CHECK_INTERVAL = 5000; // Check heap every 5 seconds

  while (1) {
    unsigned long now = millis();

#ifdef DEBUG
    // Monitor stack usage
    if (now - lastStackCheck > STACK_CHECK_INTERVAL) {
      UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
      if (stackHighWater < 1024) {
        D_printf("WARNING: WebSerialTask low stack: %d bytes free\n", stackHighWater);
      }
      lastStackCheck = now;
    }
#endif

    // Monitor heap health
    if (now - lastHeapCheck > HEAP_CHECK_INTERVAL) {
      uint32_t freeHeap = ESP.getFreeHeap();
      uint32_t minFreeHeap = ESP.getMinFreeHeap();

      if (freeHeap < 80000) {  // Less than 80KB free
        D_printf("WARNING: Heap low - Free: %d, Min: %d\n", freeHeap, minFreeHeap);
      }
#ifdef DEBUG
      else {
        // Log heap status periodically for monitoring
        D_printf("Heap: %d free, %d min\n", freeHeap, minFreeHeap);
      }
#endif
      lastHeapCheck = now;
    }

    // Send status periodically, not every loop
    if (now - lastStatusTime > STATUS_INTERVAL) {
      // Use snprintf instead of String concatenation to prevent heap fragmentation
      char readMsg[128];
      snprintf(readMsg, sizeof(readMsg), "Status:\n0,%.1f,%.1f,%d,%d\nWifi: %s",
               temp, temp, sendBuffer[HEAT_BYTE], sendBuffer[VENT_BYTE],
               WiFi.localIP().toString().c_str());
      displayMessage(readMsg);
      lastStatusTime = now;
    }

    // Clean up clients less frequently
    if (now - lastCleanupTime > CLEANUP_INTERVAL) {
      wsConsole.cleanupClients();
      lastCleanupTime = now;
    }

    ledControl();
#ifdef S3
    serialLoop();
#endif
    webSocketLoop();
    bleLoop();

    delay(200);
    taskYIELD(); // Explicitly yield to other tasks
  }
  vTaskDelete(NULL);
}

void loop() {
  // roaster shut down, clear our buffers
  if (itsbeentoolong()) {
    D_println("too long, shutting down");
    shutdown();
  }

  // roaster message start found, go get it
  if (roasterStartFound) {
    getRoasterMessage();
  }

  processStateQueue();
  // Ensure PID or manual heat control is handled
  handlePIDControl();

  sendRoasterMessage();

  _currentState = getCurrentState();
}

unsigned long LED_LAST_ON_MS = 0;
const unsigned long LED_FLASH_DELAY_MS = 2000;
extern bool deviceConnected;

void ledControl() {
  int now = millis();
  if (now - LED_LAST_ON_MS > LED_FLASH_DELAY_MS) {
    isOn = !isOn;
    LED_LAST_ON_MS = now;
  }
  if (isOn) {
    rgbLedWrite(rgbLedPin, 0, 0, 0);
  } else {
    rgbLedWrite(rgbLedPin, LED_BLUE[0], LED_BLUE[1], LED_BLUE[2]);
  }
}
