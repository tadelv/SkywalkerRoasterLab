#include <ArduinoJson.h>

#include "CommandLoop.h"
#include "dlog.h"
#include "model.h"
#include "state_request_queue.h"
#include <ESPAsyncWebServer.h>

AsyncWebSocket ws("/ws");

StateDataT state = {0};
StateRequestT request = {255, 255, 255, 255, ""};

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  switch (type) {
  case WS_EVT_CONNECT:
    D_printf("[%u] Connected!\n", client->id());
    // client->text("Connected");

    break;
  case WS_EVT_DISCONNECT: {
    D_printf("[%u] Disconnected!\n", client->id());
    // turn off heater and set fan to 100%
    // setHeaterPower(0);
    // setFanSpeed(100);
  } break;
  case WS_EVT_DATA: {

    AwsFrameInfo *info = (AwsFrameInfo *)arg;
#ifdef DEBUG
    D_printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(),
             (info->opcode == WS_TEXT) ? "text" : "binary", info->len);
    D_printf("final: %d\n", info->final);
#endif
    /*if (info->opcode != WS_TEXT || !info->final) {*/
    /*  break;*/
    /*}*/

    // Use stack buffer instead of String to prevent heap fragmentation
    char msgBuf[512];
    size_t copyLen = (info->len < sizeof(msgBuf) - 1) ? info->len : sizeof(msgBuf) - 1;
    memcpy(msgBuf, data, copyLen);
    msgBuf[copyLen] = '\0';
#ifdef DEBUG
    D_printf("msg: %s\n", msgBuf);
#endif

    JsonDocument doc;

    // DEBUG WEBSOCKET
    // D_printf("[%u] get Text: %s\n", num, payload);

    // Extract Values lt. https://arduinojson.org/v6/example/http-client/
    // Artisan Anleitung: https://artisan-scope.org/devices/websockets/

    DeserializationError error = deserializeJson(doc, msgBuf);
    if (error) {
      D_printf("JSON parse error: %s\n", error.c_str());
      return;
    }

    long ln_id = doc["id"].as<long>();
    // Get BurnerVal from Artisan over Websocket
    if (!doc["BurnerVal"].isNull()) {
      unsigned char val = doc["BurnerVal"].as<unsigned char>();
      D_printf("BurnerVal: %d\n", val);
      // DimmerVal = doc["BurnerVal"].as<long>();
      request.heater = val;
    }
    if (!doc["FanVal"].isNull()) {
      unsigned char val = doc["FanVal"].as<unsigned char>();
      D_printf("FanVal: %d\n", val);
      request.fan = val;
    }
    if (!doc["Drum"].isNull()) {
      unsigned char val = doc["Drum"].as<unsigned char>();
      D_printf("Drum: %d\n", val);
      request.drum = val;
    }
    if (!doc["Cooling"].isNull()) {
      unsigned char val = doc["Cooling"].as<unsigned char>();
      D_printf("Cooling: %d\n", val);
      request.cooling = val;
    }

    // Send Values to Artisan over Websocket
    JsonDocument root;
    root["id"] = ln_id;
    const char *command = doc["command"].as<const char *>();
    if (command != NULL && strncmp(command, "getData", 7) == 0) {
      root["data"]["ET"] = state.temp; // Med_ExhaustTemp.getMedian()
      root["data"]["BT"] = state.temp; // Med_BeanTemp.getMedian();
      root["data"]["BurnerVal"] = state.request.heater;
      root["data"]["FanVal"] = state.request.fan;
      root["data"]["Drum"] = state.request.drum;
      root["data"]["Cool"] = state.request.cooling;
    }

    // Increase buffer size and add overflow check
    char buffer[384];
    size_t len = serializeJson(root, buffer, sizeof(buffer));
    if (len >= sizeof(buffer)) {
      D_println("ERROR: JSON response truncated");
      return;
    }

    // Validate client connection before sending
    if (client && client->status() == WS_CONNECTED) {
      client->text(buffer, len);
    }
    // send message to client
    // webSocket.sendTXT(num, "message here");

    // send data to all connected clients
    // webSocket.broadcastTXT("message here");
    enqueueStateRequest(request, SOURCE_WEBSOCKET);
  } break;
  default: // send message to client
    // D_printf("unhandled message type: %d\n", type);
    // webSocket.sendBIN(num, payload, length);
    break;
  }
}

void setupMainLoop(AsyncWebServer *server) {
  ws.onEvent(onWsEvent);
  server->addHandler(&ws);
}

StateRequestT socketTick(StateDataT data) {
  state = data;
  // Cleanup removed - already handled in webSerialLoop every 5 seconds
  StateRequestT response = request;
  request = {255, 255, 255, 255, ""};
  return response;
}
