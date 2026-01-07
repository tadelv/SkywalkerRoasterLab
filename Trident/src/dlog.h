#include <ESPAsyncWebServer.h>

#ifndef DLOG
#define DLOG

extern AsyncWebSocket wsConsole;  // Declare external WebSocket

// Helper function to broadcast to WebSocket
inline void broadcastToConsole(const String &message) {
  wsConsole.textAll(message);
}

// Logging macros that send to both Serial and WebSocket
// Optimized to reduce heap fragmentation

// For String objects - use .c_str() method
#define D_println(x) do { \
  Serial.println(x); \
  if (wsConsole.count() > 0) { \
    String _msg = String(x); \
    _msg += "\n"; \
    wsConsole.textAll(_msg); \
  } \
} while(0)

#define D_print(x) do { \
  Serial.print(x); \
  if (wsConsole.count() > 0) { \
    wsConsole.textAll(String(x)); \
  } \
} while(0)

// For format strings - use this version with printf-style formatting
#define D_printf(...) do { \
  char buffer[256]; \
  snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
  Serial.print(buffer); \
  if (wsConsole.count() > 0) wsConsole.textAll(buffer); \
} while(0)

#endif
