#ifdef DEBUG_PORT

// Use:
// #define DEBUG_PORT Serial
//   or...
// #define DEBUG_PORT TelnetDebug
//
// DEBUG_PRINT("My string: %s", str);
// DEBUG_PRINTLN("My string");
//
// TODO: DEBUG_PRINTLN to take args like printf(..., "\n")
//   https://stackoverflow.com/a/9204947/109707

#define DEBUG_PRINT(...) DEBUG_PORT.printf(__VA_ARGS__)
#define DEBUG_PRINTLN(...) DEBUG_PORT.println(__VA_ARGS__)

#if DEBUG_PORT == TelnetDebug
#include <RemoteDebug.h>

// telnet esp-ota.local
//   or
// while true; do nc -w 5 esp-ota.local 23; echo "\nReconnecting..."; done

RemoteDebug TelnetDebug;

void initTelnetDebug() {
  // TODO: Extract this
  TelnetDebug.begin(ArduinoOTA.getHostname());
  TelnetDebug.setResetCmdEnabled(true);
  TelnetDebug.setSerialEnabled(true);
  TelnetDebug.showDebugLevel(false);
}
#endif // DEBUG_PORT == TelnetDebug

#else // DEBUG_PORT

#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)

#endif // DEBUG_PORT
