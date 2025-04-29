#include <WebSerial.h>

#ifndef DLOG
#define DLOG

#define D_println(...) WebSerial.println(__VA_ARGS__)
#define D_print(...) WebSerial.print(__VA_ARGS__)
#define D_printf(...) WebSerial.printf(__VA_ARGS__)
#endif
