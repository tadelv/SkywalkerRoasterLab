#ifndef PINDEF
#define PINDEF
#if defined(S3MINI) || defined(S3)
const int RX_PIN = 20;
const int TX_PIN = 19;
const int RGB_PIN = 48;
#else
const int RX_PIN = 13;
const int TX_PIN = 12;
const int RGB_PIN = 8;
#endif
#endif
