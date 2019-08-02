#ifndef __CONFIG_H__
#define __CONFIG_H__

// Enable WiFi for NTP time synchronization
#define WITH_WIFI

// Enable 4-BIT bus signalling to SD card(if supported by card). This is
// disabled by default because the AI_THINKER boards have a Flash LED connected
// to SD DATA1 line.
//#define WITH_SD_4BIT

#endif // __CONFIG_H__
