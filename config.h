#ifndef __CONFIG_H__
#define __CONFIG_H__

// Select camera board model
// Currently only the ESP32-CAM board from AI-Thinker is supported
#define CAMERA_MODEL_AI_THINKER

// Enable deep-sleep in between pictures
#define WITH_SLEEP

// Enable GNSS(GPS/Glonass/Gallileo/etc...) support
//#define WITH_GNSS

// Enable WiFi for NTP time synchronization
#define WITH_WIFI

// Enable Flash LED support
#define WITH_FLASH

// Enable Camera Power down support
// NOTE: This requires a modification on the AI-Thinker ESP32-CAM boards
//#define WITH_CAM_PWDN

// Shutdown camera low voltage regulators
// This shuts down the camera 1.2 and 2.8 Volt regulators in sleep state. This
// brings down the the current consumption to about 4 mA. However it leaves the
// camera in a half powered state since the 3.3 Volt is not shut down. Doing
// this is probably not according to spec. Using the WITH_CAM_PWDN option
// instead is suggested.
//#define WITH_EVIL_CAM_PWR_SHUTDOWN

// Enable 4-BIT bus signalling to SD card(if supported by card).
// This is disabled by default because the AI-Thinker ESP32-CAM boards have a
// Flash LED connected to SD DATA1 line.
//#define WITH_SD_4BIT

// Version number of firmware
#define VERSION_STR "0.1"

#endif // __CONFIG_H__
