ESP32-CAM Interval
==================

__Status Update:__ Since I bought another stock camera that matches my needs I
no longer actively develop this project.


This firmware turns a ESP32-CAM module into a low power time-lapse camera.
Taking pictures at a set interval, and storing them to SD card. In between
captures the device will go into deep sleep mode to save battery power.

Compilation
-----------
Use the Arduino IDE with ESP32 support to compile the firmware.

The config.h file offers some compile time customizations.

Before compiling the html_content.cpp file needs to be generated. Do this by
executing the gen_html_content.sh script.

The tzinfo.json file can be updated with the tool from
https://github.com/pgurenko/tzinfo. But this is normally not necessary.

Picture Names
-------------
Every time the device boots a new directory is created on the SD card. The
directory name is created following the template 'timelapseXXXX', where 'XXXX'
is replaced by a free sequence number. Pictures are stored to this directory.

The picture filenames contain the date and time of taking the pictures. If the
time is not set, the clock will start at UNIX epoch, i.e. 01-01-1970 00:00:00.

Set-up mode
-----------
When the camera is powered up it will go into set-up mode. In this mode the
camera has an open Wi-Fi access point and a web server. Connect to the access
point and go to http://camera.local to configure the camera and enable it.

Upon opening the set-up mode web site the camera's clock is automatically
synchronized to the browsers clock.

In set-up mode the LED will blink at a 1 second interval.

Configuration file
------------------
To configure the software create a file named camera.cfg in the root
directory of the SD card. The configuration file is text based containing one
configuration option per line. Configuration options have the format
'key=value'. Spaces are striped from the front and back of both the key and
value. Lines starting with a '#' and empty lines are ignored.

For a full list of available configuration options, see the camera.cfg example
configuration.

Power saving
------------
The firmware puts the device in deep sleep if the interval between images is
big. However the camera is _not_ powered down. This is because the pin required
to properly power down the camera is not connected to the MCU by default on the
ESP32-CAM board. While the CAM_PWR pin that is available only switches of the
1.2 and 2.8 Volt voltage regulators. This only partially powers down the camera
and leaves it in some undefined state. This is probably not according to spec

To properly power down the camera a modification must be made to the PCB. For
details see doc/power_consumption.md.

Troubleshooting
---------------
If the red LED is flashing two short pulses every 2 seconds, this means an fatal
software error occurred.

The only way to troubleshoot issues is to connect to the serial port and check
the logging.

The serial port uses the following settings: 115200 Baud, 8N1.

Notes
-----
The following things are important to know about the ESP32-CAM board hardware:

 - The OV3640(/OV3660?) camera is NOT supported by the ESP32-CAM board. Because
   the camera uses a different core voltage, 1.5 V instead of 1.2 V. And the
   maximum Vdd I/O is only 3.0 V, instead of 3.3 V.
 - Other cameras then the OV2640 are unlikely to work. Because the driver uses
   the old parallel DVP bus to communicate to the camera. Most newer cameras
   use the serial MIPI bus instead.
 - GPIO12/HS_DATA2 pin is free for use in 1-bit SD-card bus mode. However, this
   pin is used by the ESP-32s module as strapping pin to configure the VDD_SDIO
   voltage, and thus must be low or floating at boot. See:
   https://wiki.ai-thinker.com/esp32/spec/esp32s
