ESP32-CAM Interval
==================
This firmware turns a ESP32-CAM module into a low power time-lapse camera.
Taking pictures at a set interval, and storing them to SD card. In between
captures the device will go into deep sleep mode to save battery power.

Compilation
-----------
Use the Arduino IDE with ESP32 support to compile the firmware.

The config.h file offers some compile time customizations.

Picture Names
-------------
Every time the device boots a new directory is created on the SD card. The
directory name is created following the template 'timelapseXXXX', where 'XXXX'
is replaced by a free sequence number. Pictures are stored to this directory.

The picture filenames contain the date and time of taking the pictures. If the
time is not set, the clock will start at UNIX epoch, i.e. 01-01-1970 00:00:00.

Configuration
-------------
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
The only way to troubleshoot issues is to connect to the serial port and check
the logging.

The serial port uses the following settings: 115200 Baud, 8N1.
