ESP32-CAM Interval
==================
This firmware turns a Esp32-Cam module into a time-lapse camera, with
geo-tagging support. Taking pictures at a set interval, tagging them with the
current location and storing them to SD card.

Compilation
-----------
Use the Arduino IDE with ESP32 support to compile the firmware. For GNSS
support you also need the MicroNMEA library available from the library manager
in the Arduino IDE.

The config.h file allows some subsystems to be disabled.

Picture Names
-------------
Every time the device boots a new directory is created on the SD card. The
directory name is created following the template 'timelapseXXXX', where 'XXXX'
is replaced by a free sequence number. Pictures are stored to this directory.

The picture filenames contain the date and time of taking the pictures. For the
clock the be set correctly GNSS receiver will be used. If GNSS is not available
a NTP server can be used. For this Wi-Fi needs to be configured correctly and
be available at boot time. If no NTP is available the clock will start at UNIX
epoch, i.e. 01-01-1970 00:00:00.

Configuration
-------------
To configure the software create a file named camera.cfg in the root
directory of the SD card. The configuration file is text based containing one
configuration option per line. Configuration options have the format
'key=value'. Spaces are striped from the front and back of both the key and
value. Lines starting with a '#' and empty lines are ignored.

For a full list of available configuration options, see the camera.cfg example
configuration.

Global Navigation Satellite System (GNSS)
-----------------------------------------
A NMEA GNSS receiver can be connected. The GNSS data is used to set the clock
at boot and to geo-tag the images.

The GNSS receiver is connected to the serial port UART0. Only the receivers TX
signal is connected. The ESP32's UART0 TX signal is used for error/debug output.

    GNSS Receiver   |  ESP32
    ----------------------------------
    Serial TX       |  U0RXD
    Serial RX       |  (Not connected)
    GND             |  GND
    3.3 Volt        |  3.3 Volt

The receiver must the following serial port settings: 115200 baud, 8N1.

Power saving
------------
The firmware puts the device in deep sleep if the interval between images is
big. The camera is powered down by powering down the 1.2 and 2.8 Volt voltage
regulators. This is probably not according to spec. But the camera reset line
and power down line are not connected to the ESP32 on the ESP32-CAM boards. So
this seems the only way to do it.

Note that even in deep sleep mode the camera module still uses a little more
than 3 mA.

Troubleshooting
---------------
The only way to troubleshoot issues is to connect to the serial port and check
the logging.

The serial port uses the following settings: 115200 Baud, 8N1.
