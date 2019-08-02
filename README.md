Esp32-Cam Timelapse camera
==========================
This firmware turns a Esp32-Cam module into a timelapse camera. Taking pictures
at a set interval and storing them to SD card.

Compilation
-----------
Use the Arduino IDE with ESP32 support to compile the firmware.

Picture Names
-------------
Every time the device boots a new directory is created on the SD card. The
directory name is created following the template 'timelapseXXXX', where 'XXXX'
is replaced by a free sequence number. Pictures are stored to this directory.

The picture filenames contain the date and time of taking the pictures. For the
clock the be set correctly NTP can be used. For this Wi-Fi needs to be
configured correctly and be available at boot time. If no NTP is available the
clock will start at UNIX epoch, i.e. 01-01-1970 00:00:00.

Configuration
-------------
To configure the software create a file named camera.cfg in the root
directory of the SD card. The configuration file is text based containing one
configuration option per line. Configuration options have the format
'key=value'. Spaces are striped from the front and back of both the key and
value.

The following configuration keys are available:

 - interval: interval in milliseconds between taking pictures (default: 5000)
 - ssid: Wi-Fi SSID, an empty value disables Wi-Fi (default: _empty_)
 - password: Wi-Fi Password (default: _empty_)
 - ntp_server: NTP server address (default: pool (default: pool.ntp.org)
 - timezone: Timezone spec. See POSIX TZ(5) environment variable (default: GMT0)
 - enable_busy_led: Enable small LED when taking picture (default: true)
 - enable_flash: Enable Flash LED when taking picture (default: false)

Lines starting with a '#' and empty lines are ignored.

Troubleshooting
---------------
The only way to troubleshoot issues is to connect to the serial port and check
the logging.

The serial port uses the following settings: 115200 Baud, 8N1.
