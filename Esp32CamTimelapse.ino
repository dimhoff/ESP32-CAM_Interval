/**
 * Esp32CamTimelapse.c - Capture pictures to SD card at set interval.
 *
 * Based on the code from https://robotzero.one/time-lapse-esp32-cameras/
 * Original Copyright: Copyright (c) 2019, Robot Zero One
 *
 * Copyright (c) 2019, David Imhoff <dimhoff.devel@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "config.h"

#include "Arduino.h"

#include "esp_camera.h"

#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include <dirent.h>

// WIFI
#ifdef WITH_WIFI
# include <WiFi.h>

// SNTP
# include "lwip/err.h"
# include "lwip/apps/sntp.h"
#endif

// Config parser
#include "parse_kv_file.h"

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
#define CAMERA_MODEL_AI_THINKER

#define LED_GPIO_NUM 33
#define FLASH_GPIO_NUM 4

#include "camera_pins.h"

// Minimal unix time for clock to be considered valid.
#define NOT_BEFORE_TIME 1564437734

// Timelapse directory name format: /sdcard/timelapseXXXX/
#define CAPTURE_DIR_PREFIX "timelapse"
#define CAPTURE_DIR_PREFIX_LEN 9

// Config
#define CONFIG_PATH "/sdcard/camera.cfg"
struct s_config {
  unsigned int capture_interval;
        /**< Microseconds between captures.
         * Must be > 1000 if using timestamp filenames
         */
  bool enable_busy_led;
        /* Enable LED when taking a picture to indicate device is busy. */
  bool enable_flash;
        /* Enable Flash LED when taking a picture */

#ifdef WITH_WIFI
  char ssid[65]; /**< WiFi ssid to connect to */
  char password[65]; /**< WiFi Password */
  char ntp_server[65]; /**< NTP server address */
  char tzinfo[65]; /**< Timezone spec, see POSIX TZ(5) environment variable */
#endif // WITH_WIFI
} cfg = {
  5000,
  true,
  false,
#ifdef WITH_WIFI
  "",
  "",
  "pool.ntp.org",
  "GMT0",
#endif // WITH_WIFI
};

// Globals
static bool internet_connected = false;
static char capture_path[8 + CAPTURE_DIR_PREFIX_LEN + 4 + 1];

/************************ Initialization ************************/
void setup()
{
  Serial.begin(115200);
  Serial.println();

  // Init SD Card
  if (!init_sdcard()) {
    goto fail;
  }
  
#if !defined(WITH_SD_4BIT) && defined(CAMERA_MODEL_AI_THINKER)
  // WORKAROUND:
  // Force Flash LED off on AI Thinker boards.
  // This is needed because resistors R11, R12 and R13 form a voltage divider
  // that causes a voltage of about 0.57 Volt on the base of transistor Q1.
  // This will dimly light up the flash LED.
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);
#endif //!defined(WITH_SD_4BIT) && defined(CAMERA_MODEL_AI_THINKER)
  
  // Configure red LED
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, HIGH);
  
  // TODO: error log to file?

  // Load config file
  if (!init_config()) {
    goto fail;
  }

  // Init WiFi
#ifdef WITH_WIFI
  if (init_wifi()) { // Connected to WiFi
    internet_connected = true;
    init_time();
  }
#endif // WITH_WIFI

  // camera init
  if (!init_camera()) {
    goto fail;
  }

  if (!init_capture_dir()) {
    goto fail;
  }

  Serial.println("--- Initialization Done ---");

  return;

fail:
  while (true) {
    digitalWrite(LED_GPIO_NUM, LOW);
    delay(1000);
    digitalWrite(LED_GPIO_NUM, HIGH);
    delay(1000);
  }
}

/**
 * Initialize camera driver
 */
bool init_camera()
{
  esp_err_t err = ESP_FAIL;
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    Serial.println("PSRAM found, using UXGA frame size");
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    Serial.println("No PSRAM found, using SVGA frame size");
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }

  return true;
}

#ifdef WITH_WIFI
/**
 * Connect to WiFi
 */
bool init_wifi()
{
  // Return if no SSID configured
  if (cfg.ssid[0] == '\0') {
    return false;
  }
  
  Serial.printf("Connecting to '%s': ", cfg.ssid);
  WiFi.begin(cfg.ssid, cfg.password);

  // Wait for connection
  int connAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connAttempts < 10) {
    delay(500);
    Serial.print(".");
    connAttempts++;
  }
  if (connAttempts >= 10) {
    Serial.println("FAILED");
    return false;
  }
  Serial.println("Done");
  return true;
}

/**
 * Start NTP time synchronization
 */
void init_time()
{
  Serial.print("Waiting for time to synchronize: ");
  
  configTzTime(cfg.tzinfo, cfg.ntp_server);

  // wait for time to be set
  int retry = 0;
  const int retry_count = 10;
  time_t now = time(NULL);
  while (now < NOT_BEFORE_TIME && ++retry < retry_count) {
    delay(2000);
    (void) time(&now);
  }
  
  if (now > NOT_BEFORE_TIME) {
    Serial.println("Done");
    Serial.printf("Current time: %s", ctime(&now));
  } else {
    Serial.println("Timeout, continue in background");
  }
}
#endif // WITH_WIFI

/**
 * Mount SD Card
 */
static bool init_sdcard()
{
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 1,
  };
  sdmmc_card_t *card;
  
#ifndef WITH_SD_4BIT
  // Force host to 1-bit mode
  host.flags = SDMMC_HOST_FLAG_1BIT;
#endif

  Serial.print("Mounting SD card... ");
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config,
                                &card);
  if (ret == ESP_OK) {
    Serial.println("Done");
  }  else  {
    Serial.println("FAILED");
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s\n",
                     esp_err_to_name(ret));
    return false;
  }

  return true;
}

/**
 * Callback used by parse_kv_file() to set configuration options
 */
int config_set(const char *key, const char *value)
{
  char *endp = NULL;

  Serial.printf(" - '%s' => '%s'\n", key, value);
  
  if (strcasecmp(key, "interval") == 0) {
    cfg.capture_interval = strtoul(value, &endp, 10);
    if (endp == NULL) {
      Serial.println("Value for 'interval' is not a valid integer number");
      return -2;
    }
    if (cfg.capture_interval < 1000) {
      // Date/Time filename format doesn't support intervals < 1 Second.
      Serial.println("Capture interval to small, changing to 1 Sec.");
      cfg.capture_interval = 1000;
    }
#ifdef WITH_WIFI
  } else if (strcasecmp(key, "ssid") == 0) {
    if (strlen(value) > sizeof(cfg.ssid) - 1) {
      Serial.printf("Value of 'ssid' too long (>= %d byte)\n",
                        sizeof(cfg.ssid));
      return -2;
    }
    strcpy(cfg.ssid, value);
  } else if (strcasecmp(key, "password") == 0) {
    if (strlen(value) > sizeof(cfg.password) - 1) {
      Serial.printf("Value of 'password' too long (>= %d byte)\n",
                        sizeof(cfg.password));
      return -2;
    }
    strcpy(cfg.password, value);
  } else if (strcasecmp(key, "ntp_server") == 0) {
    if (strlen(value) > sizeof(cfg.ntp_server) - 1) {
      Serial.printf("Value of 'ntp_server' too long (>= %d byte)\n",
                        sizeof(cfg.ntp_server));
      return -2;
    }
    strcpy(cfg.ntp_server, value);
  } else if (strcasecmp(key, "timezone") == 0) {
    if (strlen(value) > sizeof(cfg.tzinfo) - 1) {
      Serial.printf("Value of 'tzinfo' too long (>= %d byte)\n",
                        sizeof(cfg.tzinfo));
      return -2;
    }
    strcpy(cfg.tzinfo, value);
#endif // WITH_WIFI
  } else if (strcasecmp(key, "enable_busy_led") == 0) {
    if (strcasecmp(value, "true") == 0 ||
        strcasecmp(value, "yes") == 0 ||
        strcmp(value, "1") == 0) {
      cfg.enable_busy_led = true;
    } else {
      cfg.enable_busy_led = false;
    }
  } else if (strcasecmp(key, "enable_flash") == 0) {
    if (strcasecmp(value, "true") == 0 ||
        strcasecmp(value, "yes") == 0 ||
        strcmp(value, "1") == 0) {
      cfg.enable_flash = true;
    } else {
      cfg.enable_flash = false;
    }
  } else {
    Serial.printf("Unknown key '%s', ignoring", key);
  }
  
  return 0;
}

/**
 * Load configuration file
 */
static bool init_config()
{
  FILE *file = fopen(CONFIG_PATH, "r");
  if (file != NULL)  {
    Serial.println("Loading config... ");
    
    int err = parse_kv_file(file, &config_set);

    fclose(file);

    if (err != 0) {
      Serial.printf("Failed to parse configuration, Error %d\n", err);
      return false;
    } else {
      Serial.println("Config loaded.");
    }
  } else {
    Serial.println("No config found, using defaults.");
  }

  return true;
}

/**
 * Create new directory to store images
 */
static bool init_capture_dir()
{
  DIR *dirp;
  struct dirent *dp;
  int max_idx = 0;

  // Find unused directory name
  if ((dirp = opendir("/sdcard/")) == NULL) {
    Serial.println("couldn't open directory /sdcard/");
    return -1;
  }

  do {
    errno = 0;
    if ((dp = readdir(dirp)) != NULL) {
      if (strlen(dp->d_name) != CAPTURE_DIR_PREFIX_LEN + 4)
        continue;
      if (strncmp(dp->d_name, CAPTURE_DIR_PREFIX,
          CAPTURE_DIR_PREFIX_LEN) != 0)
        continue;

      char *endp;
      int idx = strtoul(&(dp->d_name[CAPTURE_DIR_PREFIX_LEN]), &endp, 10);
      if (*endp != '\0')
        continue;

      if (idx > max_idx) {
        max_idx = idx;
      }
    }
  } while (dp != NULL);

  (void) closedir(dirp);

  if (errno != 0) {
    Serial.println("Error reading directory /sdcard/");
    return false;
  }

  // Create new dir
  snprintf(capture_path, sizeof(capture_path), "/sdcard/" CAPTURE_DIR_PREFIX "%04u", max_idx+1);
  
  if (mkdir(capture_path, 0644) != 0) {
    Serial.print("Failed to create directory: ");
    Serial.println(capture_path);
    return false;
  }

  Serial.print("Storing pictures in: ");
  Serial.println(capture_path);

  return true;
}

/************************ Main ************************/

/**
 * Take picture and save to SD card
 */
static void save_photo()
{
  if (cfg.enable_busy_led) {
    digitalWrite(LED_GPIO_NUM, LOW);
  }
  if (cfg.enable_flash) {
    digitalWrite(FLASH_GPIO_NUM, HIGH);
  }

  // Take picture
  Serial.print("Taking picture... ");
  camera_fb_t *fb = esp_camera_fb_get();

  // Generate filename
  time_t now = time(NULL);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  char filename[sizeof(capture_path) + 15 + 4 + 1];
  size_t capture_path_len = strlen(capture_path);
  strcpy(filename, capture_path);
  strftime(&filename[capture_path_len], sizeof(filename) - capture_path_len,
             "/%Y%m%d_%H%M%S.jpg", &timeinfo);

  // Save picture
  FILE *file = fopen(filename, "w");
  if (file != NULL)  {
    size_t ret = fwrite(fb->buf, fb->len, 1, file);
    if (ret != 1) {
      Serial.println("Failed\nError while writing to file");
    } else {
      Serial.printf("Saved as %s\n", filename);
    }
    fclose(file);
  } else {
    Serial.printf("Failed\nCould not open file: %s\n", filename);
  }
  esp_camera_fb_return(fb);
  
  if (cfg.enable_flash) {
    digitalWrite(FLASH_GPIO_NUM, LOW);
  }
  if (cfg.enable_busy_led) {
    digitalWrite(LED_GPIO_NUM, HIGH);
  }
}

void loop()
{
  static long current_millis;
  static long last_capture_millis = 0;

  current_millis = millis();
  if (current_millis - last_capture_millis > cfg.capture_interval) {
    // Take another picture
    last_capture_millis = current_millis;
    save_photo();
  }
}
