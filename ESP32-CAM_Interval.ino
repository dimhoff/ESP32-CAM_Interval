/**
 * ESP32-CAM_Interval.ino - Capture pictures to SD card at set interval.
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
#include <sys/time.h>
#include <sys/types.h>

// GPIO (rtc_gpio_hold_en())
#include "driver/rtc_io.h"

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
#endif // WITH_WIFI

// GNSS
#ifdef WITH_GNSS
# include <MicroNMEA.h>
#endif // WITH_GNSS

#include "camera_pins.h"

#include "configuration.h"
#include "exif.h"

// Check for incompatible configuration
#ifdef CAMERA_MODEL_AI_THINKER
#  if defined(WITH_SD_4BIT) && defined(WITH_CAM_PWDN)
#    error "WITH_SD_4BIT option is incompatible with the PWDN hack"
#  endif // defined(WITH_SD_4BIT) && defined(WITH_CAM_PWDN)

#  if defined(WITH_SD_4BIT) && defined(WITH_FLASH)
#    error "WITH_SD_4BIT option is incompatible with the WITH_FLASH option"
#  endif
#endif // CAMERA_MODEL_AI_THINKER
#ifndef CAMERA_MODEL_AI_THINKER
// Currently I developed on the ESP32-CAM board. The firmware should also work
// on other board but you have to check if the pin configuration and if the
// flash LED and camera power down work arounds are needed.
# error "Firmware currently only supports the AI-Thinker ESP32-CAM board."
#endif // !CAMERA_MODEL_AI_THINKER

#define LED_GPIO_NUM 33
#define FLASH_GPIO_NUM 4
#define CAM_PWR_GPIO_NUM 32
#ifdef WITH_CAM_PWDN
# undef PWDN_GPIO_NUM
# define PWDN_GPIO_NUM 32
#endif

#if defined(WITH_CAM_PWDN) && defined(WITH_EVIL_CAM_PWR_SHUTDOWN) && \
    PWDN_GPIO_NUM == CAM_PWR_GPIO_NUM
# error "PWDN_GPIO_NUM can not be equal to CAM_PWR_GPIO_NUM"
#endif

// Time unit defines
#define MSEC_AS_USEC (1000L)
#define SEC_AS_USEC (1000L * MSEC_AS_USEC)

// Minimum sleep time.
// If next capture is less then this many micro seconds away, then stay awake.
// FIXME: This can probably be reduced if WITH_EVIL_CAM_PWR_SHUTDOWN is
//        disabled, test this.
#define MIN_SLEEP_TIME (15 * SEC_AS_USEC)
// Wake-up this many micro seconds before capture time to allow initialization.
#define WAKE_USEC_EARLY (6 * SEC_AS_USEC)

#define GPS_NMEA_TIMEOUT_MS (15L * 1000L)
#define GPS_DATETIME_TIMEOUT_MS (120L * 1000L)

// Minimal unix time for clock to be considered valid.
#define NOT_BEFORE_TIME 1564437734

// Timelapse directory name format: /sdcard/timelapseXXXX/
#define CAPTURE_DIR_PREFIX "timelapse"
#define CAPTURE_DIR_PREFIX_LEN 9

// RTC memory storage
RTC_DATA_ATTR struct {
	struct timeval next_capture_time;
} nv_data;

// Globals
static char capture_path[8 + CAPTURE_DIR_PREFIX_LEN + 4 + 1];
static struct timeval capture_interval_tv;
static struct timeval next_capture_time;
#ifdef WITH_GNSS
char nmeaBuffer[255];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
#endif // WITH_GNSS

/************************ Initialization ************************/
void setup()
{
  bool time_set = false;
  bool is_wakeup = false;

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED) {
    is_wakeup = true;
  }

  Serial.begin(115200);
  Serial.println();

  // Init SD Card
  if (!init_sdcard()) {
    goto fail;
  }

  // Enable camera 1.2 and 2.8 Volt
  rtc_gpio_hold_dis(gpio_num_t(CAM_PWR_GPIO_NUM));
  pinMode(CAM_PWR_GPIO_NUM, OUTPUT);
  digitalWrite(CAM_PWR_GPIO_NUM, LOW);
  // TODO: maybe wait for voltage to stabelize?

#if PWDN_GPIO_NUM >= 0
  // Wakeup camera
  rtc_gpio_hold_dis(gpio_num_t(PWDN_GPIO_NUM));
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, LOW);
#endif // PWDN_GPIO_NUM >= 0

#ifdef WITH_FLASH
  // WORKAROUND:
  // Force Flash LED off on AI Thinker boards.
  // This is needed because resistors R11, R12 and R13 form a voltage divider
  // that causes a voltage of about 0.57 Volt on the base of transistor Q1.
  // This will dimly light up the flash LED.
  rtc_gpio_hold_dis(gpio_num_t(FLASH_GPIO_NUM));
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);
#endif // WITH_FLASH
  
  // Configure red LED
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, HIGH);

  // TODO: error log to file?

  // Load config file
  if (!cfg.loadConfig()) {
    goto fail;
  }
  update_exif_from_cfg(cfg);
  capture_interval_tv.tv_sec = cfg.getCaptureInterval() / 1000;
  capture_interval_tv.tv_usec = (cfg.getCaptureInterval() % 1000) * 1000;

  // Get current Time
#ifdef WITH_GNSS
  time_set = init_time_gnss();
#endif // WITH_GNSS
#ifdef WITH_WIFI
  if (!time_set) {
    if (init_wifi()) {
      time_set = init_time_ntp();
    }
  }
#endif // WITH_WIFI
  if (time_set) {
    time_t now = time(NULL);
    Serial.printf("Current time: %s", ctime(&now));
  } else {
    Serial.println("Failed to determine current date/time\n");
  }

  // Inititialize next capture time
  if (is_wakeup) {
    next_capture_time = nv_data.next_capture_time;
  } else {
    (void) gettimeofday(&next_capture_time, NULL);
  }
  Serial.printf("Next image at: %s", ctime(&next_capture_time.tv_sec));

  // Initialize capture directory
  if (!init_capture_dir(is_wakeup)) {
    goto fail;
  }

  // camera init
  if (!init_camera()) {
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

  int res;
  sensor_t *s = esp_camera_sensor_get();

  // Set configuration
  res = s->set_framesize(s, cfg.getFrameSize());
  if (res != 0) {
    Serial.printf("Unable to set 'frame size': return code %d\n", res);
    return false;
  }

  res = s->set_quality(s, cfg.getQuality());
  if (res != 0) {
    Serial.printf("Unable to set 'quality': return code %d\n", res);
    return false;
  }

  res = s->set_contrast(s, cfg.getContrast());
  if (res != 0) {
    Serial.printf("Unable to set 'contrast': return code %d\n", res);
    return false;
  }

  res = s->set_brightness(s, cfg.getBrightness());
  if (res != 0) {
    Serial.printf("Unable to set 'brightness': return code %d\n", res);
    return false;
  }

  res = s->set_saturation(s, cfg.getSaturation());
  if (res != 0) {
    Serial.printf("Unable to set 'saturation': return code %d\n", res);
    return false;
  }

  res = s->set_colorbar(s, cfg.getColorBar());
  if (res != 0) {
    Serial.printf("Unable to set 'colorbar': return code %d\n", res);
    return false;
  }

  res = s->set_hmirror(s, cfg.getHMirror());
  if (res != 0) {
    Serial.printf("Unable to set 'hmirror': return code %d\n", res);
    return false;
  }

  res = s->set_vflip(s, cfg.getVFlip());
  if (res != 0) {
    Serial.printf("Unable to set 'vflip': return code %d\n", res);
    return false;
  }

  res = s->set_whitebal(s, cfg.getAwb());
  if (res != 0) {
    Serial.printf("Unable to set 'whitebal': return code %d\n", res);
    return false;
  }

  res = s->set_awb_gain(s, cfg.getAwbGain());
  if (res != 0) {
    Serial.printf("Unable to set 'awb_gain': return code %d\n", res);
    return false;
  }

  res = s->set_wb_mode(s, cfg.getWhiteBalanceMode());
  if (res != 0) {
    Serial.printf("Unable to set 'wb_mode': return code %d\n", res);
    return false;
  }

  res = s->set_gain_ctrl(s, cfg.getAgc());
  if (res != 0) {
    Serial.printf("Unable to set 'gain_ctrl': return code %d\n", res);
    return false;
  }

  res = s->set_agc_gain(s, cfg.getAgcGain());
  if (res != 0) {
    Serial.printf("Unable to set 'agc_gain': return code %d\n", res);
    return false;
  }

  res = s->set_gainceiling(s, cfg.getGainCeiling());
  if (res != 0) {
    Serial.printf("Unable to set 'gainceiling': return code %d\n", res);
    return false;
  }

  res = s->set_exposure_ctrl(s, cfg.getAec());
  if (res != 0) {
    Serial.printf("Unable to set 'exposure_ctrl': return code %d\n", res);
    return false;
  }

  res = s->set_aec_value(s, cfg.getExposureValue());
  if (res != 0) {
    Serial.printf("Unable to set 'aec_value': return code %d\n", res);
    return false;
  }

  res = s->set_aec2(s, cfg.getAec2());
  if (res != 0) {
    Serial.printf("Unable to set 'aec2': return code %d\n", res);
    return false;
  }

  res = s->set_ae_level(s, cfg.getAeLevel());
  if (res != 0) {
    Serial.printf("Unable to set 'ae_level': return code %d\n", res);
    return false;
  }

  res = s->set_dcw(s, cfg.getDcw());
  if (res != 0) {
    Serial.printf("Unable to set 'dcw': return code %d\n", res);
    return false;
  }

  res = s->set_bpc(s, cfg.getBlackPixelCancellation());
  if (res != 0) {
    Serial.printf("Unable to set 'bpc': return code %d\n", res);
    return false;
  }

  res = s->set_wpc(s, cfg.getWhitePixelCancellation());
  if (res != 0) {
    Serial.printf("Unable to set 'wpc': return code %d\n", res);
    return false;
  }

  res = s->set_raw_gma(s, cfg.getRawGamma());
  if (res != 0) {
    Serial.printf("Unable to set 'raw_gma': return code %d\n", res);
    return false;
  }

  res = s->set_lenc(s, cfg.getLensCorrection());
  if (res != 0) {
    Serial.printf("Unable to set 'lenc': return code %d\n", res);
    return false;
  }

  res = s->set_special_effect(s, cfg.getSpecialEffect());
  if (res != 0) {
    Serial.printf("Unable to set 'special_effect': return code %d\n", res);
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
  if (cfg.getSsid()[0] == '\0') {
    return false;
  }
  
  Serial.printf("Connecting to '%s': ", cfg.getSsid());
  WiFi.begin(cfg.getSsid(), cfg.getPassword());

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
 *
 * @returns True on succes, false on failure
 */
bool init_time_ntp()
{
  Serial.print("Waiting for time to synchronize with NTP: ");
  
  configTzTime(cfg.getTzInfo(), cfg.getNtpServer());

  // wait for time to be set
  int retry = 0;
  const int retry_count = 10;
  time_t now = time(NULL);
  while (now < NOT_BEFORE_TIME && ++retry < retry_count) {
    delay(2000);
    (void) time(&now);
  }
  
  if (now < NOT_BEFORE_TIME) {
    Serial.println("Timeout, continue in background");
    return false;
  }

  Serial.println("Done");

  return true;
}
#endif // WITH_WIFI

#ifdef WITH_GNSS
/**
 * Get time from GNSS
 *
 * @returns True on succes, false on failure
 */
bool init_time_gnss()
{
  Serial.print("Waiting for time from GNSS: ");

  long start_millis;

  // Wait for some NMEA to be recognized
  start_millis = millis();
  while (millis() - start_millis < GPS_NMEA_TIMEOUT_MS) {
    while (Serial.available()) {
      char c = Serial.read();
      nmea.process(c);
    }

    // check if there is atleast some nmea
    if (nmea.getNavSystem()) {
      break;
    }
  }
  if (!nmea.getNavSystem()) {
    Serial.println("Failed, No NMEA data received");
    // TODO: maybe preserve this information across deep sleeps, so that GNSS
    // isn't tryed after deep sleep if no GNSS receiver is available.
    return false;
  }

  // Wait for a valid date/time
  bool have_time = false;
  start_millis = millis();
  while (millis() - start_millis < GPS_DATETIME_TIMEOUT_MS) {
    if (nmea.getYear() > 2018 && nmea.getYear() < 2038 ) {
      // NOTE: MTK33xx GPS starts at 2080, so also have an upper limit. By 2038
      // the time_t will probably overflow... so use that as limit.
      have_time = true;
      break;
    }

    while (Serial.available()) {
      char c = Serial.read();
      nmea.process(c);
    }
  }

  if (!have_time) {
    Serial.println("Failed, unable to get valid date from GPS");
    return false;
  }

  // Construct time
  struct tm tm;
  tm.tm_year = nmea.getYear() - 1900;
  tm.tm_mon = nmea.getMonth() - 1;
  tm.tm_mday = nmea.getDay();
  tm.tm_hour = nmea.getHour();
  tm.tm_min = nmea.getMinute();
  tm.tm_sec = nmea.getSecond();
  time_t t = mktime(&tm);
  if (t == (time_t) -1) {
    Serial.println("Failed, time could not be converted");
    return false;
  }

  // Set time
  struct timeval now = { t, 0 };
  if (settimeofday(&now, NULL) != 0) {
    Serial.println("Failed, unable to set time");
    return false;
  }

  setenv("TZ", cfg.getTzInfo(), 1);
  tzset();

  Serial.println("Done");

  return true;
}
#endif // WITH_GNSS

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
 * Create new directory to store images
 */
static bool init_capture_dir(bool reuse_last_dir)
{
  DIR *dirp;
  struct dirent *dp;
  int dir_idx = 0;

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

      if (idx > dir_idx) {
        dir_idx = idx;
      }
    }
  } while (dp != NULL);

  (void) closedir(dirp);

  if (errno != 0) {
    Serial.println("Error reading directory /sdcard/");
    return false;
  }

  if (!reuse_last_dir) {
    dir_idx += 1;
  }

  // Create new dir
  snprintf(capture_path, sizeof(capture_path),
	"/sdcard/" CAPTURE_DIR_PREFIX "%04u", dir_idx);
  
  if (!reuse_last_dir) {
    if (mkdir(capture_path, 0644) != 0) {
      Serial.print("Failed to create directory: ");
      Serial.println(capture_path);
      return false;
    }
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
  camera_fb_t *fb;

  if (cfg.getEnableBusyLed()) {
    digitalWrite(LED_GPIO_NUM, LOW);
  }
#ifdef WITH_FLASH
  if (cfg.getEnableFlash()) {
    digitalWrite(FLASH_GPIO_NUM, HIGH);
  }
#endif // WITH_FLASH

  // Take some shots to train the AGC/AWB
  Serial.print("Training:");
  for (int i=cfg.getTrainingShots(); i != 0; i--) {
    Serial.printf(" %d", i);
    fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
  }
  Serial.println(" Done");

  // Take picture
  Serial.print("Taking picture... ");
  fb = esp_camera_fb_get();

  // Disable Flash
#ifdef WITH_FLASH
  if (cfg.getEnableFlash()) {
    digitalWrite(FLASH_GPIO_NUM, LOW);
  }
#endif // WITH_FLASH

  // Generate filename
  time_t now = time(NULL);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  char filename[sizeof(capture_path) + 15 + 4 + 1];
  size_t capture_path_len = strlen(capture_path);
  strcpy(filename, capture_path);
  strftime(&filename[capture_path_len], sizeof(filename) - capture_path_len,
             "/%Y%m%d_%H%M%S.jpg", &timeinfo);

  // Generate Exif header
  const uint8_t *exif_header = NULL;
  size_t exif_len = 0;
#ifdef WITH_GNSS
  update_exif_gps(nmea);
#endif
  get_exif_header(fb, &exif_header, &exif_len);

  size_t data_offset = get_jpeg_data_offset(fb);

  // Save picture
  FILE *file = fopen(filename, "w");
  if (file != NULL)  {
    size_t ret = 0;
    if (exif_header != NULL) {
      ret = fwrite(exif_header, exif_len, 1, file);
      if (ret != 1) {
        Serial.println("Failed\nError while writing header to file");
        data_offset = 0;
      }
    } else {
        data_offset = 0;
    }

    ret = fwrite(&fb->buf[data_offset], fb->len - data_offset, 1, file);
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
  
  if (cfg.getEnableBusyLed()) {
    digitalWrite(LED_GPIO_NUM, HIGH);
  }
}

void loop()
{
  // Take picture if interval passed
  // NOTE: This breaks if clock jumps are introduced. Make sure to use
  // adjtime().
  struct timeval now;
  (void) gettimeofday(&now, NULL);
  if (!timercmp(&now, &next_capture_time, <)) {
    save_photo();

    timeradd(&next_capture_time, &capture_interval_tv, &next_capture_time);
  }

  // Process Serial input
#ifdef WITH_GNSS
  while (Serial.available()) {
    char c = Serial.read();
    nmea.process(c);
  }
#endif //WITH_GNSS

  // Sleep till next capture time
#ifdef WITH_SLEEP
  struct timeval time_to_next_capture;
  (void) gettimeofday(&now, NULL);
  if (timercmp(&now, &next_capture_time, <)) {
    timersub(&next_capture_time, &now, &time_to_next_capture);

    uint64_t sleep_time =  ((uint64_t) time_to_next_capture.tv_sec) *
                            SEC_AS_USEC + time_to_next_capture.tv_usec;

    // Wake earlier to allow initialization
    if (sleep_time > WAKE_USEC_EARLY) {
      sleep_time -= WAKE_USEC_EARLY;
    } else {
      sleep_time = 0;
    }

    if (sleep_time >= MIN_SLEEP_TIME) {
      Serial.printf("Sleeping for %llu us\n", sleep_time);
      Serial.flush();

      // Preserve non-volatile data
      nv_data.next_capture_time = next_capture_time;

      // Turn off camera power
      esp_camera_deinit();
#if PWDN_GPIO_NUM >= 0
      digitalWrite(PWDN_GPIO_NUM, HIGH); // esp_camera_deinit() doesn't power down camera...
#endif // PWDN_GPIO_NUM >= 0
#ifdef WITH_EVIL_CAM_PWR_SHUTDOWN
      digitalWrite(CAM_PWR_GPIO_NUM, HIGH);
#endif // WITH_CAM_PWR_SHUTDOWN

      // Lock pin states (need to be unlocked at init again)
#ifdef WITH_FLASH
      rtc_gpio_hold_en(gpio_num_t(FLASH_GPIO_NUM));
#endif // WITH_FLASH
      rtc_gpio_hold_en(gpio_num_t(CAM_PWR_GPIO_NUM));
#if PWDN_GPIO_NUM >= 0
      rtc_gpio_hold_en(gpio_num_t(PWDN_GPIO_NUM)); //TODO: is this needed???
#endif // PWDN_GPIO_NUM >= 0

      esp_sleep_enable_timer_wakeup(sleep_time);
      esp_deep_sleep_start();
      // This line will never be reached....
    }
  }
#endif // WITH_SLEEP
}
