/**
 * camera.cpp - Camera control functions
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
#include "driver/rtc_io.h" // rtc_gpio_hold_en()

#include "camera.h"
#include "io_defs.h"
#include "configuration.h"

static bool camera_set_config(void);

/**
 * Configure the camera based on current system configuration
 */
static bool camera_set_config(void)
{
  int res;
  sensor_t *s = esp_camera_sensor_get();

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

bool camera_init()
{
  esp_err_t err = ESP_FAIL;
  camera_config_t config;

  // Enable camera 1.2 and 2.8 Volt
  rtc_gpio_hold_dis(gpio_num_t(CAM_PWR_GPIO_NUM)); // TODO: move to after init to prevent glitch???
  pinMode(CAM_PWR_GPIO_NUM, OUTPUT);
  digitalWrite(CAM_PWR_GPIO_NUM, LOW);
  // TODO: maybe wait for voltage to stabelize?

#if PWDN_GPIO_NUM >= 0
  // Wakeup camera
  rtc_gpio_hold_dis(gpio_num_t(PWDN_GPIO_NUM)); // TODO: move to after init to prevent glitch???
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, LOW);
#endif // PWDN_GPIO_NUM >= 0

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

  return camera_set_config();
}

void camera_deinit()
{
  // Turn off camera power
  esp_camera_deinit();
#if PWDN_GPIO_NUM >= 0
  digitalWrite(PWDN_GPIO_NUM, HIGH); // esp_camera_deinit() doesn't power down camera...
#endif // PWDN_GPIO_NUM >= 0
#ifdef WITH_EVIL_CAM_PWR_SHUTDOWN
  digitalWrite(CAM_PWR_GPIO_NUM, HIGH);
#endif // WITH_CAM_PWR_SHUTDOWN
}

camera_fb_t *camera_capture()
{
  camera_fb_t *fb;

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

  return fb;
}
