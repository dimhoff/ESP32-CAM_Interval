/**
 * configuration.h - Configuration parsing and storage
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
#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include <stdint.h>
#include <stdbool.h>

#include "WString.h"
#include "esp_camera.h"

#define CONFIG_PATH "/sdcard/camera.cfg"

class Configuration {
public:
  enum WbMode {
    WbModeAuto=0,
    WbModeSunny=1,
    WbModeCloudy=2,
    WbModeOffice=3,
    WbModeHome=4
  };
  enum SpecialEffect {
    SpecialEffectNone=0,
    SpecialEffectNegative=1,
    SpecialEffectGrayscale=2,
    SpecialEffectRedTint=3,
    SpecialEffectGreenTint=4,
    SpecialEffectBlueTint=5,
    SpecialEffectSepia=6
  };

  Configuration() :
    m_capture_interval(5000),
    m_enable_busy_led(true),
    m_enable_flash(false),
    m_training_shots(0),
    m_tzinfo("GMT0"),
    m_orientation(1),
    m_frame_size(FRAMESIZE_UXGA),
    m_quality(10),
    m_contrast(0),
    m_brightness(0),
    m_saturation(0),
    m_colorbar(false),
    m_hmirror(false),
    m_vflip(false),
    m_awb(true),
    m_awb_gain(true),
    m_wb_mode(WbModeAuto),
    m_agc(true),
    m_agc_gain(1),
    m_gainceiling(GAINCEILING_8X),
    m_aec(true),
    m_aec_value(51),
    m_aec2(true),
    m_ae_level(0),
    m_dcw(true),
    m_bpc(false),
    m_wpc(true),
    m_raw_gma(true),
    m_lenc(true),
    m_special_effect(SpecialEffectNone)
  {};

  bool loadConfig();
  bool saveConfig();

  String configAsJSON() const;

  unsigned int getCaptureInterval() const { return m_capture_interval; }
  bool getEnableBusyLed() const { return m_enable_busy_led; }
  bool getEnableFlash() const { return m_enable_flash; }
  unsigned int getTrainingShots() const { return m_training_shots; };

  const char *getTzInfo() const { return m_tzinfo; }

  uint8_t getOrientation() const { return m_orientation; }

  framesize_t getFrameSize() const { return m_frame_size; }
  int8_t getQuality() const { return m_quality; }
  int8_t getContrast() const { return m_contrast; }
  int8_t getBrightness() const { return m_brightness; }
  int8_t getSaturation() const { return m_saturation; }
  bool getColorBar() const { return m_colorbar; }
  bool getHMirror() const { return m_hmirror; }
  bool getVFlip() const { return m_vflip; }
  bool getAwb() const { return m_awb; }
  bool getAwbGain() const { return m_awb_gain; }
  WbMode getWhiteBalanceMode() const { return m_wb_mode; }
  bool getAgc() const { return m_agc; }
  uint8_t getAgcGain() const { return m_agc_gain; }
  gainceiling_t getGainCeiling() const { return m_gainceiling; }
  bool getAec() const { return m_aec; }
  uint16_t getExposureValue() const { return m_aec_value; }
  bool getAec2() const { return m_aec2; }
  int8_t getAeLevel() const { return m_ae_level; }
  bool getDcw() const { return m_dcw; }
  bool getBlackPixelCancellation() const { return m_bpc; }
  bool getWhitePixelCancellation() const { return m_wpc; }
  bool getRawGamma() const { return m_raw_gma; }
  bool getLensCorrection() const { return m_lenc; }
  SpecialEffect getSpecialEffect() const { return m_special_effect; }

  /**
   * Callback used by parse_kv_file() to set configuration options
   */
  int config_set(const char *key, const char *value);

private:
  // Generic options
  unsigned int m_capture_interval;
        /**< Microseconds between captures.
         * Must be > 1000 if using timestamp filenames
         */
  bool m_enable_busy_led;
        /* Enable LED when taking a picture to indicate device is busy. */
  bool m_enable_flash;
        /* Enable Flash LED when taking a picture */
  unsigned int m_training_shots;
        /* Amount of images to take before the real shot to train the AGC/AWB */

  char m_tzinfo[65]; /**< Timezone spec, see POSIX TZ(5) environment variable */

  // Exiv options
  uint8_t m_orientation;

  // Camera options
  framesize_t m_frame_size;
  int8_t m_quality;
  int8_t m_contrast;
  int8_t m_brightness;
  int8_t m_saturation;
  bool m_colorbar;
  bool m_hmirror;
  bool m_vflip;
  bool m_awb;
  bool m_awb_gain;
  WbMode m_wb_mode;
  bool m_agc;
  uint8_t m_agc_gain;
  gainceiling_t m_gainceiling;
  bool m_aec;
  uint16_t m_aec_value;
  bool m_aec2;
  int8_t m_ae_level;
  bool m_dcw;
  bool m_bpc;
  bool m_wpc;
  bool m_raw_gma;
  bool m_lenc;
  SpecialEffect m_special_effect;
};

extern Configuration cfg;

#endif // __CONFIGURATION_H__
