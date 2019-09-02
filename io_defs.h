/**
 * io_defs.h - I/O pin defines
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
#ifndef __IO_DEFS_H__
#define __IO_DEFS_H__

#include "camera_pins.h"

#define LED_GPIO_NUM 33
#define FLASH_GPIO_NUM 4
#define CAM_PWR_GPIO_NUM 32
#ifdef WITH_CAM_PWDN
# undef PWDN_GPIO_NUM
# define PWDN_GPIO_NUM 32
#endif

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

#if defined(WITH_CAM_PWDN) && defined(WITH_EVIL_CAM_PWR_SHUTDOWN) && \
    PWDN_GPIO_NUM == CAM_PWR_GPIO_NUM
# error "PWDN_GPIO_NUM can not be equal to CAM_PWR_GPIO_NUM"
#endif

#endif // __IO_DEFS_H__
