/**
 * exif.c - Functions to generate Exif metadata
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

#include <time.h>
#include <sys/time.h>

#include "exif.h"
#include "exif_defines.h"

// TIFF header byte order
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
# define TIFF_BYTE_ORDER 0x4949
#else // __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
# define TIFF_BYTE_ORDER 0x4d4d
#endif // __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

// Reimplementation of htons() that can be used to initialize a struct member.
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
# define htons_macro(x) \
  (((x) & 0x00ff) << 8 | \
   ((x) & 0xff00) >> 8)
#else // __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
# define htons_macro(x) (x)
#endif // __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

/**
 * Type for storing Tiff Rational typed data
 */
#pragma pack(1)
typedef struct {
  uint32_t num;
  uint32_t denom;
} TiffRational;
#pragma pack()

/**
 * Type used for IFD entries
 *
 * This type is used to store a value within the Tiff IFD.
 */
#pragma pack(1)
typedef struct {
  uint16_t tag; // Data tag
  uint16_t type; // Data type
  uint32_t length; // length of data
  // Offset of data from start of TIFF data, or data it self if length <= 4.
  // Always use the IFD_SET_*() macros to modify the value.
  uint32_t value;
} IfdEntry;
#pragma pack()

// Some helper macros to initialize the IFD value's. The types shorter then
// 4-bytes need to be aligned such that they are directly after the previous
// field. These macros take care of this.
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
# define IFD_SET_BYTE(VAL)   (VAL)
# define IFD_SET_SHORT(VAL)  (VAL)
# define IFD_SET_LONG(VAL)   (VAL)
# define IFD_SET_OFFSET(REF, VAR) offsetof(REF, VAR)
# define IFD_SET_UNDEF(V0, V1, V2, V3) \
                             (((V3) & 0xff) << 24 | \
                              ((V2) & 0xff) << 16 | \
                              ((V1) & 0xff) << 8 | \
                              ((V0) & 0xff))
#else // __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
# define IFD_SET_BYTE(VAL)   (((VAL) & 0xff) << 24)
# define IFD_SET_SHORT(VAL)  (((VAL) & 0xffff) << 16)
# define IFD_SET_LONG(VAL)   (VAL)
# define IFD_SET_OFFSET(REF, VAR) offsetof(REF, VAR)
# define IFD_SET_UNDEF(V0, V1, V2, V3) \
                             (((V0) & 0xff) << 24 | \
                              ((V1) & 0xff) << 16 | \
                              ((V2) & 0xff) << 8 | \
                              ((V3) & 0xff))
#endif // __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

// Amount of entries in the 0th IFD
#define IFD0_ENTRY_CNT 10

// Amount of entries in the Exif private IFD
#define IFD_EXIF_ENTRY_CNT 6

// Camera maker string
#define CAMERA_MAKE "OmniVision"

// Camera model string
#define CAMERA_MODEL "OV2640"

// Camera Software string
#define CAMERA_SOFTWARE "ESP32-CAM Interval v" VERSION_STR

/**
 * New Jpeg/Exif header
 *
 * This defines the new JPEG/Exif header that is added to the images. To keep
 * it simple, the structure of the header is completely static.
 */
#pragma pack(1)
struct JpegExifHdr {
  uint16_t jpeg_soi; // htons(0xffd8)
  uint16_t marker; // htons(0xffe1)
  uint16_t len;    // htons(length)
  uint8_t exif_identifier[6]; // 'Exif\0\0'

  struct TiffData {
    struct {
      uint16_t byte_order; // 'II' || 'MM'
      uint16_t signature; // 0x002A
      uint32_t ifd_offset; // 0x8
    } tiff_hdr;
    struct {
      uint16_t cnt; // amount of entries
      IfdEntry entries[IFD0_ENTRY_CNT];
      uint32_t next_ifd; // Offset of next IFD, or 0x0 if last IFD
    } ifd0;
    struct {
      TiffRational XYResolution;
      char make[sizeof(CAMERA_MAKE)];
      char model[sizeof(CAMERA_MODEL)];
      char software[sizeof(CAMERA_SOFTWARE)];
      char datetime[20];
    } ifd0_data;
    struct {
      uint16_t cnt; // amount of entries
      IfdEntry entries[IFD_EXIF_ENTRY_CNT];
      uint32_t next_ifd; // Offset of next IFD, or 0x0 if last IFD
    } ifd_exif;
  } tiff_data;
} exiv_hdr = {
  htons_macro(0xffd8),
  htons_macro(0xffe1),
  htons_macro(sizeof(JpegExifHdr) - offsetof(JpegExifHdr, len)),
  { 'E', 'x', 'i', 'f', 0, 0 },
  {
    .tiff_hdr = { TIFF_BYTE_ORDER, 0x002A, 0x8 },
    .ifd0 = {
      .cnt = IFD0_ENTRY_CNT,
      .entries = {
        { TagTiffMake,
          TiffTypeAscii,
          sizeof(exiv_hdr.tiff_data.ifd0_data.make),
          IFD_SET_OFFSET(JpegExifHdr::TiffData, ifd0_data.make) },
        { TagTiffModel,
          TiffTypeAscii,
          sizeof(exiv_hdr.tiff_data.ifd0_data.model),
          IFD_SET_OFFSET(JpegExifHdr::TiffData, ifd0_data.model) },
        { TagTiffOrientation,
          TiffTypeShort, 1,
          IFD_SET_SHORT(1) },
        { TagTiffXResolution,
          TiffTypeRational, 1,
          IFD_SET_OFFSET(JpegExifHdr::TiffData, ifd0_data) },
        { TagTiffYResolution,
          TiffTypeRational, 1,
          IFD_SET_OFFSET(JpegExifHdr::TiffData, ifd0_data) },
        { TagTiffResolutionUnit,
          TiffTypeShort, 1,
          IFD_SET_SHORT(0x0002) },
        { TagTiffSoftware,
          TiffTypeAscii,
          sizeof(exiv_hdr.tiff_data.ifd0_data.software),
          IFD_SET_OFFSET(JpegExifHdr::TiffData, ifd0_data.software) },
        { TagTiffDateTime,
          TiffTypeAscii,
          sizeof(exiv_hdr.tiff_data.ifd0_data.datetime),
          IFD_SET_OFFSET(JpegExifHdr::TiffData, ifd0_data.datetime) },
        { TagTiffYCbCrPositioning,
          TiffTypeShort, 1,
          IFD_SET_SHORT(0x0001) },
        { TagTiffExifIFD,
          TiffTypeLong, 1,
          IFD_SET_OFFSET(JpegExifHdr::TiffData, ifd_exif) },
      },
      .next_ifd = 0
    },
    .ifd0_data = {
      { 72, 1 },
      CAMERA_MAKE,
      CAMERA_MODEL,
      CAMERA_SOFTWARE,
      "    :  :     :  :  ",
    },
    .ifd_exif = {
      .cnt = IFD_EXIF_ENTRY_CNT,
      .entries = {
        { TagExifVersion,
          TiffTypeUndef, 4,
          IFD_SET_UNDEF(0x30, 0x32, 0x33, 0x30) },
        { TagExifComponentsConfiguration,
          TiffTypeUndef, 4,
          IFD_SET_UNDEF(0x01, 0x02, 0x03, 0x00) },
#define TAG_EXIF_SUBSEC_TIME_IDX 2
        { TagExifSubSecTime,
          TiffTypeAscii, 4,
          IFD_SET_UNDEF(0x20, 0x20, 0x20, 0x00) },
        { TagExifColorSpace,
          TiffTypeShort, 1,
          IFD_SET_SHORT(1) },
#define TAG_EXIF_PIXEL_X_DIMENSION_IDX 4
        { TagExifPixelXDimension,
          TiffTypeShort, 1,
          IFD_SET_SHORT(1600) },
#define TAG_EXIF_PIXEL_Y_DIMENSION_IDX (TAG_EXIF_PIXEL_X_DIMENSION_IDX + 1)
        { TagExifPixelYDimension,
          TiffTypeShort, 1,
          IFD_SET_SHORT(1200) },
      },
      .next_ifd = 0
    }
  }
};
#pragma pack()

const uint8_t *get_exiv_header(camera_fb_t *fb, const uint8_t **exiv_buf, size_t *exiv_len)
{
  // TODO: pass config to function and use that to set some of the image
  // taking conditions. Or do this only once, with a update config
  // function????

  // Get current time
  struct timeval now_tv;
  if (gettimeofday(&now_tv, NULL) != 0) {
    now_tv.tv_sec = time(NULL);
    now_tv.tv_usec = 0;
  }

  // Set date time
  struct tm timeinfo;
  localtime_r(&now_tv.tv_sec, &timeinfo);
  strftime(exiv_hdr.tiff_data.ifd0_data.datetime,
      sizeof(exiv_hdr.tiff_data.ifd0_data.datetime),
      "%Y:%m:%d %H:%M:%S", &timeinfo);

  // Set sub-seconds time
  snprintf(
      (char *) &(exiv_hdr.tiff_data.ifd_exif.entries[TAG_EXIF_SUBSEC_TIME_IDX].value),
      4,
      "%03ld",
      now_tv.tv_usec/1000);

  // Update image dimensions
  exiv_hdr.tiff_data.ifd_exif.entries[TAG_EXIF_PIXEL_X_DIMENSION_IDX].value = IFD_SET_SHORT(fb->width);
  exiv_hdr.tiff_data.ifd_exif.entries[TAG_EXIF_PIXEL_Y_DIMENSION_IDX].value = IFD_SET_SHORT(fb->height);

  *exiv_len = sizeof(exiv_hdr);

  if (exiv_buf != NULL) {
    *exiv_buf = (uint8_t *) &exiv_hdr;
  }

  return (uint8_t *) &exiv_hdr;
}

size_t get_jpeg_data_offset(camera_fb_t *fb)
{
  if (fb->len < 6) {
    return 0;
  }

  // Check JPEG SOI
  if (fb->buf[0] != 0xff || fb->buf[1] != 0xd8) {
    return 0;
  }
  size_t data_offset = 2; // Offset to first JPEG segment after header

  // Check if JFIF header
  if (fb->buf[2] == 0xff && fb->buf[3] == 0xe0) {
    uint16_t jfif_len = fb->buf[4] << 8 | fb->buf[5];

    data_offset += 2 + jfif_len;
  }

  if (data_offset >= fb->len) {
    return 0;
  }

  return data_offset;
}