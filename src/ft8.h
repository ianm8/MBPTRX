#pragma once
//==============================================================================
// ft8.h  -  Complete FT8 codec, single-header version
// Derived from kgoba/ft8_lib (MIT License, see below), restructured for
// Arduino/Pico SDK by Ian Mitchell VK7IAN
//
// USAGE:
//   In exactly ONE .c/.cpp file:
//       #define FT8_CODEC_IMPLEMENTATION
//       #include "ft8.h"
//   In all other files just:
//       #include "ft8.h"
//
// FFT:
//   fft2500.h is included and provides ft8_fft_forward().
//   hann_window[] must be declared externally (static const float[2500]).
//
// NOTES:
//   - FT4 support removed.
//   - KissFFT removed; monitor calls ft8_fft_forward() from fft2500.h.
//   - All pipeline parameters are fixed constants (no runtime config struct).
//   - No heap allocation anywhere.
//   - LDPC working arrays are static (~115 KB BSS, not stack).
//==============================================================================

/*
MIT License

Copyright (c) 2018 Kārlis Goba

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

//------------------------------------------------------------------------------
// SECTION 1 - INCLUDES
//------------------------------------------------------------------------------
#include "fft2500.h"

//------------------------------------------------------------------------------
// SECTION 2 - MSVC / POSIX COMPATIBILITY
//------------------------------------------------------------------------------
#ifdef _WIN32
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>

static int clock_gettime_win(struct timespec* spec)
{
  FILETIME ft;
  GetSystemTimeAsFileTime(&ft);
  ULARGE_INTEGER uli;
  uli.LowPart = ft.dwLowDateTime;
  uli.HighPart = ft.dwHighDateTime;
  uli.QuadPart -= 116444736000000000ULL;
  spec->tv_sec = (long)(uli.QuadPart / 10000000ULL);
  spec->tv_nsec = (long)((uli.QuadPart % 10000000ULL) * 100);
  return 0;
}
#define clock_gettime(clk, spec) clock_gettime_win(spec)
#define gmtime_r(timep, result)  gmtime_s(result, timep)

static char* stpcpy(char* dst, const char* src)
{
  while ((*dst++ = *src++));
  return dst - 1;
}

#ifdef NDEBUG
#define assert(expr) ((void)((expr) || \
    (fprintf(stderr, "Assert failed: %s line %d\n", __FILE__, __LINE__), exit(1), 0)))
#else
#define assert(expr) ((void)((expr) || (__debugbreak(), 0)))
#endif

#ifndef static_assert
#define static_assert(expr, msg) typedef char static_assert_##__LINE__[(expr) ? 1 : -1]
#endif

#else
#include <assert.h>
#endif // _WIN32

#define FT8_MAX_DECODED 50

//------------------------------------------------------------------------------
// SECTION 3 - FIXED PIPELINE CONSTANTS
//
// All DSP parameters are compile-time constants.
// Input sample rate is 7812.5 Hz (decimated from 31250 Hz by factor 4).
// Frequency range analysed: 200..3000 Hz.
//------------------------------------------------------------------------------

// Core timing / FFT
#define FT8_SYMBOL_PERIOD   (0.160f)    ///< FT8 symbol duration, seconds
#define FT8_SLOT_TIME       (15.0f)     ///< FT8 slot period, seconds
#define FT8_BLOCK_SIZE      1250        ///< Real samples per symbol (7812.5 * 0.160)
#define FT8_FFT_SIZE        2500        ///< 2x zero-padded FFT size
#define FT8_FFT_NORM        (2.0f / FT8_FFT_SIZE)  ///< FFT normalisation factor = 0.0008

// Oversampling
#define FT8_TIME_OSR        2           ///< Time subdivisions per symbol
#define FT8_FREQ_OSR        2           ///< Frequency subdivisions per bin
#define FT8_SUBBLOCK_SIZE   (FT8_BLOCK_SIZE / FT8_TIME_OSR)  ///< 625 samples

// Waterfall frequency range (200..3000 Hz)
#define FT8_F_MIN           200.0f
#define FT8_F_MAX           3000.0f
#define FT8_MIN_BIN         32          ///< (int)(200  * 0.160)
#define FT8_MAX_BIN         481         ///< (int)(3000 * 0.160) + 1
#define FT8_WF_NUM_BINS     (FT8_MAX_BIN - FT8_MIN_BIN)  ///< 449 bins

// Waterfall block geometry
#define FT8_MAX_BLOCKS      93          ///< (int)(15.0 / 0.160)
#define FT8_BLOCK_STRIDE    (FT8_TIME_OSR * FT8_FREQ_OSR * FT8_WF_NUM_BINS)  ///< 1796
#define FT8_WATERFALL_SIZE  (FT8_MAX_BLOCKS * FT8_BLOCK_STRIDE)              ///< 167028

// Decoder tuning (adjust for speed vs sensitivity)
#define FT8_MAX_CANDIDATES  80          ///< Max sync candidates to evaluate
#define FT8_LDPC_ITERATIONS 20          ///< LDPC belief-propagation iterations
#define FT8_MIN_SCORE       10          ///< Minimum sync score to accept

//------------------------------------------------------------------------------
// SECTION 4 - PROTOCOL CONSTANTS
//------------------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LOG_DEBUG 0
#define LOG_INFO  1
#define LOG_WARN  2
#define LOG_ERROR 3
#define LOG_FATAL 4

#ifdef LOG_LEVEL
#ifndef LOG_PRINTF
#define LOG_PRINTF(...) fprintf(stderr, __VA_ARGS__)
#endif
#define LOG(level, ...)        \
  do {                         \
    if ((level) >= LOG_LEVEL)  \
      LOG_PRINTF(__VA_ARGS__); \
  } while (0)
#else
#define LOG(level, ...) do {} while (0)
#endif

#define FT8_ND            (58)    ///< FT8 data symbols
#define FT8_NN            (79)    ///< FT8 total channel symbols
#define FT8_LENGTH_SYNC   (7)     ///< Costas sync group length
#define FT8_NUM_SYNC      (3)     ///< Number of sync groups
#define FT8_SYNC_OFFSET   (36)    ///< Offset between sync groups

#define FTX_LDPC_N        (174)
#define FTX_LDPC_K        (91)
#define FTX_LDPC_M        (83)
#define FTX_LDPC_N_BYTES  ((FTX_LDPC_N + 7) / 8)
#define FTX_LDPC_K_BYTES  ((FTX_LDPC_K + 7) / 8)

#define FT8_CRC_POLYNOMIAL ((uint16_t)0x2757u)
#define FT8_CRC_WIDTH      (14)

//------------------------------------------------------------------------------
// SECTION 5 - MESSAGE TYPES
//------------------------------------------------------------------------------
#define FTX_PAYLOAD_LENGTH_BYTES 10
#define FTX_MAX_MESSAGE_LENGTH   35
#define FTX_MAX_DISPLAY_LENGTH   64

typedef struct
{
  uint8_t  payload[FTX_PAYLOAD_LENGTH_BYTES];
  uint16_t hash;
} ftx_message_t;

typedef enum
{
  FTX_MESSAGE_TYPE_FREE_TEXT,
  FTX_MESSAGE_TYPE_DXPEDITION,
  FTX_MESSAGE_TYPE_EU_VHF,
  FTX_MESSAGE_TYPE_ARRL_FD,
  FTX_MESSAGE_TYPE_TELEMETRY,
  FTX_MESSAGE_TYPE_CONTESTING,
  FTX_MESSAGE_TYPE_STANDARD,
  FTX_MESSAGE_TYPE_ARRL_RTTY,
  FTX_MESSAGE_TYPE_NONSTD_CALL,
  FTX_MESSAGE_TYPE_WWROF,
  FTX_MESSAGE_TYPE_UNKNOWN
} ftx_message_type_t;

typedef enum
{
  FTX_CALLSIGN_HASH_22_BITS,
  FTX_CALLSIGN_HASH_12_BITS,
  FTX_CALLSIGN_HASH_10_BITS
} ftx_callsign_hash_type_t;

typedef enum
{
  FTX_MESSAGE_RC_OK,
  FTX_MESSAGE_RC_ERROR_CALLSIGN1,
  FTX_MESSAGE_RC_ERROR_CALLSIGN2,
  FTX_MESSAGE_RC_ERROR_SUFFIX,
  FTX_MESSAGE_RC_ERROR_GRID,
  FTX_MESSAGE_RC_ERROR_TYPE
} ftx_message_rc_t;

//------------------------------------------------------------------------------
// SECTION 6 - WATERFALL AND DECODE TYPES
//
// Waterfall magnitudes stored as uint8: value = 2*dB + 240
// Range 0..240 covers -120..0 dB in 0.5 dB steps.
//
// All geometry (num_bins, time_osr, freq_osr, block_stride) is now encoded
// in the compile-time constants FT8_WF_NUM_BINS, FT8_TIME_OSR, etc.
//------------------------------------------------------------------------------
#define WF_ELEM_T          uint8_t
#define WF_ELEM_MAG(x)     ((float)(x) * 0.5f - 120.0f)
#define WF_ELEM_MAG_INT(x) ((int)(x))

typedef struct
{
  int num_blocks; // blocks filled so far (max FT8_MAX_BLOCKS)
  WF_ELEM_T* mag; // [num_blocks][FT8_TIME_OSR][FT8_FREQ_OSR][FT8_WF_NUM_BINS]
} ftx_waterfall_t;

typedef struct
{
  int16_t score;
  int16_t time_offset;
  int16_t freq_offset;
  uint8_t time_sub;
  uint8_t freq_sub;
} ftx_candidate_t;

typedef struct
{
  float    freq;
  float    time;
  int      ldpc_errors;
  uint16_t crc_extracted;
  uint16_t crc_calculated;
} ftx_decode_status_t;

//------------------------------------------------------------------------------
// SECTION 7 - MONITOR TYPE
//
// Manages the DSP pipeline: audio blocks -> FFT -> waterfall.
// All geometry fields have been replaced by compile-time constants.
// last_frame points to a static buffer allocated inside monitor_init.
//------------------------------------------------------------------------------
typedef struct
{
  float* last_frame;  ///< Current analysis frame [FT8_FFT_SIZE]
  ftx_waterfall_t wf;
  float max_mag;     ///< Peak magnitude seen (dB), for diagnostics
} monitor_t;

//------------------------------------------------------------------------------
// SECTION 8 - PUBLIC API
//------------------------------------------------------------------------------
// decode callback
typedef void (*ft8_ui_cb_t)(void);

typedef enum
{
  FT8_CHAR_TABLE_FULL,
  FT8_CHAR_TABLE_ALPHANUM_SPACE_SLASH,
  FT8_CHAR_TABLE_ALPHANUM_SPACE,
  FT8_CHAR_TABLE_LETTERS_SPACE,
  FT8_CHAR_TABLE_ALPHANUM,
  FT8_CHAR_TABLE_NUMERIC,
} ft8_char_table_e;

//------------------------------------------------------------------------------
// Callsign hash table
//------------------------------------------------------------------------------
#define CALLSIGN_HASHTABLE_SIZE 256

static struct
{
  char callsign[12];
  uint32_t hash;
}
callsign_hashtable[CALLSIGN_HASHTABLE_SIZE];
static int callsign_hashtable_size = 0;

static void ft8_hashtable_init(void)
{
  callsign_hashtable_size = 0;
  memset(callsign_hashtable, 0, sizeof(callsign_hashtable));
}

static void ft8_hashtable_cleanup(uint8_t max_age)
{
  for (int i = 0; i < CALLSIGN_HASHTABLE_SIZE; ++i)
  {
    if (callsign_hashtable[i].callsign[0] == '\0') continue;
    uint8_t age = (uint8_t)(callsign_hashtable[i].hash >> 24);
    if (age > max_age)
    {
      LOG(LOG_INFO, "Removing [%s] from hash table, age = %d\n",
        callsign_hashtable[i].callsign, age);
      callsign_hashtable[i].callsign[0] = '\0';
      callsign_hashtable[i].hash = 0;
      callsign_hashtable_size--;
    }
    else
    {
      callsign_hashtable[i].hash =
        (((uint32_t)age + 1u) << 24) | (callsign_hashtable[i].hash & 0x3FFFFFu);
    }
  }
}

static void ft8_hashtable_add(const char* callsign, uint32_t hash)
{
  uint16_t hash10 = (hash >> 12) & 0x3FFu;
  uint32_t idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
  while (callsign_hashtable[idx].callsign[0] != '\0')
  {
    if (((callsign_hashtable[idx].hash & 0x3FFFFFu) == hash) &&
      (0 == strcmp(callsign_hashtable[idx].callsign, callsign)))
    {
      callsign_hashtable[idx].hash &= 0x3FFFFFu;   // reset age
      LOG(LOG_DEBUG, "Found a duplicate [%s]\n", callsign);
      return;
    }
    LOG(LOG_DEBUG, "Hash table clash!\n");
    idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
  }
  callsign_hashtable_size++;
  strncpy(callsign_hashtable[idx].callsign, callsign, 11);
  callsign_hashtable[idx].callsign[11] = '\0';
  callsign_hashtable[idx].hash = hash;
}

static const bool ft8_hashtable_lookup(ftx_callsign_hash_type_t hash_type, uint32_t hash, char* callsign)
{
  uint8_t  shift = (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? 12
    : (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? 10 : 0;
  uint16_t hash10 = (hash >> (12 - shift)) & 0x3FFu;
  int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
  while (callsign_hashtable[idx].callsign[0] != '\0')
  {
    if (((callsign_hashtable[idx].hash & 0x3FFFFFu) >> shift) == hash)
    {
      strcpy(callsign, callsign_hashtable[idx].callsign);
      return true;
    }
    idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
  }
  callsign[0] = '\0';
  return false;
}

//------------------------------------------------------------------------------
// CONSTANT TABLES
//------------------------------------------------------------------------------
static const uint8_t kFT8_Costas_pattern[7] = { 3, 1, 4, 0, 6, 5, 2 };
static const uint8_t kFT8_Gray_map[8] = { 0, 1, 3, 2, 5, 6, 4, 7 };

static const uint8_t kFTX_LDPC_Nm[FTX_LDPC_M][7] =
{
  { 4, 31, 59, 91, 92, 96, 153 }, { 5, 32, 60, 93, 115, 146, 0 },
  { 6, 24, 61, 94, 122, 151, 0 }, { 7, 33, 62, 95, 96, 143, 0 },
  { 8, 25, 63, 83, 93, 96, 148 }, { 6, 32, 64, 97, 126, 138, 0 },
  { 5, 34, 65, 78, 98, 107, 154 }, { 9, 35, 66, 99, 139, 146, 0 },
  { 10, 36, 67, 100, 107, 126, 0 }, { 11, 37, 67, 87, 101, 139, 158 },
  { 12, 38, 68, 102, 105, 155, 0 }, { 13, 39, 69, 103, 149, 162, 0 },
  { 8, 40, 70, 82, 104, 114, 145 }, { 14, 41, 71, 88, 102, 123, 156 },
  { 15, 42, 59, 106, 123, 159, 0 }, { 1, 33, 72, 106, 107, 157, 0 },
  { 16, 43, 73, 108, 141, 160, 0 }, { 17, 37, 74, 81, 109, 131, 154 },
  { 11, 44, 75, 110, 121, 166, 0 }, { 45, 55, 64, 111, 130, 161, 173 },
  { 8, 46, 71, 112, 119, 166, 0 }, { 18, 36, 76, 89, 113, 114, 143 },
  { 19, 38, 77, 104, 116, 163, 0 }, { 20, 47, 70, 92, 138, 165, 0 },
  { 2, 48, 74, 113, 128, 160, 0 }, { 21, 45, 78, 83, 117, 121, 151 },
  { 22, 47, 58, 118, 127, 164, 0 }, { 16, 39, 62, 112, 134, 158, 0 },
  { 23, 43, 79, 120, 131, 145, 0 }, { 19, 35, 59, 73, 110, 125, 161 },
  { 20, 36, 63, 94, 136, 161, 0 }, { 14, 31, 79, 98, 132, 164, 0 },
  { 3, 44, 80, 124, 127, 169, 0 }, { 19, 46, 81, 117, 135, 167, 0 },
  { 7, 49, 58, 90, 100, 105, 168 }, { 12, 50, 61, 118, 119, 144, 0 },
  { 13, 51, 64, 114, 118, 157, 0 }, { 24, 52, 76, 129, 148, 149, 0 },
  { 25, 53, 69, 90, 101, 130, 156 }, { 20, 46, 65, 80, 120, 140, 170 },
  { 21, 54, 77, 100, 140, 171, 0 }, { 35, 82, 133, 142, 171, 174, 0 },
  { 14, 30, 83, 113, 125, 170, 0 }, { 4, 29, 68, 120, 134, 173, 0 },
  { 1, 4, 52, 57, 86, 136, 152 }, { 26, 51, 56, 91, 122, 137, 168 },
  { 52, 84, 110, 115, 145, 168, 0 }, { 7, 50, 81, 99, 132, 173, 0 },
  { 23, 55, 67, 95, 172, 174, 0 }, { 26, 41, 77, 109, 141, 148, 0 },
  { 2, 27, 41, 61, 62, 115, 133 }, { 27, 40, 56, 124, 125, 126, 0 },
  { 18, 49, 55, 124, 141, 167, 0 }, { 6, 33, 85, 108, 116, 156, 0 },
  { 28, 48, 70, 85, 105, 129, 158 }, { 9, 54, 63, 131, 147, 155, 0 },
  { 22, 53, 68, 109, 121, 174, 0 }, { 3, 13, 48, 78, 95, 123, 0 },
  { 31, 69, 133, 150, 155, 169, 0 }, { 12, 43, 66, 89, 97, 135, 159 },
  { 5, 39, 75, 102, 136, 167, 0 }, { 2, 54, 86, 101, 135, 164, 0 },
  { 15, 56, 87, 108, 119, 171, 0 }, { 10, 44, 82, 91, 111, 144, 149 },
  { 23, 34, 71, 94, 127, 153, 0 }, { 11, 49, 88, 92, 142, 157, 0 },
  { 29, 34, 87, 97, 147, 162, 0 }, { 30, 50, 60, 86, 137, 142, 162 },
  { 10, 53, 66, 84, 112, 128, 165 }, { 22, 57, 85, 93, 140, 159, 0 },
  { 28, 32, 72, 103, 132, 166, 0 }, { 28, 29, 84, 88, 117, 143, 150 },
  { 1, 26, 45, 80, 128, 147, 0 }, { 17, 27, 89, 103, 116, 153, 0 },
  { 51, 57, 98, 163, 165, 172, 0 }, { 21, 37, 73, 138, 152, 169, 0 },
  { 16, 47, 76, 130, 137, 154, 0 }, { 3, 24, 30, 72, 104, 139, 0 },
  { 9, 40, 90, 106, 134, 151, 0 }, { 15, 58, 60, 74, 111, 150, 163 },
  { 18, 42, 79, 144, 146, 152, 0 }, { 25, 38, 65, 99, 122, 160, 0 },
  { 17, 42, 75, 129, 170, 172, 0 }
};

static const uint8_t kFTX_LDPC_Mn[FTX_LDPC_N][3] =
{
  { 16, 45, 73 }, { 25, 51, 62 }, { 33, 58, 78 }, { 1, 44, 45 },
  { 2, 7, 61 }, { 3, 6, 54 }, { 4, 35, 48 }, { 5, 13, 21 },
  { 8, 56, 79 }, { 9, 64, 69 }, { 10, 19, 66 }, { 11, 36, 60 },
  { 12, 37, 58 }, { 14, 32, 43 }, { 15, 63, 80 }, { 17, 28, 77 },
  { 18, 74, 83 }, { 22, 53, 81 }, { 23, 30, 34 }, { 24, 31, 40 },
  { 26, 41, 76 }, { 27, 57, 70 }, { 29, 49, 65 }, { 3, 38, 78 },
  { 5, 39, 82 }, { 46, 50, 73 }, { 51, 52, 74 }, { 55, 71, 72 },
  { 44, 67, 72 }, { 43, 68, 78 }, { 1, 32, 59 }, { 2, 6, 71 },
  { 4, 16, 54 }, { 7, 65, 67 }, { 8, 30, 42 }, { 9, 22, 31 },
  { 10, 18, 76 }, { 11, 23, 82 }, { 12, 28, 61 }, { 13, 52, 79 },
  { 14, 50, 51 }, { 15, 81, 83 }, { 17, 29, 60 }, { 19, 33, 64 },
  { 20, 26, 73 }, { 21, 34, 40 }, { 24, 27, 77 }, { 25, 55, 58 },
  { 35, 53, 66 }, { 36, 48, 68 }, { 37, 46, 75 }, { 38, 45, 47 },
  { 39, 57, 69 }, { 41, 56, 62 }, { 20, 49, 53 }, { 46, 52, 63 },
  { 45, 70, 75 }, { 27, 35, 80 }, { 1, 15, 30 }, { 2, 68, 80 },
  { 3, 36, 51 }, { 4, 28, 51 }, { 5, 31, 56 }, { 6, 20, 37 },
  { 7, 40, 82 }, { 8, 60, 69 }, { 9, 10, 49 }, { 11, 44, 57 },
  { 12, 39, 59 }, { 13, 24, 55 }, { 14, 21, 65 }, { 16, 71, 78 },
  { 17, 30, 76 }, { 18, 25, 80 }, { 19, 61, 83 }, { 22, 38, 77 },
  { 23, 41, 50 }, { 7, 26, 58 }, { 29, 32, 81 }, { 33, 40, 73 },
  { 18, 34, 48 }, { 13, 42, 64 }, { 5, 26, 43 }, { 47, 69, 72 },
  { 54, 55, 70 }, { 45, 62, 68 }, { 10, 63, 67 }, { 14, 66, 72 },
  { 22, 60, 74 }, { 35, 39, 79 }, { 1, 46, 64 }, { 1, 24, 66 },
  { 2, 5, 70 }, { 3, 31, 65 }, { 4, 49, 58 }, { 1, 4, 5 },
  { 6, 60, 67 }, { 7, 32, 75 }, { 8, 48, 82 }, { 9, 35, 41 },
  { 10, 39, 62 }, { 11, 14, 61 }, { 12, 71, 74 }, { 13, 23, 78 },
  { 11, 35, 55 }, { 15, 16, 79 }, { 7, 9, 16 }, { 17, 54, 63 },
  { 18, 50, 57 }, { 19, 30, 47 }, { 20, 64, 80 }, { 21, 28, 69 },
  { 22, 25, 43 }, { 13, 22, 37 }, { 2, 47, 51 }, { 23, 54, 74 },
  { 26, 34, 72 }, { 27, 36, 37 }, { 21, 36, 63 }, { 29, 40, 44 },
  { 19, 26, 57 }, { 3, 46, 82 }, { 14, 15, 58 }, { 33, 52, 53 },
  { 30, 43, 52 }, { 6, 9, 52 }, { 27, 33, 65 }, { 25, 69, 73 },
  { 38, 55, 83 }, { 20, 39, 77 }, { 18, 29, 56 }, { 32, 48, 71 },
  { 42, 51, 59 }, { 28, 44, 79 }, { 34, 60, 62 }, { 31, 45, 61 },
  { 46, 68, 77 }, { 6, 24, 76 }, { 8, 10, 78 }, { 40, 41, 70 },
  { 17, 50, 53 }, { 42, 66, 68 }, { 4, 22, 72 }, { 36, 64, 81 },
  { 13, 29, 47 }, { 2, 8, 81 }, { 56, 67, 73 }, { 5, 38, 50 },
  { 12, 38, 64 }, { 59, 72, 80 }, { 3, 26, 79 }, { 45, 76, 81 },
  { 1, 65, 74 }, { 7, 18, 77 }, { 11, 56, 59 }, { 14, 39, 54 },
  { 16, 37, 66 }, { 10, 28, 55 }, { 15, 60, 70 }, { 17, 25, 82 },
  { 20, 30, 31 }, { 12, 67, 68 }, { 23, 75, 80 }, { 27, 32, 62 },
  { 24, 69, 75 }, { 19, 21, 71 }, { 34, 53, 61 }, { 35, 46, 47 },
  { 33, 59, 76 }, { 40, 43, 83 }, { 41, 42, 63 }, { 49, 75, 83 },
  { 20, 44, 48 }, { 42, 49, 57 }
};

static const uint8_t kFTX_LDPC_Num_rows[FTX_LDPC_M] =
{
  7, 6, 6, 6, 7, 6, 7, 6, 6, 7, 6, 6, 7, 7, 6, 6,
  6, 7, 6, 7, 6, 7, 6, 6, 6, 7, 6, 6, 6, 7, 6, 6,
  6, 6, 7, 6, 6, 6, 7, 7, 6, 6, 6, 6, 7, 7, 6, 6,
  6, 6, 7, 6, 6, 6, 7, 6, 6, 6, 6, 7, 6, 6, 6, 7,
  6, 6, 6, 7, 7, 6, 6, 7, 6, 6, 6, 6, 6, 6, 6, 7,
  6, 6, 6
};

//------------------------------------------------------------------------------
// CRC
//------------------------------------------------------------------------------
#define FT8_TOPBIT (1u << (FT8_CRC_WIDTH - 1))

static const uint16_t ftx_compute_crc(const uint8_t message[], int num_bits)
{
  uint16_t remainder = 0;
  int idx_byte = 0;
  for (int idx_bit = 0; idx_bit < num_bits; ++idx_bit)
  {
    if (idx_bit % 8 == 0)
    {
      remainder ^= (message[idx_byte] << (FT8_CRC_WIDTH - 8));
      ++idx_byte;
    }
    if (remainder & FT8_TOPBIT)
      remainder = (remainder << 1) ^ FT8_CRC_POLYNOMIAL;
    else
      remainder = (remainder << 1);
  }
  return remainder & ((FT8_TOPBIT << 1) - 1u);
}

static const uint16_t ftx_extract_crc(const uint8_t a91[])
{
  return ((a91[9] & 0x07) << 11) | (a91[10] << 3) | (a91[11] >> 5);
}

static void ftx_add_crc(const uint8_t payload[], uint8_t a91[])
{
  for (int i = 0; i < 10; i++) a91[i] = payload[i];
  a91[9] &= 0xF8u; a91[10] = 0;
  const uint16_t checksum = ftx_compute_crc(a91, 96 - 14);
  a91[9] |= (uint8_t)(checksum >> 11);
  a91[10] = (uint8_t)(checksum >> 3);
  a91[11] = (uint8_t)(checksum << 5);
}

//------------------------------------------------------------------------------
// LDPC  (static arrays avoid ~115 KB stack)
//------------------------------------------------------------------------------
static const float fast_tanh(float x)
{
  if (x < -4.97f) return -1.0f;
  if (x > 4.97f) return  1.0f;
  const float x2 = x * x;
  return x * (945.0f + x2 * (105.0f + x2)) / (945.0f + x2 * (420.0f + x2 * 15.0f));
}

static const float fast_atanh(float x)
{
  const float x2 = x * x;
  return x * (945.0f + x2 * (-735.0f + x2 * 64.0f)) / (945.0f + x2 * (-1050.0f + x2 * 225.0f));
}

static const int ldpc_check(uint8_t codeword[])
{
  int errors = 0;
  for (int m = 0; m < FTX_LDPC_M; ++m)
  {
    uint8_t x = 0;
    for (int i = 0; i < kFTX_LDPC_Num_rows[m]; ++i) x ^= codeword[kFTX_LDPC_Nm[m][i] - 1];
    if (x != 0) ++errors;
  }
  return errors;
}

static void bp_decode(float codeword[], int max_iters, uint8_t plain[], int* ok)
{
  static float tov[FTX_LDPC_N][3];
  static float toc[FTX_LDPC_M][7];
  int min_errors = FTX_LDPC_M;

  for (int n = 0; n < FTX_LDPC_N; ++n) tov[n][0] = tov[n][1] = tov[n][2] = 0;

  for (int iter = 0; iter < max_iters; ++iter)
  {
    int plain_sum = 0;
    for (int n = 0; n < FTX_LDPC_N; ++n)
    {
      plain[n] = ((codeword[n] + tov[n][0] + tov[n][1] + tov[n][2]) > 0) ? 1 : 0; plain_sum += plain[n];
    }
    if (plain_sum == 0) break;

    int errors = ldpc_check(plain);
    if (errors < min_errors) { min_errors = errors; if (errors == 0) break; }

    for (int m = 0; m < FTX_LDPC_M; ++m)
      for (int n_idx = 0; n_idx < kFTX_LDPC_Num_rows[m]; ++n_idx)
      {
        int n = kFTX_LDPC_Nm[m][n_idx] - 1;
        float Tnm = codeword[n];
        for (int m_idx = 0; m_idx < 3; ++m_idx)
          if ((kFTX_LDPC_Mn[n][m_idx] - 1) != m) Tnm += tov[n][m_idx];
        toc[m][n_idx] = fast_tanh(-Tnm / 2);
      }

    for (int n = 0; n < FTX_LDPC_N; ++n)
      for (int m_idx = 0; m_idx < 3; ++m_idx)
      {
        int m = kFTX_LDPC_Mn[n][m_idx] - 1;
        float Tmn = 1.0f;
        for (int n_idx = 0; n_idx < kFTX_LDPC_Num_rows[m]; ++n_idx)
          if ((kFTX_LDPC_Nm[m][n_idx] - 1) != n) Tmn *= toc[m][n_idx];
        tov[n][m_idx] = -2 * fast_atanh(Tmn);
      }
  }
  *ok = min_errors;
}

//------------------------------------------------------------------------------
// TEXT UTILITIES
//------------------------------------------------------------------------------
static const char* trim_front(const char* str, char to_trim) { while (*str == to_trim) str++; return str; }
static void trim_back(char* str, char to_trim) { int i = (int)strlen(str) - 1; while (i >= 0 && str[i] == to_trim) str[i--] = '\0'; }
static char* trim(char* str) { str = (char*)trim_front(str, ' '); trim_back(str, ' '); return str; }
static char* trim_brackets(char* str) { str = (char*)trim_front(str, '<'); trim_back(str, '>'); return str; }
static void trim_copy(char* t, const char* str) { str = (char*)trim_front(str, ' '); int n = (int)strlen(str) - 1; while (n >= 0 && str[n] == ' ')n--; strncpy(t, str, n + 1); t[n + 1] = '\0'; }
static char to_upper(char c) { return (c >= 'a' && c <= 'z') ? (c - 'a' + 'A') : c; }
static bool is_digit(char c) { return (c >= '0') && (c <= '9'); }
static bool is_letter(char c) { return ((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z')); }
static bool is_space(char c) { return (c == ' '); }
static bool in_range(char c, char mn, char mx) { return (c >= mn) && (c <= mx); }
static bool starts_with(const char* s, const char* p) { return 0 == memcmp(s, p, strlen(p)); }
static bool ends_with(const char* s, const char* sfx) { int p = (int)(strlen(s) - strlen(sfx)); return (p >= 0) && (0 == memcmp(s + p, sfx, strlen(sfx))); }
static bool equals(const char* s1, const char* s2) { return 0 == strcmp(s1, s2); }
static void fmtmsg(char* o, const char* i) { char c, l = 0; while ((c = *i)) { if (c != ' ' || l != ' ') { l = to_upper(c); *o = l; ++o; }++i; } *o = 0; }
static char* append_string(char* s, const char* t) { while (*t != '\0') { *s = *t; s++; t++; } *s = '\0'; return s; }
static const char* copy_token(char* t, int n, const char* s) { while (*s != ' ' && *s != '\0') { if (n > 1) { *t = *s; t++; n--; }s++; } while (n > 0) { *t = '\0'; t++; n--; } while (*s == ' ')s++; return s; }

static const int dd_to_int(const char* str, int length)
{
  int result = 0; bool neg; int i;
  if (str[0] == '-') { neg = true; i = 1; }
  else { neg = false; i = (str[0] == '+') ? 1 : 0; }
  while (i < length) { if (!str[i] || !is_digit(str[i]))break; result = result * 10 + (str[i] - '0'); ++i; }
  return neg ? -result : result;
}
static void int_to_dd(char* str, int value, int width, bool full_sign)
{
  if (value < 0) { *str = '-'; ++str; value = -value; }
  else if (full_sign) { *str = '+'; ++str; }
  int d = 1; for (int i = 0; i < width - 1; ++i)d *= 10;
  while (d >= 1) { int dg = value / d; *str = '0' + dg; ++str; value -= dg * d; d /= 10; } *str = 0;
}
static char charn(int c, ft8_char_table_e t)
{
  if (t != FT8_CHAR_TABLE_ALPHANUM && t != FT8_CHAR_TABLE_NUMERIC) { if (c == 0)return ' '; c -= 1; }
  if (t != FT8_CHAR_TABLE_LETTERS_SPACE) { if (c < 10)return '0' + c; c -= 10; }
  if (t != FT8_CHAR_TABLE_NUMERIC) { if (c < 26)return 'A' + c; c -= 26; }
  if (t == FT8_CHAR_TABLE_FULL && c < 5)return "+-./??"[c];
  if (t == FT8_CHAR_TABLE_ALPHANUM_SPACE_SLASH && c == 0)return '/';
  return '_';
}
static const int nchar(char c, ft8_char_table_e t)
{
  int n = 0;
  if (t != FT8_CHAR_TABLE_ALPHANUM && t != FT8_CHAR_TABLE_NUMERIC) { if (c == ' ')return n; n += 1; }
  if (t != FT8_CHAR_TABLE_LETTERS_SPACE) { if (c >= '0' && c <= '9')return n + (c - '0'); n += 10; }
  if (t != FT8_CHAR_TABLE_NUMERIC) { if (c >= 'A' && c <= 'Z')return n + (c - 'A'); n += 26; }
  if (t == FT8_CHAR_TABLE_FULL) { if (c == '+')return n; if (c == '-')return n + 1; if (c == '.')return n + 2; if (c == '/')return n + 3; if (c == '?')return n + 4; }
  if (t == FT8_CHAR_TABLE_ALPHANUM_SPACE_SLASH && c == '/')return n;
  return -1;
}

//------------------------------------------------------------------------------
// MESSAGE PACK / UNPACK
//------------------------------------------------------------------------------
#define MAX22    ((uint32_t)4194304ul)
#define NTOKENS  ((uint32_t)2063592ul)
#define MAXGRID4 ((uint16_t)32400ul)

static void add_brackets(char* r, const char* o, int n) { r[0] = '<'; memcpy(r + 1, o, n); r[n + 1] = '>'; r[n + 2] = '\0'; }

static const bool save_callsign(const char* cs, uint32_t* n22o, uint16_t* n12o, uint16_t* n10o)
{
  uint64_t n58 = 0; int i = 0;
  while (cs[i] != '\0' && i < 11) { int j = nchar(cs[i], FT8_CHAR_TABLE_ALPHANUM_SPACE_SLASH); if (j < 0)return false; n58 = (38 * n58) + j; i++; }
  while (i < 11) { n58 = (38 * n58); i++; }
  uint32_t n22 = ((47055833459ull * n58) >> (64 - 22)) & (0x3FFFFFul);
  if (n22o) *n22o = n22;
  if (n12o) *n12o = (uint16_t)(n22 >> 10);
  if (n10o) *n10o = (uint16_t)(n22 >> 12);
  ft8_hashtable_add(cs, n22);
  return true;
}

static const bool lookup_callsign(ftx_callsign_hash_type_t ht, uint32_t hash, char* cs)
{
  char c11[12];
  const bool found = ft8_hashtable_lookup(ht, hash, c11);
  if (!found) strcpy(cs, "<...>"); else add_brackets(cs, c11, (int)strlen(c11)); return found;
}

static const int parse_cq_modifier(const char* s)
{
  int nnum = 0, nlet = 0, m = 0;
  for (int i = 3; i < 8; ++i) { if (!s[i] || is_space(s[i]))break; else if (is_digit(s[i]))++nnum; else if (is_letter(s[i])) { ++nlet; m = 27 * m + (s[i] - 'A' + 1); } else return -1; }
  if (nnum == 3 && nlet == 0)return atoi(s + 3); if (nnum == 0 && nlet <= 4)return 1000 + m; return -1;
}

static const int32_t pack_basecall(const char* cs, int len)
{
  if (len > 2)
  {
    char c6[6] = { ' ',' ',' ',' ',' ',' ' };
    if (starts_with(cs, "3DA0") && len > 4 && len <= 7) { memcpy(c6, "3D0", 3); memcpy(c6 + 3, cs + 4, len - 4); }
    else if (starts_with(cs, "3X") && is_letter(cs[2]) && len <= 7) { memcpy(c6, "Q", 1); memcpy(c6 + 1, cs + 2, len - 2); }
    else { if (is_digit(cs[2]) && len <= 6)memcpy(c6, cs, len); else if (is_digit(cs[1]) && len <= 5)memcpy(c6 + 1, cs, len); }
    int i0 = nchar(c6[0], FT8_CHAR_TABLE_ALPHANUM_SPACE), i1 = nchar(c6[1], FT8_CHAR_TABLE_ALPHANUM), i2 = nchar(c6[2], FT8_CHAR_TABLE_NUMERIC);
    int i3 = nchar(c6[3], FT8_CHAR_TABLE_LETTERS_SPACE), i4 = nchar(c6[4], FT8_CHAR_TABLE_LETTERS_SPACE), i5 = nchar(c6[5], FT8_CHAR_TABLE_LETTERS_SPACE);
    if (i0 >= 0 && i1 >= 0 && i2 >= 0 && i3 >= 0 && i4 >= 0 && i5 >= 0) { int32_t n = i0; n = n * 36 + i1; n = n * 10 + i2; n = n * 27 + i3; n = n * 27 + i4; n = n * 27 + i5; return n; }
  }
  return -1;
}

static const int32_t pack28(const char* cs, uint8_t* ip)
{
  *ip = 0;
  if (equals(cs, "DE"))return 0; if (equals(cs, "QRZ"))return 1; if (equals(cs, "CQ"))return 2;
  int len = (int)strlen(cs);
  if (starts_with(cs, "CQ ") && len < 8) { int v = parse_cq_modifier(cs); if (v < 0)return -1; return 3 + v; }
  int lb = len;
  if (ends_with(cs, "/P") || ends_with(cs, "/R")) { *ip = 1; lb = len - 2; }
  int32_t n28 = pack_basecall(cs, lb);
  if (n28 >= 0) { if (!save_callsign(cs, NULL, NULL, NULL))return -1; return NTOKENS + MAX22 + (uint32_t)n28; }
  if (len >= 3 && len <= 11) { uint32_t n22; if (!save_callsign(cs, &n22, NULL, NULL))return -1; *ip = 0; return NTOKENS + n22; }
  return -1;
}

static const int unpack28(uint32_t n28, uint8_t ip, uint8_t i3, char* r)
{
  if (n28 < NTOKENS)
  {
    if (n28 <= 2u)
    {
      if (n28 == 0)
      {
        strcpy(r, "DE");
      }
      else if (n28 == 1)
      {
        strcpy(r, "QRZ");
      }
      else
      {
        strcpy(r, "CQ");
      } return 0;
    }
    if (n28 <= 1002u)
    {
      strcpy(r, "CQ ");
      int_to_dd(r + 3, n28 - 3, 3, false);
      return 0;
    }
    if (n28 <= 532443ul)
    {
      uint32_t n = n28 - 1003u;
      char a[5];
      a[4] = '\0';
      for (int i = 3;; --i)
      {
        a[i] = charn(n % 27u, FT8_CHAR_TABLE_LETTERS_SPACE);
        if (!i) break;
        n /= 27u;
      }
      strcpy(r, "CQ ");
      strcat(r, trim_front(a, ' '));
      return 0;
    }
    return -1;
  }
  n28 -= NTOKENS;
  if (n28 < MAX22)
  {
    lookup_callsign(FTX_CALLSIGN_HASH_22_BITS, n28, r);
    return 0;
  }
  uint32_t n = n28 - MAX22;
  char cs[7];
  cs[6] = '\0';
  cs[5] = charn(n % 27, FT8_CHAR_TABLE_LETTERS_SPACE); n /= 27; cs[4] = charn(n % 27, FT8_CHAR_TABLE_LETTERS_SPACE); n /= 27;
  cs[3] = charn(n % 27, FT8_CHAR_TABLE_LETTERS_SPACE); n /= 27; cs[2] = charn(n % 10, FT8_CHAR_TABLE_NUMERIC); n /= 10;
  cs[1] = charn(n % 36, FT8_CHAR_TABLE_ALPHANUM); n /= 36; cs[0] = charn(n % 37, FT8_CHAR_TABLE_ALPHANUM_SPACE);
  if (starts_with(cs, "3D0") && !is_space(cs[3])) { memcpy(r, "3DA0", 4); trim_copy(r + 4, cs + 3); }
  else if (cs[0] == 'Q' && is_letter(cs[1])) { memcpy(r, "3X", 2); trim_copy(r + 2, cs + 1); }
  else trim_copy(r, cs);
  if ((int)strlen(r) < 3) return -1;
  if (ip != 0) { if (i3 == 1) strcat(r, "/R"); else if (i3 == 2) strcat(r, "/P"); else return -2; }
  save_callsign(r, NULL, NULL, NULL);
  return 0;
}

static bool pack58(const char* cs, uint64_t* n58)
{
  const char* s = cs; if (*s == '<')s++; int len = 0; uint64_t res = 0; char c11[12];
  while (*s != '\0' && *s != '<' && len < 11) { c11[len] = *s; int j = nchar(*s, FT8_CHAR_TABLE_ALPHANUM_SPACE_SLASH); if (j < 0)return false; res = res * 38 + j; s++; len++; }
  c11[len] = '\0'; if (!save_callsign(c11, NULL, NULL, NULL))return false; *n58 = res; return true;
}

static bool unpack58(uint64_t n58, char* cs)
{
  char c11[12];
  c11[11] = '\0';
  uint64_t nb = n58;
  for (int i = 10;; --i) { c11[i] = charn(nb % 38, FT8_CHAR_TABLE_ALPHANUM_SPACE_SLASH); if (!i)break; nb /= 38; }
  trim_copy(cs, c11); return (strlen(cs) >= 3) ? save_callsign(cs, NULL, NULL, NULL) : false;
}

static uint16_t packgrid(const char* g)
{
  if (!g || !g[0]) return MAXGRID4 + 1;
  if (equals(g, "RRR")) return MAXGRID4 + 2; if (equals(g, "RR73"))return MAXGRID4 + 3; if (equals(g, "73"))return MAXGRID4 + 4;
  if (in_range(g[0], 'A', 'R') && in_range(g[1], 'A', 'R') && is_digit(g[2]) && is_digit(g[3]))
  {
    uint16_t ig = (g[0] - 'A'); ig = ig * 18 + (g[1] - 'A'); ig = ig * 10 + (g[2] - '0'); ig = ig * 10 + (g[3] - '0'); return ig;
  }
  if (g[0] == 'R') { int d = dd_to_int(g + 1, 3); return (MAXGRID4 + 35 + d) | 0x8000; }
  else { int d = dd_to_int(g, 3); return (MAXGRID4 + 35 + d); }
}

static int unpackgrid(uint16_t ig, uint8_t ir, char* ex)
{
  char* d = ex;
  if (ig <= MAXGRID4)
  {
    if (ir > 0) d = stpcpy(d, "R ");
    uint16_t n = ig;
    d[4] = '\0';
    d[3] = '0' + (n % 10);
    n /= 10;
    d[2] = '0' + (n % 10);
    n /= 10;
    d[1] = 'A' + (n % 18);
    n /= 18;
    d[0] = 'A' + (n % 18);
  }
  else
  {
    int irpt = ig - MAXGRID4;
    switch (irpt)
    {
    case 1: d[0] = '\0'; break;
    case 2: strcpy(d, "RRR"); break;
    case 3: strcpy(d, "RR73"); break;
    case 4: strcpy(d, "73"); break;
    default:
      if (ir > 0) *d++ = 'R';
      int_to_dd(d, irpt - 35, 2, true);
      break;
    }
  }
  return 0;
}

void ftx_message_init(ftx_message_t* m) { memset(m, 0, sizeof(ftx_message_t)); }
uint8_t ftx_message_get_i3(const ftx_message_t* m) { return (m->payload[9] >> 3) & 0x07u; }
uint8_t ftx_message_get_n3(const ftx_message_t* m) { return ((m->payload[8] << 2) & 0x04u) | ((m->payload[9] >> 6) & 0x03u); }

static ftx_message_type_t ftx_message_get_type(const ftx_message_t* m)
{
  uint8_t i3 = (m->payload[9] >> 3) & 0x07u;
  switch (i3)
  {
  case 0:
  {
    uint8_t n3 = ((m->payload[8] << 2) & 0x04u) | ((m->payload[9] >> 6) & 0x03u);
    switch (n3)
    {
    case 0: return FTX_MESSAGE_TYPE_FREE_TEXT;
    case 1: return FTX_MESSAGE_TYPE_DXPEDITION;
    case 2: return FTX_MESSAGE_TYPE_EU_VHF;
    case 3: return FTX_MESSAGE_TYPE_ARRL_FD;
    case 4: return FTX_MESSAGE_TYPE_ARRL_FD;
    case 5: return FTX_MESSAGE_TYPE_TELEMETRY;
    default: return FTX_MESSAGE_TYPE_UNKNOWN;
    }
  }
  case 1: return FTX_MESSAGE_TYPE_STANDARD;
  case 2: return FTX_MESSAGE_TYPE_STANDARD;
  case 3: return FTX_MESSAGE_TYPE_ARRL_RTTY;
  case 4: return FTX_MESSAGE_TYPE_NONSTD_CALL;
  case 5: return FTX_MESSAGE_TYPE_WWROF;
  default: return FTX_MESSAGE_TYPE_UNKNOWN;
  }
}

static ftx_message_rc_t ftx_message_encode_std(ftx_message_t* msg, const char* ct, const char* cd, const char* ex)
{
  uint8_t ipa, ipb;
  int32_t n28a = pack28(ct, &ipa);
  int32_t n28b = pack28(cd, &ipb);
  if (n28a < 0) return FTX_MESSAGE_RC_ERROR_CALLSIGN1;
  if (n28b < 0) return FTX_MESSAGE_RC_ERROR_CALLSIGN2;
  uint8_t i3 = 1;
  if (ends_with(ct, "/P") || ends_with(cd, "/P"))
  {
    i3 = 2;
    if (ends_with(ct, "/R") || ends_with(cd, "/R")) return FTX_MESSAGE_RC_ERROR_SUFFIX;
  }
  const char* sd = strchr(cd, '/');
  uint8_t icq = (uint8_t)(equals(ct, "CQ") || starts_with(ct, "CQ "));
  if (sd && (sd - cd >= 2) && icq && !equals(sd, "/P") && !equals(sd, "/R")) return FTX_MESSAGE_RC_ERROR_CALLSIGN2;
  uint16_t ig = packgrid(ex);
  uint32_t n29a = ((uint32_t)n28a << 1) | ipa; uint32_t n29b = ((uint32_t)n28b << 1) | ipb;
  if (ends_with(ct, "/R"))n29a |= 1; else if (ends_with(ct, "/P")) { n29a |= 1; i3 = 2; }
  msg->payload[0] = (uint8_t)(n29a >> 21); msg->payload[1] = (uint8_t)(n29a >> 13); msg->payload[2] = (uint8_t)(n29a >> 5);
  msg->payload[3] = (uint8_t)(n29a << 3) | (uint8_t)(n29b >> 26); msg->payload[4] = (uint8_t)(n29b >> 18);
  msg->payload[5] = (uint8_t)(n29b >> 10); msg->payload[6] = (uint8_t)(n29b >> 2);
  msg->payload[7] = (uint8_t)(n29b << 6) | (uint8_t)(ig >> 10); msg->payload[8] = (uint8_t)(ig >> 2);
  msg->payload[9] = (uint8_t)(ig << 6) | (uint8_t)(i3 << 3);
  return FTX_MESSAGE_RC_OK;
}

static ftx_message_rc_t ftx_message_encode_nonstd(ftx_message_t* msg, const char* ct, const char* cd, const char* ex)
{
  uint8_t i3 = 4, icq = (uint8_t)(equals(ct, "CQ") || starts_with(ct, "CQ "));
  const int ld = (int)strlen(cd);
  if (!icq && (int)strlen(ct) < 3) return FTX_MESSAGE_RC_ERROR_CALLSIGN1;
  if (ld < 3) return FTX_MESSAGE_RC_ERROR_CALLSIGN2;
  uint8_t iflip; uint16_t n12; uint64_t n58; uint8_t nrpt; const char* c58;
  if (!icq) { iflip = (cd[0] == '<' && cd[ld - 1] == '>') ? 1 : 0; const char* c12 = iflip ? cd : ct; c58 = iflip ? ct : cd; if (!save_callsign(c12, NULL, &n12, NULL))return FTX_MESSAGE_RC_ERROR_CALLSIGN1; }
  else { iflip = 0; n12 = 0; c58 = cd; }
  if (!pack58(c58, &n58)) return FTX_MESSAGE_RC_ERROR_CALLSIGN2;
  if (icq) nrpt = 0; else if (equals(ex, "RRR"))nrpt = 1; else if (equals(ex, "RR73")) nrpt = 2; else if (equals(ex, "73"))nrpt = 3; else nrpt = 0;
  msg->payload[0] = (uint8_t)(n12 >> 4); msg->payload[1] = (uint8_t)(n12 << 4) | (uint8_t)(n58 >> 54);
  msg->payload[2] = (uint8_t)(n58 >> 46); msg->payload[3] = (uint8_t)(n58 >> 38); msg->payload[4] = (uint8_t)(n58 >> 30);
  msg->payload[5] = (uint8_t)(n58 >> 22); msg->payload[6] = (uint8_t)(n58 >> 14); msg->payload[7] = (uint8_t)(n58 >> 6);
  msg->payload[8] = (uint8_t)(n58 << 2) | (uint8_t)(iflip << 1) | (uint8_t)(nrpt >> 1);
  msg->payload[9] = (uint8_t)(nrpt << 7) | (uint8_t)(icq << 6) | (uint8_t)(i3 << 3);
  return FTX_MESSAGE_RC_OK;
}

static ftx_message_rc_t ftx_message_encode_telemetry(ftx_message_t* msg, const uint8_t* t)
{
  uint8_t c = 0; for (int i = 8; i >= 0; --i) { msg->payload[i] = (t[i] << 1) | (c >> 7); c = t[i] & 0x80; } return FTX_MESSAGE_RC_OK;
}

static ftx_message_rc_t ftx_message_encode_free(ftx_message_t* msg, const char* text)
{
  if ((int)strlen(text) > 13) return FTX_MESSAGE_RC_ERROR_TYPE;
  uint8_t b[9]; memset(b, 0, 9); int sl = (int)strlen(text);
  for (int i = 0; i < 13; i++) { char c = (i < sl) ? text[i] : ' '; int8_t cid = nchar(c, FT8_CHAR_TABLE_FULL); if (cid < 0)return FTX_MESSAGE_RC_ERROR_TYPE; uint16_t r = cid; for (int j = 8; j >= 0; j--) { r += b[j] * 42; b[j] = r & 0xff; r >>= 8; } }
  ftx_message_rc_t ret = ftx_message_encode_telemetry(msg, b); msg->payload[9] = 0; return ret;
}

static ftx_message_rc_t ftx_message_encode(ftx_message_t* msg, const char* text)
{
  char ct[12], cd[12], ex[20]; const char* p = text;
  const bool iscq = starts_with(text, "CQ ");
  if (iscq) { p += 3; memset(ct, 0, sizeof(ct)); copy_token(cd, 12, p); int v = parse_cq_modifier(text); if (v >= 0) { memcpy(ct, "CQ \0", 4); p = copy_token(ct + 3, sizeof(ct) - 3, p); } else memcpy(ct, "CQ\0", 3); }
  else p = copy_token(ct, sizeof(ct), p);
  p = copy_token(cd, sizeof(cd), p); p = copy_token(ex, sizeof(ex), p);
  if (ct[sizeof(ct) - 1] != '\0') return FTX_MESSAGE_RC_ERROR_CALLSIGN1;
  if (cd[sizeof(cd) - 1] != '\0') return FTX_MESSAGE_RC_ERROR_CALLSIGN2;
  if (ex[sizeof(ex) - 1] != '\0') return FTX_MESSAGE_RC_ERROR_GRID;
  ftx_message_rc_t rc;
  if (!p[0])
  {
    rc = ftx_message_encode_std(msg, ct, cd, ex);
    if (rc == FTX_MESSAGE_RC_OK) return rc;
    rc = ftx_message_encode_nonstd(msg, ct, cd, ex);
    if (rc == FTX_MESSAGE_RC_OK) return rc;
  }
  return ftx_message_encode_free(msg, text);
}

static ftx_message_rc_t ftx_message_decode_std(const ftx_message_t* msg, char* ct, char* cd, char* ex)
{
  uint32_t n29a = (msg->payload[0] << 21) | (msg->payload[1] << 13) | (msg->payload[2] << 5) | (msg->payload[3] >> 3);
  uint32_t n29b = ((msg->payload[3] & 0x07u) << 26) | (msg->payload[4] << 18) | (msg->payload[5] << 10) | (msg->payload[6] << 2) | (msg->payload[7] >> 6);
  uint8_t ir = ((msg->payload[7] & 0x20u) >> 5);
  uint16_t ig = ((msg->payload[7] & 0x1Fu) << 10) | (msg->payload[8] << 2) | (msg->payload[9] >> 6);
  uint8_t i3 = (msg->payload[9] >> 3) & 0x07u; ct[0] = cd[0] = ex[0] = '\0';
  if (unpack28(n29a >> 1, n29a & 1u, i3, ct) < 0) return FTX_MESSAGE_RC_ERROR_CALLSIGN1;
  if (unpack28(n29b >> 1, n29b & 1u, i3, cd) < 0) return FTX_MESSAGE_RC_ERROR_CALLSIGN2;
  if (unpackgrid(ig, ir, ex) < 0) return FTX_MESSAGE_RC_ERROR_GRID;
  return FTX_MESSAGE_RC_OK;
}

static ftx_message_rc_t ftx_message_decode_nonstd(const ftx_message_t* msg, char* ct, char* cd, char* ex)
{
  const uint16_t n12 = ((uint16_t)msg->payload[0] << 4) | (msg->payload[1] >> 4);
  const uint64_t n58 = ((uint64_t)(msg->payload[1] & 0x0Fu) << 54) | ((uint64_t)msg->payload[2] << 46) | ((uint64_t)msg->payload[3] << 38) | ((uint64_t)msg->payload[4] << 30) | ((uint64_t)msg->payload[5] << 22) | ((uint64_t)msg->payload[6] << 14) | ((uint64_t)msg->payload[7] << 6) | ((uint64_t)msg->payload[8] >> 2);
  const uint16_t iflip = (msg->payload[8] >> 1) & 0x01u, nrpt = ((msg->payload[8] & 0x01u) << 1) | (msg->payload[9] >> 7), icq = (msg->payload[9] >> 6) & 0x01u;
  char cdd[14];
  char c3[14];
  unpack58(n58, cdd);
  lookup_callsign(FTX_CALLSIGN_HASH_12_BITS, n12, c3);
  const char* c1 = iflip ? cdd : c3;
  const char* c2 = iflip ? c3 : cdd;
  if (!icq)
  {
    strcpy(ct, c1);
    if (nrpt == 1) { strcpy(ex, "RRR"); }
    else if (nrpt == 2) { strcpy(ex, "RR73"); }
    else if (nrpt == 3) { strcpy(ex, "73"); }
    else { ex[0] = '\0'; }
  }
  else
  {
    strcpy(ct, "CQ");
    ex[0] = '\0';
  }
  strcpy(cd, c2);
  return FTX_MESSAGE_RC_OK;
}

static void ftx_message_decode_telemetry(const ftx_message_t* msg, uint8_t* t)
{
  uint8_t c = 0;
  for (int i = 0; i < 9; ++i)
  {
    t[i] = (c << 7) | (msg->payload[i] >> 1);
    c = msg->payload[i] & 0x01u;
  }
}

static void ftx_message_decode_telemetry_hex(const ftx_message_t* msg, char* hex)
{
  uint8_t b[9];
  ftx_message_decode_telemetry(msg, b);
  for (int i = 0; i < 9; ++i)
  {
    const uint8_t n1 = b[i] >> 4;
    const uint8_t n2 = b[i] & 0x0Fu;
    hex[i * 2] = (n1 > 9) ? n1 - 10 + 'A' : n1 + '0';
    hex[i * 2 + 1] = (n2 > 9) ? n2 - 10 + 'A' : n2 + '0';
  }
  hex[18] = '\0';
}

static void ftx_message_decode_free(const ftx_message_t* msg, char* text)
{
  uint8_t b[9];
  ftx_message_decode_telemetry(msg, b);
  char c[14];
  c[13] = 0;
  for (int i = 12; i >= 0; --i)
  {
    uint16_t r = 0;
    for (int j = 0; j < 9; ++j)
    {
      r = (r << 8) | b[j];
      b[j] = r / 42;
      r %= 42;
    }
    c[i] = charn(r, FT8_CHAR_TABLE_FULL);
  }
  strcpy(text, trim(c));
}

static ftx_message_rc_t ftx_message_decode(const ftx_message_t* msg, char* message)
{
  char buf[35];
  char* f1 = buf;
  char* f2 = buf + 14;
  char* f3 = buf + 28;
  message[0] = '\0';
  ftx_message_rc_t rc;
  switch (ftx_message_get_type(msg))
  {
    case FTX_MESSAGE_TYPE_STANDARD:    rc = ftx_message_decode_std(msg, f1, f2, f3); break;
    case FTX_MESSAGE_TYPE_NONSTD_CALL: rc = ftx_message_decode_nonstd(msg, f1, f2, f3); break;
    case FTX_MESSAGE_TYPE_FREE_TEXT:   ftx_message_decode_free(msg, f1); f2 = NULL; f3 = NULL; rc = FTX_MESSAGE_RC_OK; break;
    case FTX_MESSAGE_TYPE_TELEMETRY:   ftx_message_decode_telemetry_hex(msg, f1); f2 = NULL; f3 = NULL; rc = FTX_MESSAGE_RC_OK; break;
    default: f1 = NULL; rc = FTX_MESSAGE_RC_ERROR_TYPE; break;
  }
  if (f1)
  {
    const char* ms = message; message = append_string(message, f1);
    if (f2)
    {
      message = append_string(message, " ");
      message = append_string(message, f2);
      if (f3 && f3[0])
      {
        message = append_string(message, " ");
        message = append_string(message, f3);
      }
    }
  }
  return rc;
}

//------------------------------------------------------------------------------
// DECODE
//------------------------------------------------------------------------------
static const WF_ELEM_T* get_cand_mag(const ftx_waterfall_t* wf, const ftx_candidate_t* cand)
{
  int off = cand->time_offset;
  off = (off * FT8_TIME_OSR) + cand->time_sub;
  off = (off * FT8_FREQ_OSR) + cand->freq_sub;
  off = (off * FT8_WF_NUM_BINS) + cand->freq_offset;
  return wf->mag + off;
}

static float max2(float a, float b) { return (a >= b) ? a : b; }
static float max4(float a, float b, float c, float d) { return max2(max2(a, b), max2(c, d)); }

static void heapify_down(ftx_candidate_t h[], int n)
{
  int cur = 0;
  while (true)
  {
    const int l = 2 * cur + 1;
    const int r = l + 1;
    int s = cur;
    if (l < n && h[l].score < h[s].score) s = l;
    if (r < n && h[r].score < h[s].score) s = r;
    if (s == cur) break;
    const ftx_candidate_t t = h[s];
    h[s] = h[cur];
    h[cur] = t;
    cur = s;
  }
}

static void heapify_up(ftx_candidate_t h[], int n)
{
  int cur = n - 1;
  while (cur > 0)
  {
    const int p = (cur - 1) / 2;
    if (!(h[cur].score < h[p].score)) break;
    const ftx_candidate_t t = h[p];
    h[p] = h[cur];
    h[cur] = t;
    cur = p;
  }
}

static int ft8_sync_score(const ftx_waterfall_t* wf, const ftx_candidate_t* cand)
{
  int score = 0;
  int nav = 0;
  const WF_ELEM_T* mc = get_cand_mag(wf, cand);
  for (int m = 0; m < FT8_NUM_SYNC; ++m)
  {
    for (int k = 0; k < FT8_LENGTH_SYNC; ++k)
    {
      const int blk = (FT8_SYNC_OFFSET * m) + k;
      const int ba = cand->time_offset + blk;
      if (ba < 0) continue;
      if (ba >= wf->num_blocks) break;
      const WF_ELEM_T* p = mc + (blk * FT8_BLOCK_STRIDE);
      const int sm = kFT8_Costas_pattern[k];
      if (sm > 0) { score += WF_ELEM_MAG_INT(p[sm]) - WF_ELEM_MAG_INT(p[sm - 1]); ++nav; }
      if (sm < 7) { score += WF_ELEM_MAG_INT(p[sm]) - WF_ELEM_MAG_INT(p[sm + 1]); ++nav; }
      if (k > 0 && ba > 0) { score += WF_ELEM_MAG_INT(p[sm]) - WF_ELEM_MAG_INT(p[sm - FT8_BLOCK_STRIDE]); ++nav; }
      if ((k + 1) < FT8_LENGTH_SYNC && (ba + 1) < wf->num_blocks) { score += WF_ELEM_MAG_INT(p[sm]) - WF_ELEM_MAG_INT(p[sm + FT8_BLOCK_STRIDE]); ++nav; }
    }
  }
  if (nav > 0) score /= nav;
  return score;
}

static int ftx_find_candidates(const ftx_waterfall_t* wf, int num_cand, ftx_candidate_t heap[], int min_score)
{
  int hs = 0;
  ftx_candidate_t cand;
  for (cand.time_sub = 0; cand.time_sub < FT8_TIME_OSR; ++cand.time_sub)
    for (cand.freq_sub = 0; cand.freq_sub < FT8_FREQ_OSR; ++cand.freq_sub)
      for (cand.time_offset = -10; cand.time_offset < 20; ++cand.time_offset)
        for (cand.freq_offset = 0; (cand.freq_offset + 8 - 1) < FT8_WF_NUM_BINS; ++cand.freq_offset) {
          cand.score = ft8_sync_score(wf, &cand);
          if (cand.score < min_score)continue;
          if (hs == num_cand && cand.score > heap[0].score) { --hs; heap[0] = heap[hs]; heapify_down(heap, hs); }
          if (hs < num_cand) { heap[hs] = cand; ++hs; heapify_up(heap, hs); }
        }
  int lu = hs;
  while (lu > 1)
  {
    const ftx_candidate_t t = heap[lu - 1];
    heap[lu - 1] = heap[0];
    heap[0] = t;
    lu--;
    heapify_down(heap, lu);
  }
  return hs;
}

static void ft8_extract_symbol(const WF_ELEM_T* wf, float* logl)
{
  float s[8];
  for (int j = 0; j < 8; ++j) s[j] = WF_ELEM_MAG(wf[kFT8_Gray_map[j]]);
  logl[0] = max4(s[4], s[5], s[6], s[7]) - max4(s[0], s[1], s[2], s[3]);
  logl[1] = max4(s[2], s[3], s[6], s[7]) - max4(s[0], s[1], s[4], s[5]);
  logl[2] = max4(s[1], s[3], s[5], s[7]) - max4(s[0], s[2], s[4], s[6]);
}

static void ft8_extract_likelihood(const ftx_waterfall_t* wf, const ftx_candidate_t* cand, float* log174)
{
  const WF_ELEM_T* mag = get_cand_mag(wf, cand);
  for (int k = 0; k < FT8_ND; ++k)
  {
    int si = k + ((k < 29) ? 7 : 14), bi = 3 * k, blk = cand->time_offset + si;
    if (blk < 0 || blk >= wf->num_blocks) { log174[bi] = log174[bi + 1] = log174[bi + 2] = 0; }
    else ft8_extract_symbol(mag + (si * FT8_BLOCK_STRIDE), log174 + bi);
  }
}

static void ftx_normalize_logl(float* log174)
{
  float sum = 0, sum2 = 0;
  for (int i = 0; i < FTX_LDPC_N; ++i) { sum += log174[i]; sum2 += log174[i] * log174[i]; }
  float inv = 1.0f / FTX_LDPC_N, var = (sum2 - (sum * sum * inv)) * inv;
  float nf = sqrtf(24.0f / var);
  for (int i = 0; i < FTX_LDPC_N; ++i) log174[i] *= nf;
}

static void pack_bits(const uint8_t ba[], int nb, uint8_t pk[])
{
  int by = (nb + 7) / 8; for (int i = 0; i < by; ++i)pk[i] = 0;
  uint8_t mask = 0x80; int bi = 0;
  for (int i = 0; i < nb; ++i) { if (ba[i])pk[bi] |= mask; mask >>= 1; if (!mask) { mask = 0x80; ++bi; } }
}

static bool ftx_decode_candidate(const ftx_waterfall_t* wf, const ftx_candidate_t* cand, int max_iters, ftx_message_t* msg, ftx_decode_status_t* st)
{
  float log174[FTX_LDPC_N];
  ft8_extract_likelihood(wf, cand, log174);
  ftx_normalize_logl(log174);
  uint8_t plain[FTX_LDPC_N];
  bp_decode(log174, max_iters, plain, &st->ldpc_errors);
  if (st->ldpc_errors > 0) return false;
  uint8_t a91[FTX_LDPC_K_BYTES];
  pack_bits(plain, FTX_LDPC_K, a91);
  st->crc_extracted = ftx_extract_crc(a91);
  a91[9] &= 0xF8;
  a91[10] = 0x00;
  st->crc_calculated = ftx_compute_crc(a91, 96 - 14);
  if (st->crc_extracted != st->crc_calculated) return false;
  msg->hash = st->crc_calculated;
  for (int i = 0; i < 10; ++i)msg->payload[i] = a91[i];
  return true;
}

//------------------------------------------------------------------------------
// FT8 Encode
//------------------------------------------------------------------------------
// 
// Parity generator matrix for (174,91) LDPC code,
// stored in bitpacked format (MSB first)
static const uint8_t kFTX_LDPC_generator[FTX_LDPC_M][FTX_LDPC_K_BYTES] =
{
  { 0x83, 0x29, 0xce, 0x11, 0xbf, 0x31, 0xea, 0xf5, 0x09, 0xf2, 0x7f, 0xc0 },
  { 0x76, 0x1c, 0x26, 0x4e, 0x25, 0xc2, 0x59, 0x33, 0x54, 0x93, 0x13, 0x20 },
  { 0xdc, 0x26, 0x59, 0x02, 0xfb, 0x27, 0x7c, 0x64, 0x10, 0xa1, 0xbd, 0xc0 },
  { 0x1b, 0x3f, 0x41, 0x78, 0x58, 0xcd, 0x2d, 0xd3, 0x3e, 0xc7, 0xf6, 0x20 },
  { 0x09, 0xfd, 0xa4, 0xfe, 0xe0, 0x41, 0x95, 0xfd, 0x03, 0x47, 0x83, 0xa0 },
  { 0x07, 0x7c, 0xcc, 0xc1, 0x1b, 0x88, 0x73, 0xed, 0x5c, 0x3d, 0x48, 0xa0 },
  { 0x29, 0xb6, 0x2a, 0xfe, 0x3c, 0xa0, 0x36, 0xf4, 0xfe, 0x1a, 0x9d, 0xa0 },
  { 0x60, 0x54, 0xfa, 0xf5, 0xf3, 0x5d, 0x96, 0xd3, 0xb0, 0xc8, 0xc3, 0xe0 },
  { 0xe2, 0x07, 0x98, 0xe4, 0x31, 0x0e, 0xed, 0x27, 0x88, 0x4a, 0xe9, 0x00 },
  { 0x77, 0x5c, 0x9c, 0x08, 0xe8, 0x0e, 0x26, 0xdd, 0xae, 0x56, 0x31, 0x80 },
  { 0xb0, 0xb8, 0x11, 0x02, 0x8c, 0x2b, 0xf9, 0x97, 0x21, 0x34, 0x87, 0xc0 },
  { 0x18, 0xa0, 0xc9, 0x23, 0x1f, 0xc6, 0x0a, 0xdf, 0x5c, 0x5e, 0xa3, 0x20 },
  { 0x76, 0x47, 0x1e, 0x83, 0x02, 0xa0, 0x72, 0x1e, 0x01, 0xb1, 0x2b, 0x80 },
  { 0xff, 0xbc, 0xcb, 0x80, 0xca, 0x83, 0x41, 0xfa, 0xfb, 0x47, 0xb2, 0xe0 },
  { 0x66, 0xa7, 0x2a, 0x15, 0x8f, 0x93, 0x25, 0xa2, 0xbf, 0x67, 0x17, 0x00 },
  { 0xc4, 0x24, 0x36, 0x89, 0xfe, 0x85, 0xb1, 0xc5, 0x13, 0x63, 0xa1, 0x80 },
  { 0x0d, 0xff, 0x73, 0x94, 0x14, 0xd1, 0xa1, 0xb3, 0x4b, 0x1c, 0x27, 0x00 },
  { 0x15, 0xb4, 0x88, 0x30, 0x63, 0x6c, 0x8b, 0x99, 0x89, 0x49, 0x72, 0xe0 },
  { 0x29, 0xa8, 0x9c, 0x0d, 0x3d, 0xe8, 0x1d, 0x66, 0x54, 0x89, 0xb0, 0xe0 },
  { 0x4f, 0x12, 0x6f, 0x37, 0xfa, 0x51, 0xcb, 0xe6, 0x1b, 0xd6, 0xb9, 0x40 },
  { 0x99, 0xc4, 0x72, 0x39, 0xd0, 0xd9, 0x7d, 0x3c, 0x84, 0xe0, 0x94, 0x00 },
  { 0x19, 0x19, 0xb7, 0x51, 0x19, 0x76, 0x56, 0x21, 0xbb, 0x4f, 0x1e, 0x80 },
  { 0x09, 0xdb, 0x12, 0xd7, 0x31, 0xfa, 0xee, 0x0b, 0x86, 0xdf, 0x6b, 0x80 },
  { 0x48, 0x8f, 0xc3, 0x3d, 0xf4, 0x3f, 0xbd, 0xee, 0xa4, 0xea, 0xfb, 0x40 },
  { 0x82, 0x74, 0x23, 0xee, 0x40, 0xb6, 0x75, 0xf7, 0x56, 0xeb, 0x5f, 0xe0 },
  { 0xab, 0xe1, 0x97, 0xc4, 0x84, 0xcb, 0x74, 0x75, 0x71, 0x44, 0xa9, 0xa0 },
  { 0x2b, 0x50, 0x0e, 0x4b, 0xc0, 0xec, 0x5a, 0x6d, 0x2b, 0xdb, 0xdd, 0x00 },
  { 0xc4, 0x74, 0xaa, 0x53, 0xd7, 0x02, 0x18, 0x76, 0x16, 0x69, 0x36, 0x00 },
  { 0x8e, 0xba, 0x1a, 0x13, 0xdb, 0x33, 0x90, 0xbd, 0x67, 0x18, 0xce, 0xc0 },
  { 0x75, 0x38, 0x44, 0x67, 0x3a, 0x27, 0x78, 0x2c, 0xc4, 0x20, 0x12, 0xe0 },
  { 0x06, 0xff, 0x83, 0xa1, 0x45, 0xc3, 0x70, 0x35, 0xa5, 0xc1, 0x26, 0x80 },
  { 0x3b, 0x37, 0x41, 0x78, 0x58, 0xcc, 0x2d, 0xd3, 0x3e, 0xc3, 0xf6, 0x20 },
  { 0x9a, 0x4a, 0x5a, 0x28, 0xee, 0x17, 0xca, 0x9c, 0x32, 0x48, 0x42, 0xc0 },
  { 0xbc, 0x29, 0xf4, 0x65, 0x30, 0x9c, 0x97, 0x7e, 0x89, 0x61, 0x0a, 0x40 },
  { 0x26, 0x63, 0xae, 0x6d, 0xdf, 0x8b, 0x5c, 0xe2, 0xbb, 0x29, 0x48, 0x80 },
  { 0x46, 0xf2, 0x31, 0xef, 0xe4, 0x57, 0x03, 0x4c, 0x18, 0x14, 0x41, 0x80 },
  { 0x3f, 0xb2, 0xce, 0x85, 0xab, 0xe9, 0xb0, 0xc7, 0x2e, 0x06, 0xfb, 0xe0 },
  { 0xde, 0x87, 0x48, 0x1f, 0x28, 0x2c, 0x15, 0x39, 0x71, 0xa0, 0xa2, 0xe0 },
  { 0xfc, 0xd7, 0xcc, 0xf2, 0x3c, 0x69, 0xfa, 0x99, 0xbb, 0xa1, 0x41, 0x20 },
  { 0xf0, 0x26, 0x14, 0x47, 0xe9, 0x49, 0x0c, 0xa8, 0xe4, 0x74, 0xce, 0xc0 },
  { 0x44, 0x10, 0x11, 0x58, 0x18, 0x19, 0x6f, 0x95, 0xcd, 0xd7, 0x01, 0x20 },
  { 0x08, 0x8f, 0xc3, 0x1d, 0xf4, 0xbf, 0xbd, 0xe2, 0xa4, 0xea, 0xfb, 0x40 },
  { 0xb8, 0xfe, 0xf1, 0xb6, 0x30, 0x77, 0x29, 0xfb, 0x0a, 0x07, 0x8c, 0x00 },
  { 0x5a, 0xfe, 0xa7, 0xac, 0xcc, 0xb7, 0x7b, 0xbc, 0x9d, 0x99, 0xa9, 0x00 },
  { 0x49, 0xa7, 0x01, 0x6a, 0xc6, 0x53, 0xf6, 0x5e, 0xcd, 0xc9, 0x07, 0x60 },
  { 0x19, 0x44, 0xd0, 0x85, 0xbe, 0x4e, 0x7d, 0xa8, 0xd6, 0xcc, 0x7d, 0x00 },
  { 0x25, 0x1f, 0x62, 0xad, 0xc4, 0x03, 0x2f, 0x0e, 0xe7, 0x14, 0x00, 0x20 },
  { 0x56, 0x47, 0x1f, 0x87, 0x02, 0xa0, 0x72, 0x1e, 0x00, 0xb1, 0x2b, 0x80 },
  { 0x2b, 0x8e, 0x49, 0x23, 0xf2, 0xdd, 0x51, 0xe2, 0xd5, 0x37, 0xfa, 0x00 },
  { 0x6b, 0x55, 0x0a, 0x40, 0xa6, 0x6f, 0x47, 0x55, 0xde, 0x95, 0xc2, 0x60 },
  { 0xa1, 0x8a, 0xd2, 0x8d, 0x4e, 0x27, 0xfe, 0x92, 0xa4, 0xf6, 0xc8, 0x40 },
  { 0x10, 0xc2, 0xe5, 0x86, 0x38, 0x8c, 0xb8, 0x2a, 0x3d, 0x80, 0x75, 0x80 },
  { 0xef, 0x34, 0xa4, 0x18, 0x17, 0xee, 0x02, 0x13, 0x3d, 0xb2, 0xeb, 0x00 },
  { 0x7e, 0x9c, 0x0c, 0x54, 0x32, 0x5a, 0x9c, 0x15, 0x83, 0x6e, 0x00, 0x00 },
  { 0x36, 0x93, 0xe5, 0x72, 0xd1, 0xfd, 0xe4, 0xcd, 0xf0, 0x79, 0xe8, 0x60 },
  { 0xbf, 0xb2, 0xce, 0xc5, 0xab, 0xe1, 0xb0, 0xc7, 0x2e, 0x07, 0xfb, 0xe0 },
  { 0x7e, 0xe1, 0x82, 0x30, 0xc5, 0x83, 0xcc, 0xcc, 0x57, 0xd4, 0xb0, 0x80 },
  { 0xa0, 0x66, 0xcb, 0x2f, 0xed, 0xaf, 0xc9, 0xf5, 0x26, 0x64, 0x12, 0x60 },
  { 0xbb, 0x23, 0x72, 0x5a, 0xbc, 0x47, 0xcc, 0x5f, 0x4c, 0xc4, 0xcd, 0x20 },
  { 0xde, 0xd9, 0xdb, 0xa3, 0xbe, 0xe4, 0x0c, 0x59, 0xb5, 0x60, 0x9b, 0x40 },
  { 0xd9, 0xa7, 0x01, 0x6a, 0xc6, 0x53, 0xe6, 0xde, 0xcd, 0xc9, 0x03, 0x60 },
  { 0x9a, 0xd4, 0x6a, 0xed, 0x5f, 0x70, 0x7f, 0x28, 0x0a, 0xb5, 0xfc, 0x40 },
  { 0xe5, 0x92, 0x1c, 0x77, 0x82, 0x25, 0x87, 0x31, 0x6d, 0x7d, 0x3c, 0x20 },
  { 0x4f, 0x14, 0xda, 0x82, 0x42, 0xa8, 0xb8, 0x6d, 0xca, 0x73, 0x35, 0x20 },
  { 0x8b, 0x8b, 0x50, 0x7a, 0xd4, 0x67, 0xd4, 0x44, 0x1d, 0xf7, 0x70, 0xe0 },
  { 0x22, 0x83, 0x1c, 0x9c, 0xf1, 0x16, 0x94, 0x67, 0xad, 0x04, 0xb6, 0x80 },
  { 0x21, 0x3b, 0x83, 0x8f, 0xe2, 0xae, 0x54, 0xc3, 0x8e, 0xe7, 0x18, 0x00 },
  { 0x5d, 0x92, 0x6b, 0x6d, 0xd7, 0x1f, 0x08, 0x51, 0x81, 0xa4, 0xe1, 0x20 },
  { 0x66, 0xab, 0x79, 0xd4, 0xb2, 0x9e, 0xe6, 0xe6, 0x95, 0x09, 0xe5, 0x60 },
  { 0x95, 0x81, 0x48, 0x68, 0x2d, 0x74, 0x8a, 0x38, 0xdd, 0x68, 0xba, 0xa0 },
  { 0xb8, 0xce, 0x02, 0x0c, 0xf0, 0x69, 0xc3, 0x2a, 0x72, 0x3a, 0xb1, 0x40 },
  { 0xf4, 0x33, 0x1d, 0x6d, 0x46, 0x16, 0x07, 0xe9, 0x57, 0x52, 0x74, 0x60 },
  { 0x6d, 0xa2, 0x3b, 0xa4, 0x24, 0xb9, 0x59, 0x61, 0x33, 0xcf, 0x9c, 0x80 },
  { 0xa6, 0x36, 0xbc, 0xbc, 0x7b, 0x30, 0xc5, 0xfb, 0xea, 0xe6, 0x7f, 0xe0 },
  { 0x5c, 0xb0, 0xd8, 0x6a, 0x07, 0xdf, 0x65, 0x4a, 0x90, 0x89, 0xa2, 0x00 },
  { 0xf1, 0x1f, 0x10, 0x68, 0x48, 0x78, 0x0f, 0xc9, 0xec, 0xdd, 0x80, 0xa0 },
  { 0x1f, 0xbb, 0x53, 0x64, 0xfb, 0x8d, 0x2c, 0x9d, 0x73, 0x0d, 0x5b, 0xa0 },
  { 0xfc, 0xb8, 0x6b, 0xc7, 0x0a, 0x50, 0xc9, 0xd0, 0x2a, 0x5d, 0x03, 0x40 },
  { 0xa5, 0x34, 0x43, 0x30, 0x29, 0xea, 0xc1, 0x5f, 0x32, 0x2e, 0x34, 0xc0 },
  { 0xc9, 0x89, 0xd9, 0xc7, 0xc3, 0xd3, 0xb8, 0xc5, 0x5d, 0x75, 0x13, 0x00 },
  { 0x7b, 0xb3, 0x8b, 0x2f, 0x01, 0x86, 0xd4, 0x66, 0x43, 0xae, 0x96, 0x20 },
  { 0x26, 0x44, 0xeb, 0xad, 0xeb, 0x44, 0xb9, 0x46, 0x7d, 0x1f, 0x42, 0xc0 },
  { 0x60, 0x8c, 0xc8, 0x57, 0x59, 0x4b, 0xfb, 0xb5, 0x5d, 0x69, 0x60, 0x00 }
};

// Returns 1 if an odd number of bits are set in x, zero otherwise
static const uint8_t parity8(uint8_t x)
{
  x ^= x >> 4;  // a b c d ae bf cg dh
  x ^= x >> 2;  // a b ac bd cae dbf aecg bfdh
  x ^= x >> 1;  // a ab bac acbd bdcae caedbf aecgbfdh
  return x % 2; // modulo 2
}

// Encode via LDPC a 91-bit message and return a 174-bit codeword.
// The generator matrix has dimensions (87,87).
// The code is a (174,91) regular LDPC code with column weight 3.
// Arguments:
// [IN] message   - array of 91 bits stored as 12 bytes (MSB first)
// [OUT] codeword - array of 174 bits stored as 22 bytes (MSB first)
static void encode174(const uint8_t* message, uint8_t* codeword)
{
  // This implementation accesses the generator bits straight from
  // the packed binary representation in kFTX_LDPC_generator

  // Fill the codeword with message and zeros, as we will only update binary ones later
  for (int j = 0; j < FTX_LDPC_N_BYTES; ++j)
  {
    codeword[j] = (j < FTX_LDPC_K_BYTES) ? message[j] : 0;
  }

  // Compute the byte index and bit mask for the first checksum bit
  uint8_t col_mask = (0x80u >> (FTX_LDPC_K % 8u)); // bitmask of current byte
  uint8_t col_idx = FTX_LDPC_K_BYTES - 1;          // index into byte array

  // Compute the LDPC checksum bits and store them in codeword
  for (int i = 0; i < FTX_LDPC_M; ++i)
  {
    // Fast implementation of bitwise multiplication and parity checking
    // Normally nsum would contain the result of dot product between message and kFTX_LDPC_generator[i],
    // but we only compute the sum modulo 2.
    uint8_t nsum = 0;
    for (int j = 0; j < FTX_LDPC_K_BYTES; ++j)
    {
      uint8_t bits = message[j] & kFTX_LDPC_generator[i][j]; // bitwise AND (bitwise multiplication)
      nsum ^= parity8(bits);                                 // bitwise XOR (addition modulo 2)
    }

    // Set the current checksum bit in codeword if nsum is odd
    if (nsum % 2)
    {
      codeword[col_idx] |= col_mask;
    }

    // Update the byte index and bit mask for the next checksum bit
    col_mask >>= 1;
    if (col_mask == 0)
    {
      col_mask = 0x80u;
      ++col_idx;
    }
  }
}

static void ft8_encode(const uint8_t* payload, uint8_t* tones)
{
  uint8_t a91[FTX_LDPC_K_BYTES]; // Store 77 bits of payload + 14 bits CRC

  // Compute and add CRC at the end of the message
  // a91 contains 77 bits of payload + 14 bits of CRC
  ftx_add_crc(payload, a91);

  uint8_t codeword[FTX_LDPC_N_BYTES];
  encode174(a91, codeword);

  // Message structure: S7 D29 S7 D29 S7
  // Total symbols: 79 (FT8_NN)

  uint8_t mask = 0x80u; // Mask to extract 1 bit from codeword
  int i_byte = 0;       // Index of the current byte of the codeword
  for (int i_tone = 0; i_tone < FT8_NN; ++i_tone)
  {
    if ((i_tone >= 0) && (i_tone < 7))
    {
      tones[i_tone] = kFT8_Costas_pattern[i_tone];
    }
    else if ((i_tone >= 36) && (i_tone < 43))
    {
      tones[i_tone] = kFT8_Costas_pattern[i_tone - 36];
    }
    else if ((i_tone >= 72) && (i_tone < 79))
    {
      tones[i_tone] = kFT8_Costas_pattern[i_tone - 72];
    }
    else
    {
      // Extract 3 bits from codeword at i-th position
      uint8_t bits3 = 0;
      if (codeword[i_byte] & mask)
        bits3 |= 4;
      if (0 == (mask >>= 1))
      {
        mask = 0x80u;
        i_byte++;
      }
      if (codeword[i_byte] & mask)
        bits3 |= 2;
      if (0 == (mask >>= 1))
      {
        mask = 0x80u;
        i_byte++;
      }
      if (codeword[i_byte] & mask)
        bits3 |= 1;
      if (0 == (mask >>= 1))
      {
        mask = 0x80u;
        i_byte++;
      }
      tones[i_tone] = kFT8_Gray_map[bits3];
    }
  }
}

static const bool ft8_encode_message(const char *sz_message, uint8_t* tones)
{
  memset(tones, 0, FT8_NN);
  ftx_message_t msg = {0};
  ftx_message_rc_t rc = ftx_message_encode(&msg, sz_message);
  if (rc != FTX_MESSAGE_RC_OK)
  {
    return false;
  }
  ft8_encode(msg.payload, tones);
  return true;
}

//------------------------------------------------------------------------------
// MONITOR
// hann_window[FT8_FFT_SIZE] must be declared externally as a static const array.
// All geometry is encoded in the FT8_* compile-time constants.
//------------------------------------------------------------------------------

static void waterfall_init(ftx_waterfall_t* wf)
{
  static uint8_t mag_buffer[FT8_WATERFALL_SIZE];
  wf->num_blocks = 0;
  wf->mag = mag_buffer;
  LOG(LOG_INFO, "Waterfall: %d blocks * stride %d = %d bytes\n",
    FT8_MAX_BLOCKS, FT8_BLOCK_STRIDE, FT8_WATERFALL_SIZE);
}

static void ft8_monitor_init(monitor_t* me)
{
  static float frame_buffer[FT8_FFT_SIZE];

  me->last_frame = frame_buffer;
  me->max_mag = -120.0f;

  memset(frame_buffer, 0, sizeof(frame_buffer));
  waterfall_init(&me->wf);

  LOG(LOG_INFO, "Monitor init: block=%d sub=%d fft=%d bins=%d stride=%d wf=%d bytes\n",
    FT8_BLOCK_SIZE, FT8_SUBBLOCK_SIZE, FT8_FFT_SIZE,
    FT8_WF_NUM_BINS, FT8_BLOCK_STRIDE, FT8_WATERFALL_SIZE);
}

static void ft8_monitor_reset(monitor_t* me)
{
  me->wf.num_blocks = 0;
  me->max_mag = -120.0f;
}

static void ft8_monitor_process(monitor_t* me, const float* frame)
{
  // FFT working buffers - static, 20 KB BSS
  static float s_fft_re[FT8_FFT_SIZE] = { 0.0f };
  static float s_fft_im[FT8_FFT_SIZE] = { 0.0f };

  if (me->wf.num_blocks >= FT8_MAX_BLOCKS) return;

  uint32_t offset = me->wf.num_blocks * FT8_BLOCK_STRIDE;
  uint32_t frame_pos = 0;

  for (int time_sub = 0; time_sub < FT8_TIME_OSR; ++time_sub)
  {
    // Shift analysis frame left by one subblock, bring in new samples
    for (int pos = 0; pos < FT8_FFT_SIZE - FT8_SUBBLOCK_SIZE; ++pos)
      me->last_frame[pos] = me->last_frame[pos + FT8_SUBBLOCK_SIZE];
    for (int pos = FT8_FFT_SIZE - FT8_SUBBLOCK_SIZE; pos < FT8_FFT_SIZE; ++pos)
      me->last_frame[pos] = frame[frame_pos++];
    //LOG(LOG_INFO, "frame_pos = %d\n", frame_pos);

    // Apply window and clear imaginary part
    for (int pos = 0; pos < FT8_FFT_SIZE; ++pos)
    {
      s_fft_re[pos] = hann_window[pos] * me->last_frame[pos];
      s_fft_im[pos] = 0.0f;
    }

    // note, expects buffer size to be 2500
    ft8_fft_forward(s_fft_re, s_fft_im);

    // Store magnitude in waterfall for each freq oversampling offset
    for (int freq_sub = 0; freq_sub < FT8_FREQ_OSR; ++freq_sub)
    {
      for (int bin = FT8_MIN_BIN; bin < FT8_MAX_BIN; ++bin)
      {
        const int src = bin * FT8_FREQ_OSR + freq_sub;
        const float re = s_fft_re[src];
        const float im = s_fft_im[src];
        static const float log10e = 10.0f / logf(10.0f);
        const float db = log10e * logf(1E-12f + re * re + im * im);
        const int sc = (int)(2.0f * db + 240.0f);
        assert(offset < FT8_WATERFALL_SIZE);
        me->wf.mag[offset++] = (sc < 0) ? 0 : ((sc > 255) ? 255 : (uint8_t)sc);
        if (db > me->max_mag) me->max_mag = db;
      }
    }
  }
  ++me->wf.num_blocks;
}

//------------------------------------------------------------------------------
// Estimate noise floor as 25th percentile of waterfall magnitudes.
// Returns the encoded value (uint8 in range 0..255).
//------------------------------------------------------------------------------
static const int ft8_estimate_noise_floor(const ftx_waterfall_t* wf)
{
  static uint32_t hist[256] = { 0 };
  memset(hist,0,sizeof(hist));
  const uint32_t total = wf->num_blocks * FT8_BLOCK_STRIDE;
  for (uint32_t i = 0; i < total; i++) hist[wf->mag[i]]++;
  const uint32_t target = total / 4;   // 25th percentile
  uint32_t cum = 0;
  for (int i = 0; i < 256; i++)
  {
    cum += hist[i];
    if (cum >= target) return i;
  }
  return 0;
}

//------------------------------------------------------------------------------
// Decode one slot
//------------------------------------------------------------------------------
static const uint32_t ft8_decode(
  const monitor_t* mon,
  char ft8_lines[FT8_MAX_DECODED][FTX_MAX_DISPLAY_LENGTH],
  ft8_ui_cb_t ui_callback = NULL)
{
  ftx_message_t message = { 0 };
  ftx_decode_status_t status = { 0 };
  static ftx_candidate_t candidate_list[FT8_MAX_CANDIDATES];
  static ftx_message_t decoded[FT8_MAX_DECODED];
  static ftx_message_t* decoded_ht[FT8_MAX_DECODED];
  memset(candidate_list, 0, sizeof(candidate_list));
  memset(decoded, 0, sizeof(decoded));
  memset(decoded_ht, 0, sizeof(decoded_ht));

  const ftx_waterfall_t* wf = &mon->wf;
  const int num_candidates = ftx_find_candidates(wf, FT8_MAX_CANDIDATES, candidate_list, FT8_MIN_SCORE);
  const int noise_floor = ft8_estimate_noise_floor(wf);
  uint32_t num_decoded = 0;

  for (int idx = 0; idx < num_candidates; ++idx)
  {
    // Periodic UI update - call every N candidates
    if (ui_callback && (idx % 16 == 0))
    {
      ui_callback();
    }
    
    const ftx_candidate_t* cand = &candidate_list[idx];

    const float freq_hz = (FT8_MIN_BIN + cand->freq_offset
      + (float)cand->freq_sub / FT8_FREQ_OSR) / FT8_SYMBOL_PERIOD;
    const float time_sec = (cand->time_offset
      + (float)cand->time_sub / FT8_TIME_OSR) * FT8_SYMBOL_PERIOD;

    if (!ftx_decode_candidate(wf, cand, FT8_LDPC_ITERATIONS, &message, &status))
    {
      if (status.ldpc_errors > 0)
        LOG(LOG_DEBUG, "LDPC decode: %d errors\n", status.ldpc_errors);
      else if (status.crc_calculated != status.crc_extracted)
        LOG(LOG_DEBUG, "CRC mismatch!\n");
      continue;
    }

    LOG(LOG_DEBUG, "Checking hash table for %4.1fs / %4.1fHz [%d]...\n",
      time_sec, freq_hz, cand->score);

    int idx_hash = message.hash % FT8_MAX_DECODED;
    bool found_empty = false;
    bool found_duplicate = false;
    int probes = 0;
    do
    {
      if (decoded_ht[idx_hash] == NULL)
      {
        found_empty = true;
      }
      else if ((decoded_ht[idx_hash]->hash == message.hash) &&
        (0 == memcmp(decoded_ht[idx_hash]->payload, message.payload, sizeof(message.payload))))
      {
        found_duplicate = true;
      }
      else
      {
        LOG(LOG_DEBUG, "Hash table clash!\n");
        idx_hash = (idx_hash + 1) % FT8_MAX_DECODED;
        if (++probes >= FT8_MAX_DECODED) break;
      }
    }
    while (!found_empty && !found_duplicate);

    if (found_empty)
    {
      memcpy(&decoded[idx_hash], &message, sizeof(message));
      decoded_ht[idx_hash] = &decoded[idx_hash];

      char text[FTX_MAX_MESSAGE_LENGTH] = "";
      ftx_message_rc_t rc = ftx_message_decode(&message, text);
      if (rc != FTX_MESSAGE_RC_OK)
        snprintf(text, sizeof(text), "Error [%d] unpacking!", (int)rc);

      // Estimate signal level: average magnitude across this candidate's sync tones
      const WF_ELEM_T* mc = get_cand_mag(wf, cand);
      int sig_sum = 0, sig_n = 0;
      for (int m = 0; m < FT8_NUM_SYNC; ++m)
      {
        for (int k = 0; k < FT8_LENGTH_SYNC; ++k)
        {
          const int blk = (FT8_SYNC_OFFSET * m) + k;
          const int ba  = cand->time_offset + blk;
          if (ba < 0) continue;
          if (ba >= wf->num_blocks) break;
          const WF_ELEM_T* p = mc + (blk * FT8_BLOCK_STRIDE);
          sig_sum += (int)p[kFT8_Costas_pattern[k]];
          sig_n++;
        }
      }
      const int sig_sc = (sig_n > 0) ? (sig_sum / sig_n) : 0;
      const float sig_db = (float)sig_sc * 0.5f - 120.0f;
      const float noise_db = (float)noise_floor * 0.5f - 120.0f;

      // SNR per bin, normalised to 2500 Hz reference BW (10*log10(2500/3.125) = 29 dB)
      const float snr = sig_db - noise_db - 29.0f;

      snprintf(ft8_lines[num_decoded], FTX_MAX_DISPLAY_LENGTH, "%+05.1f %+4.2f %4.0f %s", snr, time_sec, freq_hz, text);
      ++num_decoded;
      if (num_decoded >= FT8_MAX_DECODED) break; 
    }
  }

  LOG(LOG_INFO, "Decoded %d messages, callsign hashtable size %d\n",
    num_decoded, callsign_hashtable_size);
  ft8_hashtable_cleanup(10);
  return num_decoded;
}
