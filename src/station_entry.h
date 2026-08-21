/*
 * station_entry.h
 *
 * Callsign and 4-character Maidenhead grid entry for uDST / MBPTRX.
 * 240 x 135 display, drawn into the "lcd" sprite (TFT_eSPI2).
 *
 * User interface - one short push does everything, no timed gestures:
 *
 *      +--------------------------------------------+
 *      |                 CALLSIGN                   |
 *      |        V K 7 I A N _ _ _ _ _               |
 *      |      [ BACK ] [ SAVE ] [ CANCEL ]          |
 *      |         TURN = SELECT  PUSH = CONFIRM      |
 *      +--------------------------------------------+
 *
 *   on a character cell   turn = change the character
 *                         push = move to the next cell
 *                                (the last cell jumps to SAVE)
 *   on a button           turn = move between the buttons
 *                         push = do it
 *
 *   BACK returns the cursor to the first character, so a typo half way
 *   along is fixed without touching SAVE or CANCEL.
 *
 * This is a blocking modal editor.  Call it only from the core that owns the
 * display and the encoder:
 *      uDST    - core0, loop()      (DSP is on core1, audio keeps running)
 *      MBPTRX  - whichever core runs the UI, NOT the DSP core
 *
 * Public API:
 *      void check_station_details(void);          // after restore_settings()
 *      bool enter_station_details(bool cancel);   // on demand, true = saved
 */

#ifndef STATION_ENTRY_H
#define STATION_ENTRY_H

#include <Arduino.h>
#include <string.h>

/* ==========================================================================
 *  ADAPTATION SECTION - the only part that should need editing
 * ========================================================================== */

/* Add these two fields to the struct that save_settings()/restore_settings()
 * move in and out of EEPROM, then bump the settings magic/version so an old
 * EEPROM image is not read back as garbage:
 *
 *      char callsign[STATION_CALL_BUF];   // 12 bytes
 *      char grid[STATION_GRID_BUF];       //  5 bytes
 */
#define ENTRY_CALLSIGN radio.callsign
#define ENTRY_GRID     radio.grid

/* Sprite, and however update_display() sends it to the panel. */
#define ENTRY_LCD    lcd
#define ENTRY_PUSH() do { lcd.pushSprite(0,0); } while (0)

/* Raw button level: true while the encoder button is pressed. */
static inline bool entry_button(void)
{
  return digitalRead(PIN_ENCBUT) == LOW;
}

/* Panel geometry and RGB565 colours (swap in your LCD_* names if you like). */
#define ENTRY_W             240
#define ENTRY_H             135

#define ENTRY_Y_TITLE       4       /* top of the title, size 2         */
#define ENTRY_Y_CELLS       42      /* vertical centre of the char row  */
#define ENTRY_Y_BUTTONS     70      /* top of the button row            */
#define ENTRY_H_BUTTONS     26
#define ENTRY_Y_HINT1       104
#define ENTRY_Y_HINT2       116

#define ENTRY_COL_BG        0x0000
#define ENTRY_COL_TITLE     0xFFE0
#define ENTRY_COL_TEXT      0xFFFF
#define ENTRY_COL_CURSOR    0x001F
#define ENTRY_COL_HINT      0x07FF
#define ENTRY_COL_BACK      0xC618
#define ENTRY_COL_SAVE      0x07E0
#define ENTRY_COL_CANCEL    0xF800

/* ==========================================================================
 *  Nothing below here should need changing
 * ========================================================================== */

#define STATION_CALL_MAX    6
#define STATION_CALL_BUF    (STATION_CALL_MAX + 1)
#define STATION_GRID_LEN    4
#define STATION_GRID_BUF    (STATION_GRID_LEN + 1)

#define ENTRY_DEBOUNCE_MS   25u
#define ENTRY_FRAME_MS      50u     /* 20 Hz, same as the main display */
#define ENTRY_ERROR_MS      1500u

/* Space is the "no character here" entry and must be first. */
static const char ENTRY_RING_CALL[]   = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/";
static const char ENTRY_RING_FIELD[]  = "ABCDEFGHIJKLMNOPQR";
static const char ENTRY_RING_SQUARE[] = "0123456789";

/* Buttons, in the order they appear on screen.  CANCEL is dropped when the
 * entry is compulsory, so it cannot be reached at all. */
#define ENTRY_ACT_BACK      0
#define ENTRY_ACT_SAVE      1
#define ENTRY_ACT_CANCEL    2

static const char *const ENTRY_ACT_LABEL[3] = { "BACK", "SAVE", "CANCEL" };
static const uint16_t ENTRY_ACT_COL[3] =
{
  ENTRY_COL_BACK, ENTRY_COL_SAVE, ENTRY_COL_CANCEL
};

typedef enum
{
  ENTRY_MODE_CALL = 0,
  ENTRY_MODE_GRID
} entry_mode_t;

/* --------------------------------------------------------------------------
 *  Helpers
 * -------------------------------------------------------------------------- */

/* Bounded length: returns cap if no terminator is found (erased flash). */
static const uint8_t entry_len(const char *s, const uint8_t cap)
{
  uint8_t n = 0;
  while (n < cap && s[n] != '\0')
  {
    n++;
  }
  return n;
}

static const int16_t entry_index(const char *ring, const char c)
{
  for (int16_t i = 0; ring[i] != '\0'; i++)
  {
    if (ring[i] == c)
    {
      return i;
    }
  }
  return 0;
}

static const char *entry_ring(const entry_mode_t mode, const uint8_t pos)
{
  if (mode == ENTRY_MODE_GRID)
  {
    return (pos < 2u) ? ENTRY_RING_FIELD : ENTRY_RING_SQUARE;
  }
  return ENTRY_RING_CALL;
}

/* A callsign needs at least one letter and one digit, and may contain '/'. */
static const bool entry_call_valid(const char *s)
{
  const uint8_t n = entry_len(s, STATION_CALL_BUF);
  bool has_alpha = false;
  bool has_digit = false;

  if (n < 3u || n > STATION_CALL_MAX)
  {
    return false;
  }
  for (uint8_t i = 0; i < n; i++)
  {
    const char c = s[i];
    if (c >= 'A' && c <= 'Z')
    {
      has_alpha = true;
    }
    else if (c >= '0' && c <= '9')
    {
      has_digit = true;
    }
    else if (c != '/')
    {
      return false;
    }
  }
  return has_alpha && has_digit;
}

static bool entry_grid_valid(const char *s)
{
  if (entry_len(s, STATION_GRID_BUF) != STATION_GRID_LEN)
  {
    return false;
  }
  return (s[0] >= 'A' && s[0] <= 'R') &&
         (s[1] >= 'A' && s[1] <= 'R') &&
         (s[2] >= '0' && s[2] <= '9') &&
         (s[3] >= '0' && s[3] <= '9');
}

/* --------------------------------------------------------------------------
 *  Encoder and button
 * -------------------------------------------------------------------------- */

static inline void entry_encoder_reset(void)
{
  r.process();
}

static bool entry_btn_down = false;
static uint32_t entry_btn_t = 0;

/* Fires once on the press edge, bounce ignored. */
static bool entry_pressed(void)
{
  const bool down = entry_button();
  const uint32_t now = millis();

  if (down != entry_btn_down && (now - entry_btn_t) >= ENTRY_DEBOUNCE_MS)
  {
    entry_btn_down = down;
    entry_btn_t = now;
    return down;
  }
  return false;
}

/* Swallow the press that got us here, so it is not seen by the editor. */
static void entry_button_reset(void)
{
  uint32_t stable = millis();
  while (millis() - stable < ENTRY_DEBOUNCE_MS)
  {
    r.process();
    if (entry_button())
    {
      stable = millis();
    }
  }
  entry_btn_down = false;
  entry_btn_t = millis();
}

/* --------------------------------------------------------------------------
 *  Drawing - the whole sprite is rebuilt each frame
 * -------------------------------------------------------------------------- */

static void entry_centre(const int16_t y, const uint8_t size,
  const uint16_t colour, const char *s)
{
  const int16_t w = (int16_t)(6 * size * (int)strlen(s));
  ENTRY_LCD.setTextFont(1);
  ENTRY_LCD.setTextSize(size);
  ENTRY_LCD.setTextColor(colour);
  ENTRY_LCD.setCursor((ENTRY_W - w) / 2, y);
  ENTRY_LCD.print(s);
}

static int16_t entry_button_width(const uint8_t act)
{
  return (int16_t)(12 * (int)strlen(ENTRY_ACT_LABEL[act]) + 12);
}

static void entry_draw(const char *title, const char *buf, const uint8_t nchars,
  const uint8_t cursor, const uint8_t nact,
  const char *hint1, const char *hint2,
  const uint16_t hint_colour)
{
  /* fewer cells, bigger characters */
  const uint8_t size = (nchars <= 6u) ? 3u : 2u;
  const int16_t cw = (int16_t)(6 * size + 8);
  const int16_t ch = (int16_t)(8 * size + 8);
  const int16_t cy = ENTRY_Y_CELLS - (ch / 2);
  const int16_t cx = (ENTRY_W - (cw * (int16_t)nchars)) / 2;
  const int16_t gap = 8;
  int16_t total = gap * (int16_t)(nact - 1u);
  int16_t bx;

  ENTRY_LCD.fillSprite(ENTRY_COL_BG);
  entry_centre(ENTRY_Y_TITLE, 2, ENTRY_COL_TITLE, title);

  /* character cells */
  ENTRY_LCD.setTextFont(1);
  ENTRY_LCD.setTextSize(size);
  for (uint8_t i = 0; i < nchars; i++)
  {
    const int16_t x = cx + (cw * (int16_t)i);
    const bool active = (i == cursor);

    if (active)
    {
      ENTRY_LCD.fillRect(x, cy, cw, ch, ENTRY_COL_CURSOR);
      ENTRY_LCD.drawFastHLine(x + 1, cy + ch + 2, cw - 2, ENTRY_COL_TITLE);
    }
    ENTRY_LCD.setTextColor(ENTRY_COL_TEXT);
    ENTRY_LCD.setCursor(x + 4, cy + 4);
    ENTRY_LCD.print(buf[i]);
  }

  /* button row */
  for (uint8_t a = 0; a < nact; a++)
  {
    total += entry_button_width(a);
  }
  bx = (ENTRY_W - total) / 2;

  ENTRY_LCD.setTextSize(2);
  for (uint8_t a = 0; a < nact; a++)
  {
    const int16_t bw = entry_button_width(a);
    const bool active = (cursor == (uint8_t)(nchars + a));
    const int16_t tw = (int16_t)(12 * (int)strlen(ENTRY_ACT_LABEL[a]));

    if (active)
    {
      ENTRY_LCD.fillRoundRect(bx, ENTRY_Y_BUTTONS, bw, ENTRY_H_BUTTONS, 4,
                              ENTRY_ACT_COL[a]);
      ENTRY_LCD.drawRoundRect(bx - 2, ENTRY_Y_BUTTONS - 2, bw + 4,
                              ENTRY_H_BUTTONS + 4, 5, ENTRY_COL_TEXT);
      ENTRY_LCD.setTextColor(ENTRY_COL_BG);
    }
    else
    {
      ENTRY_LCD.drawRoundRect(bx, ENTRY_Y_BUTTONS, bw, ENTRY_H_BUTTONS, 4,
                              ENTRY_ACT_COL[a]);
      ENTRY_LCD.setTextColor(ENTRY_ACT_COL[a]);
    }
    ENTRY_LCD.setCursor(bx + ((bw - tw) / 2),
                        ENTRY_Y_BUTTONS + ((ENTRY_H_BUTTONS - 16) / 2));
    ENTRY_LCD.print(ENTRY_ACT_LABEL[a]);
    bx += bw + gap;
  }

  if (hint1 != NULL)
  {
    entry_centre(ENTRY_Y_HINT1, 1, hint_colour, hint1);
  }
  if (hint2 != NULL)
  {
    entry_centre(ENTRY_Y_HINT2, 1, hint_colour, hint2);
  }
  ENTRY_PUSH();
}

/* --------------------------------------------------------------------------
 *  The editor
 * -------------------------------------------------------------------------- */

static const bool entry_editor(const char *title, char *buf, const uint8_t nchars, const entry_mode_t mode)
{
  const uint8_t nact = 2u;
  const uint8_t nslots = (uint8_t)(nchars + nact);
  uint8_t cursor = 0;
  uint32_t last_frame = 0;
  uint32_t error_until = 0;
  bool error_shown = false;
  bool dirty = true;

  entry_encoder_reset();
  entry_button_reset();

  for (;;)
  {
    const uint32_t now = millis();
    const bool error_now = ((int32_t)(error_until - now) > 0);

    /* ---- encoder ---- */
    int32_t rotary = 0l;
    switch (r.process()) 
    {
      case DIR_CW:  rotary = 1;  break;
      case DIR_CCW: rotary = -1; break;
      default: break;
    } 
    if (rotary != 0)
    {
      if (cursor < nchars)
      {
        /* on a character cell: change the character */
        const char *ring = entry_ring(mode, cursor);
        const int32_t rlen = (int32_t)strlen(ring);
        int32_t idx = (int32_t)entry_index(ring, buf[cursor]) + rotary;
        idx %= rlen;
        if (idx < 0)
        {
          idx += rlen;
        }
        buf[cursor] = ring[idx];
      }
      else
      {
        /* on a button: move between the buttons, clamped at each end so a
         * fast spin cannot wrap round onto CANCEL */
        int32_t slot = (int32_t)cursor + rotary;
        if (slot < (int32_t)nchars)
        {
          slot = (int32_t)nchars;
        }
        if (slot > (int32_t)(nslots - 1u))
        {
          slot = (int32_t)(nslots - 1u);
        }
        cursor = (uint8_t)slot;
      }
      dirty = true;
    }

    /* ---- button ---- */
    if (entry_pressed())
    {
      dirty = true;
      if (cursor + 1u < nchars)
      {
        // next characgter
        cursor++;
      }
      else if (cursor < nchars)
      {
        // last cell, SAVE
        cursor = (uint8_t)(nchars + ENTRY_ACT_SAVE);
      }
      else
      {
        switch ((uint8_t)(cursor - nchars))
        {
          case ENTRY_ACT_BACK:
          {
            cursor = 0;
            break;
          }
          case ENTRY_ACT_CANCEL:
          {
            return false;
          }
          case ENTRY_ACT_SAVE:
          default:
          {
            if (mode == ENTRY_MODE_CALL)
            {
              /* squeeze out the blanks, validate, re-pad if rejected */
              uint8_t n = 0;
              for (uint8_t i = 0; i < nchars; i++)
              {
                if (buf[i] != ' ')
                {
                  buf[n++] = buf[i];
                }
              }
              buf[n] = '\0';
              if (entry_call_valid(buf))
              {
                return true;
              }
              for (uint8_t i = n; i < nchars; i++)
              {
                buf[i] = ' ';
              }
              buf[nchars] = '\0';
            }
            else
            {
              buf[nchars] = '\0';
              if (entry_grid_valid(buf))
              {
                return true;
              }
            }
            error_until = millis() + ENTRY_ERROR_MS;
            break;
          }
        }
      }
    }

    // redraw
    if (error_now != error_shown)
    {
      error_shown = error_now;
      dirty = true;
    }
    if (dirty && (now - last_frame >= ENTRY_FRAME_MS))
    {
      last_frame = now;
      dirty = false;
      if (error_now)
      {
        entry_draw(title, buf, nchars, cursor, nact,
          "NEED A LETTER AND A DIGIT",
          "3 TO 11 CHARACTERS", ENTRY_COL_CANCEL);
      }
      else if (cursor < nchars)
      {
        entry_draw(title, buf, nchars, cursor, nact,
          "TURN = CHARACTER    PUSH = NEXT", NULL, ENTRY_COL_HINT);
      }
      else
      {
        entry_draw(title, buf, nchars, cursor, nact,
          "TURN = SELECT    PUSH = CONFIRM", NULL, ENTRY_COL_HINT);
      }
    }
  }
}

/* Load an existing value into a space-padded working buffer. */
static void entry_load(char *work, const uint8_t nchars,
  const char *src, const bool valid, const char *fallback)
{
  const char *from = valid ? src : fallback;
  const uint8_t n = entry_len(from, (uint8_t)(nchars + 1u));
  uint8_t i = 0;

  for (; i < n && i < nchars; i++)
  {
    work[i] = from[i];
  }
  for (; i < nchars; i++)
  {
    work[i] = ' ';
  }
  work[nchars] = '\0';
}

/* --------------------------------------------------------------------------
 *  Public entry points
 * -------------------------------------------------------------------------- */

/*
 * Enter callsign then grid.  Returns true if both were accepted and saved.
 * Pass allow_cancel = false to make it compulsory (first power up) - the
 * CANCEL button is then not drawn and cannot be selected.
 */
static const bool enter_station_details(void)
{
  char call[STATION_CALL_BUF];
  char grid[STATION_GRID_BUF];

  entry_load(call, STATION_CALL_MAX, (const char*)ENTRY_CALLSIGN, entry_call_valid((const char*)ENTRY_CALLSIGN), "");
  entry_load(grid, STATION_GRID_LEN, (const char*)ENTRY_GRID, entry_grid_valid((const char*)ENTRY_GRID), "AA00");

  if (!entry_editor("CALLSIGN", call, STATION_CALL_MAX, ENTRY_MODE_CALL))
  {
    return false;
  }
  if (!entry_editor("GRID SQUARE", grid, STATION_GRID_LEN, ENTRY_MODE_GRID))
  {
    return false;
  }

  memcpy((char*)ENTRY_CALLSIGN, call, STATION_CALL_BUF);
  memcpy((char*)ENTRY_GRID, grid, STATION_GRID_BUF);

  ENTRY_LCD.fillSprite(ENTRY_COL_BG);
  entry_centre(ENTRY_Y_TITLE, 2, ENTRY_COL_TITLE, "SAVED");
  entry_centre(ENTRY_Y_CELLS - 8, 2, ENTRY_COL_TEXT, (const char *)ENTRY_CALLSIGN);
  entry_centre(ENTRY_Y_BUTTONS, 2, ENTRY_COL_TEXT, (const char *)ENTRY_GRID);
  ENTRY_PUSH();

  return true;
}

/*
 * Call immediately after restore_settings(), once the sprite and the encoder
 * are both alive.  Forces entry if the stored values are missing or corrupt.
 */
static const bool check_station_details(void)
{
  /* an erased or short EEPROM image must not run off the end */
  ENTRY_CALLSIGN[STATION_CALL_BUF - 1] = '\0';
  ENTRY_GRID[STATION_GRID_BUF - 1] = '\0';

  if (entry_call_valid((const char*)ENTRY_CALLSIGN) && entry_grid_valid((const char*)ENTRY_GRID))
  {
    return true;
  }
  return false;
}

#endif /* STATION_ENTRY_H */
