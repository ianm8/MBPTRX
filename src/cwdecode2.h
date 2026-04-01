/*
 * cwdecode2.h
 *
 * INTERFACE  (identical to cwdecode1.h)
 * ---------
 *   uint8_t morse_decode(float sample);
 *
 *   Call once per sample at 31 250 Hz with the bandpass-filtered signal.
 *   Returns a uint8_t each call:
 *     Bit 7 (0x80)  — MARK flag, live tone-on indicator
 *     Bits 6..0     — decoded ASCII character, valid for one call only,
 *                     then 0x00.  0x20 (' ') = inter-word gap.
 *
 *   MD2_MARK_BIT(r)  — non-zero while tone is ON
 *   MD2_CHAR(r)      — decoded character, or '\0'
 *
 * HOW THIS ALGORITHM DIFFERS FROM morse_decoder.h
 * ------------------------------------------------
 * 1. Schmitt trigger instead of single adaptive threshold.
 *    Two separate thresholds, ON_FRAC and OFF_FRAC, derived from a tracked
 *    signal level.  The detector only flips ON when the envelope exceeds the
 *    higher threshold, and only flips OFF when it drops below the lower one.
 *    The gap between the two thresholds (hysteresis) prevents chattering
 *    when the signal sits near the boundary — eliminating false mark/space
 *    transitions and the need for a separate noise gate entirely.
 *
 * 2. Asymmetric dot estimator instead of symmetric IIR.
 *    dot_est converges QUICKLY toward shorter durations (fast attack: sender
 *    speeds up, or initial convergence from a high seed) but SLOWLY toward
 *    longer ones (slow decay: sender slows down, or a noise/impulse inflates
 *    a mark).  This is the key robustness property: dot_est cannot be driven
 *    upward quickly by interference, so the ELEM_CHAR boundary cannot be
 *    inflated into lockup the way it could with a symmetric IIR.
 *
 * 3. Level tracker is always active (not just during marks).
 *    A fast-up / slow-down IIR on the raw envelope tracks the signal peak.
 *    This gives the Schmitt trigger its reference level.  It rises
 *    immediately when a signal appears and decays slowly in its absence,
 *    keeping the thresholds meaningful without a separate IDLE/MARK switch.
 *
 * Everything else — state machine structure, gap classification thresholds,
 * dot/dash boundaries, output encoding, clamps, overflow flush — is the same.
 */

#ifndef MORSE_DECODER2_H
#define MORSE_DECODER2_H

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace CWDECODE2
{

  /* ── Build-time configuration ────────────────────────────────────────────── */

  #define MD2_SAMPLE_RATE 31250 /* Hz – must match ADC / filter chain */
  #define MD2_MAX_CODE_LEN 8 /* longest supported Morse pattern */

  /* Envelope follower time constants (seconds) */
  #define MD2_ENV_ATTACK_TC 0.001f /* 1 ms  – fast, catches mark onset */
  #define MD2_ENV_DECAY_TC 0.008f /* 8 ms  – bridges 700 Hz carrier cycles */

  /*
   * Signal level tracker — fast-up / slow-down IIR on the envelope.
   * Rises quickly when a signal appears; decays slowly during silence so the
   * Schmitt thresholds remain meaningful for several seconds after signal loss.
   */
  #define MD2_LVL_ATTACK_TC 0.005f /* 5 ms – fast rise */
  #define MD2_LVL_DECAY_TC 3.000f  /* 3 s - slow fall */

  /*
   * Schmitt trigger fractions (relative to tracked signal level).
   * Hysteresis ratio = ON_FRAC / OFF_FRAC = 0.55 / 0.22 = 2.5 : 1.
   * The signal must rise to 55 % of its tracked peak to turn ON, and must
   * fall below 22 % of that peak before it turns OFF.
   */
  #define MD2_ON_FRAC 0.55f
  #define MD2_OFF_FRAC 0.22f

  /* Gap classification boundaries (multiples of dot_est) — same as decoder 1 */
  #define MD2_DOT_DASH_BOUND 2.0f
  #define MD2_ELEM_CHAR_BOUND 2.5f
  #define MD2_CHAR_WORD_BOUND 6.0f

  /*
   * Asymmetric dot estimator rates.
   * FAST_RATE : applied when the new observation is SHORTER than dot_est.
   *             Converges quickly so the decoder tracks a speeding-up sender
   *             and locks on from a high seed within a few characters.
   * SLOW_RATE : applied when the new observation is LONGER than dot_est.
   *             Resists upward drift from noise, impulses, or a single
   *             stretched mark — the main lockup mechanism in decoder 1.
   */
  #define MD2_DOT_FAST_RATE 0.35f
  #define MD2_DOT_SLOW_RATE 0.04f

  /* Hard clamp on dot_est — identical to decoder 1. */
  #define MD2_DOT_MIN_SAMP (0.035f * (float)MD2_SAMPLE_RATE) /* ~34 WPM */
  #define MD2_DOT_MAX_SAMP (0.150f * (float)MD2_SAMPLE_RATE) /*  ~8 WPM */

  /*
   * Seed: geometric mean of min/max ≈ 17 WPM — minimises worst-case startup
   * error in both directions.
   */
  #define MD2_INITIAL_DOT_SAMP (0.072f * (float)MD2_SAMPLE_RATE)

  /* ── Private: types ──────────────────────────────────────────────────────── */

  typedef enum { MD2_IDLE, MD2_MARK, MD2_SPACE } md2_state_t;

  typedef struct
  {
    /* Envelope follower */
    float att; /* IIR coef, attack */
    float dec; /* IIR coef, decay */
    float envelope;

    /* Signal level tracker for Schmitt trigger reference */
    float lvl_att; /* IIR coef, level attack */
    float lvl_dec; /* IIR coef, level decay */
    float sig_level; /* tracked signal peak */

    /* Schmitt trigger state — remembered between samples */
    int schmitt_on; /* 1 = tone currently detected ON */

    /* State machine */
    md2_state_t state;
    uint32_t count; /* samples in current state */
    md2_state_t prev_state; /* state before MARK (for gated-mark restore) */
    uint32_t saved_space_count;

    /* Asymmetric dot estimator (samples) */
    float dot_est;

    /* Morse code accumulator  */
    char code[MD2_MAX_CODE_LEN + 1];
    int code_len;

    /* Output queue */
    char pending; /* decoded character, '\0' = nothing */
    int space_pending; /* word-space queued */
    int gap_flushed; /* this gap already classified */
    int word_active; /* at least one real mark decoded since last
                        word-space — guards against bare spaces */
  } md2_ctx_t;

  /* ── Private: Morse lookup table ─────────────────────────────────────────── */

  typedef struct
  {
    const char* pat;
    char ch;
  } md2_entry_t;

  static const md2_entry_t md2_table[] =
  {
    /* Letters */
    { ".-", 'A' },
    { "-...", 'B' },
    { "-.-.", 'C' },
    { "-..", 'D' },
    { ".", 'E' },
    { "..-.", 'F' },
    { "--.", 'G' },
    { "....", 'H' },
    { "..", 'I' },
    { ".---", 'J' },
    { "-.-", 'K' },
    { ".-..", 'L' },
    { "--", 'M' },
    { "-.", 'N' },
    { "---", 'O' },
    { ".--.", 'P' },
    { "--.-", 'Q' },
    { ".-.", 'R' },
    { "...", 'S' },
    { "-", 'T' },
    { "..-", 'U' },
    { "...-", 'V' },
    { ".--", 'W' },
    { "-..-", 'X' },
    { "-.--", 'Y' },
    { "--..", 'Z' },
    /* Digits */
    { "-----", '0' },
    { ".----", '1' },
    { "..---", '2' },
    { "...--", '3' },
    { "....-", '4' },
    { ".....", '5' },
    { "-....", '6' },
    { "--...", '7' },
    { "---..", '8' },
    { "----.", '9' },
    /* Common punctuation */
    { ".-.-.-", '.' },
    { "--..--", ',' },
    { "..--..", '?' },
    { ".----.", '\'' },
    { "-....-", '-' },
    { "-..-.", '/' },
    { "-.--.", '(' },
    { "-.--.-", ')' },
    { NULL, '\0' }
  };

  /* ── Private: helpers ────────────────────────────────────────────────────── */

  static inline float md2_alpha(float tc)
  {
    return 1.0f - expf(-1.0f / (tc * (float)MD2_SAMPLE_RATE));
  }

  static inline char md2_lookup(const char* code)
  {
    for (int i = 0; md2_table[i].pat; i++)
      if (strcmp(md2_table[i].pat, code) == 0)
        return md2_table[i].ch;
    return '?';
  }

  static inline void md2_clamp_dot(md2_ctx_t* c)
  {
    if (c->dot_est < MD2_DOT_MIN_SAMP)
      c->dot_est = MD2_DOT_MIN_SAMP;
    if (c->dot_est > MD2_DOT_MAX_SAMP)
      c->dot_est = MD2_DOT_MAX_SAMP;
  }

  static inline void md2_emit_char(md2_ctx_t* c)
  {
    if (c->code_len == 0)
      return;
    c->code[c->code_len] = '\0';
    c->pending = md2_lookup(c->code);
    c->code_len = 0;
    memset(c->code, 0, sizeof(c->code));
  }

  /*
   * Asymmetric dot estimator update.
   * obs  : the timing observation (raw mark duration for dots, dur/3 for dashes)
   * Uses FAST_RATE when obs < dot_est (convergence / speed-up),
   * and  SLOW_RATE when obs > dot_est (noise / slow-down resistance).
   */
  static inline void md2_update_dot(md2_ctx_t* c, float obs)
  {
    float rate = (obs < c->dot_est) ? MD2_DOT_FAST_RATE : MD2_DOT_SLOW_RATE;
    c->dot_est += rate * (obs - c->dot_est);
    md2_clamp_dot(c);
  }

  static inline void md2_on_mark_end(md2_ctx_t* c, uint32_t dur)
  {
    /* Overflow: buffer full means dot_est was wrong; discard silently */
    if (c->code_len >= MD2_MAX_CODE_LEN)
    {
      c->code_len = 0;
      memset(c->code, 0, sizeof(c->code));
    }
    if ((float)dur < c->dot_est * MD2_DOT_DASH_BOUND)
    {
      md2_update_dot(c, (float)dur); /* dot observation */
      c->code[c->code_len++] = '.';
    }
    else
    {
      md2_update_dot(c, (float)dur / 3.0f); /* dash/3 ≈ one dot */
      c->code[c->code_len++] = '-';
    }
  }

  static inline void md2_classify_space(md2_ctx_t* c, uint32_t dur)
  {
    if ((float)dur < c->dot_est * MD2_ELEM_CHAR_BOUND)
    {
      /* Inter-element gap — nothing to emit */
    }
    else if ((float)dur < c->dot_est * MD2_CHAR_WORD_BOUND)
    {
      /* Inter-character gap */
      md2_emit_char(c);
    }
    else
    {
      /* Inter-word gap */
      md2_emit_char(c);
      if (c->word_active)
      {
        c->space_pending = 1;
        c->word_active = 0;
      }
    }
  }

  /* ── Public API ──────────────────────────────────────────────────────────── */

  static md2_ctx_t md2_ctx;
  static int md2_initialised = 0;

  #define MD2_MARK_BIT(r) ((r) & 0x80u)
  #define MD2_CHAR(r) ((char)((r) & 0x7Fu))

  static const uint8_t morse_decode(const float sample)
  {
    if (!md2_initialised)
    {
      memset(&md2_ctx, 0, sizeof(md2_ctx));
      md2_ctx.att = md2_alpha(MD2_ENV_ATTACK_TC);
      md2_ctx.dec = md2_alpha(MD2_ENV_DECAY_TC);
      md2_ctx.lvl_att = md2_alpha(MD2_LVL_ATTACK_TC);
      md2_ctx.lvl_dec = md2_alpha(MD2_LVL_DECAY_TC);
      md2_ctx.dot_est = MD2_INITIAL_DOT_SAMP;
      md2_ctx.state = MD2_IDLE;
      md2_initialised = 1;
    }

    /* ── 1. Envelope follower ─────────────────────────────────────────── */
    const float rect = fabsf(sample);
    const float ecoef = (rect > md2_ctx.envelope) ? md2_ctx.att : md2_ctx.dec;
    md2_ctx.envelope += ecoef * (rect - md2_ctx.envelope);

    /* ── 2. Signal level tracker (fast up, slow down) ─────────────────── */
    const float lcoef = (md2_ctx.envelope > md2_ctx.sig_level) ? md2_ctx.lvl_att
                                                        : md2_ctx.lvl_dec;
    md2_ctx.sig_level += lcoef * (md2_ctx.envelope - md2_ctx.sig_level);

    /* ── 3. Schmitt trigger ────────────────────────────────────────────── */
    /*
    * The detector only changes state when the envelope crosses its own
    * side's threshold.  If currently OFF, it needs to reach ON_FRAC of
    * sig_level to turn ON.  If currently ON, it needs to fall below
    * OFF_FRAC to turn OFF.  The gap between the two thresholds means
    * brief dips or spikes near the boundary cannot cause a flip —
    * there are no false edges to gate against.
    */
    const float on_thr = md2_ctx.sig_level * MD2_ON_FRAC;
    const float off_thr = md2_ctx.sig_level * MD2_OFF_FRAC;
    if (!md2_ctx.schmitt_on && md2_ctx.envelope >= on_thr)
      md2_ctx.schmitt_on = 1;
    else if (md2_ctx.schmitt_on && md2_ctx.envelope < off_thr)
      md2_ctx.schmitt_on = 0;

    const int on = md2_ctx.schmitt_on;

    /* ── 4. State machine (identical structure to decoder 1) ──────────── */
    switch (md2_ctx.state)
    {
      case MD2_IDLE:
        if (on)
        {
          md2_ctx.prev_state = MD2_IDLE;
          md2_ctx.state = MD2_MARK;
          md2_ctx.count = 1;
        }
        break;

      case MD2_MARK:
        if (on)
        {
          md2_ctx.count++;
        }
        else
        {
          /*
          * No separate noise gate needed here — the Schmitt trigger's
          * hysteresis prevents spurious short marks from appearing.
          * Any mark that reaches here caused a genuine threshold
          * crossing, so classify it directly.
          * The only exception is a restoration from a glitched SPACE;
          * we reuse the prev_state / saved_space_count mechanism from
          * decoder 1 for that rare case where the Schmitt trigger
          * momentarily fires mid-gap due to signal fading.
          */
          md2_on_mark_end(&md2_ctx, md2_ctx.count);
          md2_ctx.word_active = 1;
          md2_ctx.state = MD2_SPACE;
          md2_ctx.count = 1;
          md2_ctx.gap_flushed = 0;
        }
        break;

      case MD2_SPACE:
        if (!on)
        {
          md2_ctx.count++;
          /* Real-time word-gap detector — fires once (gap_flushed)   */
          if (!md2_ctx.gap_flushed && (float)md2_ctx.count >= md2_ctx.dot_est * MD2_CHAR_WORD_BOUND)
          {
            md2_emit_char(&md2_ctx);
            if (md2_ctx.word_active)
            {
              md2_ctx.space_pending = 1;
              md2_ctx.word_active = 0;
            }
            md2_ctx.gap_flushed = 1;
          }
        }
        else
        {
          /* SPACE → MARK                                              */
          md2_ctx.saved_space_count = md2_ctx.count;
          md2_ctx.prev_state = MD2_SPACE;
          if (!md2_ctx.gap_flushed)
            md2_classify_space(&md2_ctx, md2_ctx.count);
          md2_ctx.gap_flushed = 0;
          md2_ctx.state = MD2_MARK;
          md2_ctx.count = 1;
        }
        break;
    }

    /* ── 5. Build return value ─────────────────────────────────────────── */
    uint8_t result = 0;
    if (md2_ctx.pending != '\0')
    {
      result = (uint8_t)(md2_ctx.pending & 0x7Fu);
      md2_ctx.pending = '\0';
    }
    else if (md2_ctx.space_pending)
    {
      result = (uint8_t)' ';
      md2_ctx.space_pending = 0;
    }
    if (md2_ctx.state == MD2_MARK) result |= 0x80u;
    return result;
  }
}
#endif /* MORSE_DECODER2_H */
