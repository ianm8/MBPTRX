/*
 * cwdecode.h
 *
 * INTERFACE
 * ---------
 *   uint8_t morse_decode(float sample);
 *
 *   Call once per sample at 31 250 Hz with the output of a bandpass filter
 *   centred on the CW tone (700 Hz).  Amplitude units are arbitrary.
 *
 *   Return value — always a uint8_t with two independent fields:
 *
 *     Bit 7 (0x80)  MARK flag – set whenever the signal is currently ON.
 *                   Always reflects the live mark/space condition and may
 *                   be used to visualise the detected Morse signal in a UI.
 *
 *     Bits 6..0     Decoded character (7-bit ASCII), valid for exactly one
 *                   call, then zero until the next event:
 *                     0x01..0x7F  – a character was just decoded
 *                       ' ' (0x20) means an inter-word gap was detected
 *                     0x00        – nothing new to report
 *
 *   To extract both fields:
 *     uint8_t  raw  = morse_decode(sample);
 *     int      mark = raw & 0x80;          // non-zero while tone is ON
 *     char     ch   = raw & 0x7F;          // '\0' if no new character
 *
 *   State is held in a file-scope static struct (md_ctx) and is initialised
 *   automatically on the very first call.
 *
 * ALGORITHM
 * ---------
 *   1. Envelope follower  – rectify + asymmetric IIR (fast attack, slow
 *                           decay) to track the carrier amplitude.
 *   2. Adaptive threshold – long-term exponential average of the envelope,
 *                           updated only while a mark (tone-on) is active so
 *                           that long silences don't drag the threshold to
 *                           zero.  Threshold = MD_AVG_RATIO × average.
 *   3. State machine      – IDLE → MARK (tone on) ↔ SPACE (tone off), with
 *                           a sample counter for the current state.
 *   4. Mark classifier    – on MARK→SPACE: dot if duration < 2 × dot_est,
 *                           dash otherwise.  dot_est is refined by
 *                           exponential smoothing after every mark.
 *   5. Space classifier   – on SPACE→MARK and mid-SPACE (word gap):
 *                           inter-element (<2×), inter-character (2–5×),
 *                           inter-word (≥5×).
 *   6. Morse→ASCII lookup – linear search through a dot/dash string table.
 */

#ifndef MORSE_DECODER_H
#define MORSE_DECODER_H

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace CWDECODE
{

  /* ── Build-time configuration ────────────────────────────────────────────── */

  #define MD_SAMPLE_RATE 31250 /* Hz – must match ADC / filter chain    */
  #define MD_MAX_CODE_LEN 8    /* longest supported Morse pattern       */

  /*
   * Envelope follower time constants (seconds).
   * Fast attack catches mark onset; slow decay bridges 700 Hz carrier cycles
   * (each cycle ≈ 1.4 ms) so the envelope stays smooth between them. 
   */
  #define MD_ENV_ATTACK_TC 0.001f /* 1 ms                                  */
  #define MD_ENV_DECAY_TC 0.008f  /* 8 ms                                  */

  /*
   * Long-term amplitude average time constants.
   *
   * MD_AVG_TC       – fast IIR used during MARK to track the signal level.
   * MD_AVG_DECAY_TC – very slow IIR used during SPACE/IDLE.  Lets avg_amp
   *                   drift back down if inflated by a noise spike, preventing
   *                   permanent threshold lock-up.  At 5 s a typical inter-
   *                   word gap (~1 s) causes only ~18 % droop, which is
   *                   harmless; recovery from a spike takes ~15 s worst-case.
   */
  #define MD_AVG_TC 0.800f       /* 800 ms – mark-time tracking           */
  #define MD_AVG_DECAY_TC 5.000f /*   5 s  – silence-time decay           */
  #define MD_AVG_RATIO 0.50f     /* threshold = MD_AVG_RATIO × avg_amp    */

  /*
   * Duration classification boundaries (multiples of dot_est).
   * Standard Morse ratios: dot=1, dash=3, elem-gap=1, char-gap=3, word-gap=7
   * Boundaries are set wider than the exact midpoints to give headroom for
   * the timing variations typical of hand-sent CW:
   *   dot/dash  : midpoint is 2×; kept at 2× (marks are less ambiguous)
   *   elem/char : midpoint is 2×; widened to 2.5× – a char-gap must be at
   *               least 2.5× dot before it triggers character emission, so
   *               slightly compressed inter-element gaps aren't misread.
   *   char/word : midpoint is 5×; widened to 6× – reduces false word-spaces
   *               from slow senders whose char-gaps stretch toward word-gap.
   */
  #define MD_DOT_DASH_BOUND 2.0f  /* < 2.0× → dot,  ≥ 2.0× → dash         */
  #define MD_ELEM_CHAR_BOUND 2.5f /* < 2.5× → elem, ≥ 2.5× → char-gap     */
  #define MD_CHAR_WORD_BOUND 6.0f /* < 6.0× → char, ≥ 6.0× → word-gap     */

  /*
   * Minimum mark duration as a fraction of dot_est.
   * Marks shorter than this are treated as noise glitches: discarded without
   * updating dot_est or appending to the code buffer.
   */
  #define MD_MIN_MARK_FRAC 0.3f

  /* Dot-length learning rates.                                         */
  #define MD_DOT_LEARN 0.25f  /* weight of each new dot observation     */
  #define MD_DASH_LEARN 0.15f /* weight of each dash/3 observation      */

  /*
   * Seed dot duration: 60 ms ≈ 20 WPM — midpoint of the clamped speed range.
   * Seeding here rather than at 15 WPM halves the convergence distance for
   * signals at 25 WPM, allowing the mark learner alone to converge within
   * 3-4 marks without needing a space-gap learner.
   */
  #define MD_INITIAL_DOT_SAMP (0.060f * (float)MD_SAMPLE_RATE)

  /*
   * Hard clamps on dot_est.  These bound the effect of any mis-learning event
   * (noise bursts, ionosonde impulses, misclassified gaps) so the decoder can
   * never drift into a state where it cannot recover.
   *   MD_DOT_MIN_SAMP : 35 ms ≈ 34 WPM — fast enough for any likely signal
   *   MD_DOT_MAX_SAMP : 150 ms ≈ 8 WPM — slow enough for any likely signal
   * Adjust to taste; the seed must lie within this range.
   */
  #define MD_DOT_MIN_SAMP (0.035f * (float)MD_SAMPLE_RATE)
  #define MD_DOT_MAX_SAMP (0.150f * (float)MD_SAMPLE_RATE)

  /* ── Private: types ──────────────────────────────────────────────────────── */

  typedef enum
  {
    MD_IDLE,
    MD_MARK,
    MD_SPACE
  } md_state_t;

  typedef struct
  {
    /* Envelope follower */
    float att; /* IIR coefficient, attack */
    float dec; /* IIR coefficient, decay  */
    float envelope;

    /* Adaptive threshold */
    float avg_coef;       /* IIR coef during MARK  (fast)           */
    float avg_decay_coef; /* IIR coef during SPACE/IDLE (very slow) */
    float avg_amp;

    /* State machine */
    md_state_t state;
    uint32_t count;             /* samples spent in current state */
    md_state_t prev_state;      /* state before entering MARK (IDLE or SPACE) */
    uint32_t saved_space_count; /* space count saved on SPACE→MARK, used
                                          to restore the gap if the mark is gated */

    /* Dot-length estimator (samples) */
    float dot_est;

    /* Morse code accumulator for the character in progress */
    char code[MD_MAX_CODE_LEN + 1];
    int code_len;

    /* One pending output character; '\0' means nothing to report */
    char pending;

    /*
     * Set when a word-gap has been detected and a ' ' still needs to be
     * returned.  Separate from pending so the preceding character and the
     * word-space are both emitted, one per call, in order.
     */
    int space_pending;

    /*
     * Set when the current SPACE gap has already been classified (either
     * by the mid-SPACE word-gap detector or by md_classify_space at
     * SPACE→MARK).  Prevents the two paths from both acting on the same
     * gap, which was the root cause of the missing-last-char bug.
     * Cleared every time a new SPACE is entered from MARK.
     */
    int gap_flushed;

    /*
     * Set when at least one real mark (post-gate) has been processed since
     * the last word-space or reset.  Guards against emitting word-spaces
     * into an empty code buffer (e.g. during noise-only input).
     */
    int word_active;
  } md_ctx_t;

  /* ── Private: Morse lookup table ─────────────────────────────────────────── */

  typedef struct
  {
    const char *pat;
    char ch;
  }
  md_entry_t;

  static const md_entry_t md_table[] =
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

  /* ── Private: helper functions ───────────────────────────────────────────── */

  /* First-order IIR coefficient for a given time constant at MD_SAMPLE_RATE */
  static inline float md_alpha(float tc)
  {
    return 1.0f - expf(-1.0f / (tc * (float)MD_SAMPLE_RATE));
  }

  /* Dot/dash string → ASCII character; returns '?' on no match */
  static inline char md_lookup(const char *code)
  {
    for (int i = 0; md_table[i].pat; i++)
      if (strcmp(md_table[i].pat, code) == 0)
        return md_table[i].ch;
    return '?';
  }

  /* Flush the accumulated code string to pending and reset the accumulator */
  static inline void md_emit_char(md_ctx_t *c)
  {
    if (c->code_len == 0)
      return;
    c->code[c->code_len] = '\0';
    c->pending = md_lookup(c->code);
    c->code_len = 0;
    memset(c->code, 0, sizeof(c->code));
  }

  /* Clamp dot_est to the allowed speed range after every update */
  static inline void md_clamp_dot(md_ctx_t *c)
  {
    if (c->dot_est < MD_DOT_MIN_SAMP) c->dot_est = MD_DOT_MIN_SAMP;
    if (c->dot_est > MD_DOT_MAX_SAMP) c->dot_est = MD_DOT_MAX_SAMP;
  }

  /* MARK→SPACE: classify the mark and update dot_est */
  static inline void md_on_mark_end(md_ctx_t *c, uint32_t dur)
  {
    /*
     * Safety flush: if the code buffer is full, dot_est was badly wrong and
     * every char-gap has been swallowed as inter-element, or noise has filled
     * the buffer.  Silently discard the garbage rather than emitting '?' —
     * a legitimate '?' (..--.. ) is always emitted via md_classify_space on
     * a real char-gap, never via this path.
     */
    if (c->code_len >= MD_MAX_CODE_LEN)
    {
      c->code_len = 0;
      memset(c->code, 0, sizeof(c->code));
    }

    if ((float)dur < c->dot_est * MD_DOT_DASH_BOUND)
    {
      /* Dot */
      c->dot_est = (1.0f - MD_DOT_LEARN) * c->dot_est + MD_DOT_LEARN * (float)dur;
      md_clamp_dot(c);
      if (c->code_len < MD_MAX_CODE_LEN)
        c->code[c->code_len++] = '.';
    }
    else
    {
      /* Dash: dash/3 approximates one dot */
      c->dot_est = (1.0f - MD_DASH_LEARN) * c->dot_est + MD_DASH_LEARN * ((float)dur / 3.0f);
      md_clamp_dot(c);
      if (c->code_len < MD_MAX_CODE_LEN)
        c->code[c->code_len++] = '-';
    }
  }

  /*
   * Classify a completed gap and emit accordingly.
   * space_pending is only set when word_active is true, preventing word-spaces
   * from being emitted into an empty code buffer (e.g. during noise).
   */
  static inline void md_classify_space(md_ctx_t *c, uint32_t dur)
  {
    if ((float)dur < c->dot_est * MD_ELEM_CHAR_BOUND)
    {
      /* Inter-element gap: nothing to do.*/
    }
    else if ((float)dur < c->dot_est * MD_CHAR_WORD_BOUND)
    {
      /* Inter-character gap */
      md_emit_char(c);
    }
    else
    {
      /*
       * Inter-word gap: emit the final character then queue the space.
       * Both will be returned by morse_decode(), one per call.
       * The space is only queued if word_active — i.e. at least one
       * real mark has been decoded since the last word-space, so we
       * don't emit a bare space into an empty output stream.
       */
      md_emit_char(c);
      if (c->word_active)
      {
        c->space_pending = 1;
        c->word_active = 0;
      }
    }
  }

  /* ── Public API ──────────────────────────────────────────────────────────── */

  /*
   * Global decoder context and initialisation flag.
   * Declared at file scope so that any translation unit including this header
   * shares a single instance.  Not thread-safe; for concurrent use, replace
   * with an explicit context parameter.
  */
  static md_ctx_t md_ctx;
  static int md_initialised = 0;

  /*
   * morse_decode()
   *
   * Feed one float PCM sample (bandpass-filtered, 700 Hz centre, 31 250 Hz
   * sample rate).
   *
   * Returns a uint8_t with two independent fields:
   *
   *   Bit 7 (0x80)  MARK flag – set while the detected tone is ON.
   *                 Always reflects the live signal state; use it to
   *                 visualise the Morse keying in a UI.
   *
   *   Bits 6..0     7-bit ASCII result, non-zero for exactly one call per
   *                 decoded event, then 0x00 until the next event.
   *                 0x20 (' ') signals an inter-word gap.
   *
   * Convenience macros:
   *   MD_MARK_BIT(r)  – non-zero while tone is ON
   *   MD_CHAR(r)      – decoded character, or '\0'
   */
  #define MD_MARK_BIT(r) ((r)&0x80u)
  #define MD_CHAR(r) ((char)((r)&0x7Fu))

  static uint8_t morse_decode(float sample)
  {
    /* ── Initialise on first call ──────────────────────────────────────── */
    if (!md_initialised)
    {
      memset(&md_ctx, 0, sizeof(md_ctx));
      md_ctx.att = md_alpha(MD_ENV_ATTACK_TC);
      md_ctx.dec = md_alpha(MD_ENV_DECAY_TC);
      md_ctx.avg_coef = md_alpha(MD_AVG_TC);
      md_ctx.avg_decay_coef = md_alpha(MD_AVG_DECAY_TC);
      md_ctx.dot_est = MD_INITIAL_DOT_SAMP;
      md_ctx.state = MD_IDLE;
      md_initialised = 1;
    }

    /* ── 1. Envelope follower ──────────────────────────────────────────── */
    float rect = fabsf(sample);
    float coef = (rect > md_ctx.envelope) ? md_ctx.att : md_ctx.dec;
    md_ctx.envelope += coef * (rect - md_ctx.envelope);

    /* ── 2. Adaptive threshold ─────────────────────────────────────────── */
    /*
     * avg_amp tracks the typical signal level when the tone is present.
     *
     * During MARK : fast IIR (MD_AVG_TC = 800 ms) locks onto the carrier
     *               level accurately.
     * During SPACE/IDLE : very slow IIR (MD_AVG_DECAY_TC = 5 s) lets
     *               avg_amp drift back down toward zero.  This has two
     *               benefits:
     *                 1. A noise spike that inflated avg_amp while idle
     *                    cannot cause permanent threshold lock-up; the
     *                    system self-recovers within ~15 s.
     *                 2. If the signal genuinely disappears the threshold
     *                    eventually falls to noise floor rather than
     *                    staying latched at the old signal level forever.
     *               The decay is slow enough that normal inter-word gaps
     *               (~1 s) cause negligible droop (~18 %).
     */
    if (md_ctx.state == MD_MARK)
      md_ctx.avg_amp += md_ctx.avg_coef * (md_ctx.envelope - md_ctx.avg_amp);
    else
      md_ctx.avg_amp += md_ctx.avg_decay_coef * (md_ctx.envelope - md_ctx.avg_amp);

    float threshold = md_ctx.avg_amp * MD_AVG_RATIO;
    if (threshold < 1e-9f) threshold = 1e-9f;

    const int on = (md_ctx.envelope >= threshold);

    /* ── 3. State machine ──────────────────────────────────────────────── */
    switch (md_ctx.state)
    {
      case MD_IDLE:
        if (on)
        {
          md_ctx.prev_state = MD_IDLE;
          md_ctx.state = MD_MARK;
          md_ctx.count = 1;
        }
        break;

      case MD_MARK:
        if (on)
        {
          md_ctx.count++;
        }
        else
        {
          if ((float)md_ctx.count < md_ctx.dot_est * MD_MIN_MARK_FRAC)
          {
            /*
             * Noise gate: mark too short to be a real element.
             * From SPACE: restore the space state so the gap
             * measurement continues as if the glitch never happened.
             * From IDLE: discard the burst and clear any partial state.
             */
            if (md_ctx.prev_state == MD_SPACE)
            {
              md_ctx.state = MD_SPACE;
              md_ctx.count = md_ctx.saved_space_count + md_ctx.count;
            }
            else
            {
              md_ctx.state = MD_IDLE;
              md_ctx.count = 0;
              md_ctx.code_len = 0;
              md_ctx.gap_flushed = 0;
              md_ctx.word_active = 0;
              memset(md_ctx.code, 0, sizeof(md_ctx.code));
            }
          }
          else
          {
            /* 
             * Real mark: classify and enter SPACE.
             * gap_flushed is cleared here so the new SPACE gap starts
             * fresh — neither the mid-SPACE detector nor SPACE→MARK
             * will see a stale flushed flag from the previous gap.
             */
            md_on_mark_end(&md_ctx, md_ctx.count);
            md_ctx.word_active = 1;
            md_ctx.state = MD_SPACE;
            md_ctx.count = 1;
            md_ctx.gap_flushed = 0;
          }
        }
        break;

      case MD_SPACE:
        if (!on)
        {
          md_ctx.count++;
          /*
           * Mid-SPACE word-gap detector.
           *
           * Fires exactly once per gap (gap_flushed prevents re-entry)
           * when the silence has lasted longer than a word gap, so the
           * last character of a word and its following space are emitted
           * promptly without waiting for the next mark to arrive.
           *
           * gap_flushed is set here so that the SPACE→MARK path later
           * skips md_classify_space — the gap is already handled.
           */
          if (!md_ctx.gap_flushed && (float)md_ctx.count >= md_ctx.dot_est * MD_CHAR_WORD_BOUND)
          {
            md_emit_char(&md_ctx); /* emit last char if any  */
            if (md_ctx.word_active)
            {
              md_ctx.space_pending = 1;
              md_ctx.word_active = 0;
            }
            md_ctx.gap_flushed = 1; /* one shot only */
          }
        }
        else
        {
          /*
           * SPACE → MARK.
           * Save the space count for glitch restoration, then classify
           * the gap only if the mid-SPACE detector hasn't done it yet.
           */
          md_ctx.saved_space_count = md_ctx.count;
          md_ctx.prev_state = MD_SPACE;
          if (!md_ctx.gap_flushed)
            md_classify_space(&md_ctx, md_ctx.count);
          md_ctx.gap_flushed = 0; /* reset for the gap after this mark */
          md_ctx.state = MD_MARK;
          md_ctx.count = 1;
        }
        break;
    }

    /* ── 4. Build return value ─────────────────────────────────────────── */
    /*
     * Priority: character > word-space > nothing.
     * If both pending and space_pending are set (word-gap immediately
     * follows the last character of a word), the character is returned
     * first; space_pending is left set and the ' ' will be returned on
     * the very next call that would otherwise return 0x00.
     */
    uint8_t result = 0u;
    if (md_ctx.pending != '\0')
    {
      result = (uint8_t)(md_ctx.pending & 0x7Fu);
      md_ctx.pending = '\0';
    }
    else if (md_ctx.space_pending)
    {
      result = (uint8_t)' ';
      md_ctx.space_pending = 0;
    }
    if (md_ctx.state == MD_MARK) result |= 0x80u;
    return result;
  }
}

#endif /* MORSE_DECODER_H */
