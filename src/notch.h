#ifndef NOTCH_H
#define NOTCH_H

// ---------------------------------------------------------------------------
//  Tunable audio notch for uDST / MBPTRX                              VK7IAN
// ---------------------------------------------------------------------------
//
//  Twin-zero ("flat bottom") notch.  Two 2nd order notch sections are placed
//  either side of the tuned frequency instead of both on top of it.  The split
//  is chosen so the hump between the two nulls sits at exactly the requested
//  depth, which turns the usual infinitely deep pinhole into a trough wide
//  enough to actually tune onto.
//
//    set(f0, bw, depth)
//        f0     tuned centre frequency, Hz  (400 .. 2800)
//        bw     -3 dB width of each section, Hz  (100 is a good default)
//        depth  stopband floor, dB  (25 .. 35 sensible)
//
//  Rejection width scales as   width(A) = bw * sqrt(A)     A = 10^(-depth/20)
//  so bw = 100 Hz, depth = 30 dB gives roughly:
//        >= 30 dB over about 26 Hz
//        >= 20 dB over about 39 Hz
//        -3 dB points at about +/- 79 Hz
//
//  Cost on RP2350 @ 240 MHz: about 60 clock cycles per sample (~0.8% of the
//  7680 cycle frame at 31250 Hz).  Coefficient design is done off the audio
//  path, only when the tuning changes.
//
//  NOTE: everything is float.  Do not let a double sneak in - double is
//  software emulated on the M33 and will cost you hundreds of cycles.
//
//  Placement: after demodulation and the main SSB/AM filter, BEFORE the AGC
//  and before any noise reduction.  Notching after the AGC leaves the
//  heterodyne pumping the gain and desensing the receiver.
// ---------------------------------------------------------------------------

#include <math.h>
#include <stdint.h>

class notch_t
{
  private:

    struct coef_t
    {
      float g;  // passband normalisation, also the b0 / b2 coefficient
      float c2; // 2 * cos(w0)
      float a1; // -2 * r * cos(w0)
      float a2; // r * r
    };

    struct state_t
    {
      float x1, x2, y1, y2;
    };

    float fs;
    bool enabled;
    volatile bool pending; // set by the UI core, cleared by the DSP core
    coef_t live[2];
    coef_t shadow[2];
    state_t st[2];

    static void design(coef_t &c, const float fs, const float f0, const float bw)
    {
      const float pi = 3.14159265358979f;
      const float w0 = 2.0f * pi * f0 / fs;
      const float cw = cosf(w0);

      // pole radius sets the -3 dB width
      float r = 1.0f - pi * bw / fs;
      if (r < 0.90f) r = 0.90f;
      if (r > 0.9999f) r = 0.9999f;

      // unity in the passband either side
      c.g  = 0.5f * (1.0f + r);
      c.c2 = 2.0f * cw;
      c.a1 = -2.0f * r * cw;
      c.a2 = r * r;
    }

  public:

    void init(const float sample_rate)
    {
      fs = sample_rate;
      enabled = false;
      pending = false;
      reset();
      set(1000.0f, 100.0f, 30.0f);
      live[0] = shadow[0];
      live[1] = shadow[1];
      pending = false;
    }

    void reset(void)
    {
      for (uint32_t i = 0u; i < 2u; i++)
      {
        st[i].x1 = st[i].x2 = st[i].y1 = st[i].y2 = 0.0f;
      }
    }

    void enable(const bool on)
    {
      if (on && !enabled) reset();
      enabled = on;
    }

    bool is_enabled(void) const
    {
      return enabled;
    }

    // Safe to call from the UI core while the DSP core is running.
    void set(float f0, float bw, float depth_db)
    {
      if (bw < 20.0f) bw = 20.0f;
      if (bw > 500.0f) bw = 500.0f;
      if (depth_db < 10.0f) depth_db = 10.0f;
      if (depth_db > 45.0f) depth_db = 45.0f;

      const float a = powf(10.0f, -depth_db / 20.0f); // linear stopband floor
      const float d = 0.5f * bw * sqrtf(a);  // half separation of the nulls

      const float lo = 0.5f * bw + d + 10.0f;
      const float hi = 0.5f * fs - (0.5f * bw + d + 10.0f);
      if (f0 < lo) f0 = lo;
      if (f0 > hi) f0 = hi;

      design(shadow[0], fs, f0 - d, bw);
      design(shadow[1], fs, f0 + d, bw);
      pending = true;
    }

    // One audio sample in, one out.  Inline this into the audio loop.
    inline float __not_in_flash_func(process)(const float x)
    {
      if (pending)
      {
        live[0] = shadow[0];
        live[1] = shadow[1];
        pending = false;
      }
      if (!enabled)
      {
        return x;
      }
      float v = x;
      for (uint32_t i = 0u; i < 2u; i++)
      {
        const coef_t &c = live[i];
        state_t &s = st[i];
        // b0 == b2 == g, b1 == -2*g*cos(w0), so factor g out of the numerator
        const float t = v + s.x2 - c.c2 * s.x1;
        const float y = c.g * t - c.a1 * s.y1 - c.a2 * s.y2;
        s.x2 = s.x1;
        s.x1 = v;
        s.y2 = s.y1;
        s.y1 = y;
        v = y;
      }
      return v;
    }

    // Convenience wrapper for an integer audio path.
    inline int32_t process(const int32_t x)
    {
      return (int32_t)lrintf(process((float)x));
    }
};

#endif
