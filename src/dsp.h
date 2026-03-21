#ifndef DSP_H
#define DSP_H

#include "filter.h"
#include "util.h"

namespace DSP
{
  volatile static float agc_peak = 0.0f;

  static const int16_t __not_in_flash_func(agc)(const float in)
  {
    // limit gain to max of 40 (32db)
    static const float max_gain = 40.0f;
    // decay about 10dB per second
    static const float k = 0.99996f;

    const float magnitude = fabsf(in);
    if (magnitude>agc_peak)
    {
      agc_peak = magnitude;
    }
    else
    {
      agc_peak *= k;
    }

    // trap issues with low values
    if (agc_peak<1.0f) return (int16_t)(in * max_gain);

    // set maximum gain possible for 12 bit DAC
    const float m = 2047.0f/agc_peak;
    return (int16_t)(in*fminf(m,max_gain));
  }

  static const uint8_t __not_in_flash_func(smeter)(const uint32_t f)
  {
    // S9 = -73dBm = 141uV PP
    // need to return 10 for S9
    const bool hiband = (f > 15000000ul);
    if (hiband)
    {
      static const float hiadjust = 4.0f;
      static const float S0_sig = 100.0f / hiadjust;
      static const float S9_sig = 300.0f / hiadjust;
      static const float S9p_sig = 8192.0f/ hiadjust;
      static const uint32_t S9_from_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
      static const uint32_t S9_from_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
      static const uint32_t S9_min = 0ul;
      static const uint32_t S9_max = 10ul;
      static const uint32_t S9p_from_min = (uint32_t)(log10f(S9_sig) * 1024.0f);
      static const uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig) * 1024.0f);
      static const uint32_t S9p_min = 11ul;
      static const uint32_t S9p_max = 15ul;
      if (agc_peak<1.0f)
      {
        return 0u;
      }
      const uint32_t log_peak = (uint32_t)(log10f(agc_peak) * 1024.0f);
      if (agc_peak>S9_sig)
      {
        return (uint8_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
      }
      return (uint8_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
    }
    static const float S0_sig = 100.0f;
    static const float S9_sig = 300.0f;
    static const float S9p_sig = 8192.0f;
    static const uint32_t S9_from_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
    static const uint32_t S9_from_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static const uint32_t S9_min = 0ul;
    static const uint32_t S9_max = 10ul;
    static const uint32_t S9p_from_min = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static const uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig) * 1024.0f);
    static const uint32_t S9p_min = 11ul;
    static const uint32_t S9p_max = 15ul;
    if (agc_peak<1.0f)
    {
      return 0u;
    }
    const uint32_t log_peak = (uint32_t)(log10f(agc_peak) * 1024.0f);
    if (agc_peak>S9_sig)
    {
      return (uint8_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
    }
    return (uint8_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
  }

  static void __not_in_flash_func(noise_blanker)(float &I, float &Q, const uint8_t level)
  {
    static const int MAX_BLANK_RUN = 32; // ~1ms at 31250 Hz
    static const float alpha = 0.002f;   // ~16ms at 31250 Hz

    // threasholds, number of times greater than
    // average to trigger a blanking event
    static const float thresholds[] =
    {
      0.0f,   // level 0 — unused (returns early)
      20.0f,  // level 1 — conservative, only very large spikes
      10.0f,  // level 2 — moderate
      6.0f,   // level 3 — fairly aggressive
      3.0f    // level 4 — maximum blanking
    };
    if (level == 0) return;
    if (level > 4) return;
    const float threshold = thresholds[level];

    static struct
    {
      float lastI;
      float lastQ;
      float avg_amp;
      uint32_t blank_run;
      uint32_t initialized;
    } s = {0};

    if (s.initialized==0)
    {
      s.lastI = I;
      s.lastQ = Q;
      s.avg_amp = 0.01f;
      s.blank_run = 0;
      s.initialized = 1;
    }

    // instantaneous amplitude (fast estimate)
    const float amp = fabsf(I) + fabsf(Q);
    const bool blanked = (amp > threshold * s.avg_amp) && (s.blank_run < MAX_BLANK_RUN);
    if (blanked)
    {
      s.blank_run++;

      // hold last good sample
      I = s.lastI;
      Q = s.lastQ;
    }
    else
    {
      // track signal envelope
      s.avg_amp += alpha * (amp - s.avg_amp);
      s.blank_run = 0;
      
      // save last good sample
      s.lastI = I;
      s.lastQ = Q;
    }
  } 

  static const int16_t __not_in_flash_func(process_ssb)(
      const int16_t in_i,
      const int16_t in_q,
      const uint32_t jnr_level,
      const uint8_t bw,
      const uint8_t nb_level)
  {
    // quadrature mixer

    // quadrature LO = 3906 (quadrature: 31250/8/4 = 2)
    // quadrature LO = 1953 (quadrature: 31250/16/4 = 4)
    // quadrature LO = 977 (quadrature: 31250/32/4 = 8)
    // quadrature LO = 488 (quadrature: 31250/64/4 = 16)
    static const uint8_t PI2 = 16u;
    volatile static uint8_t lo = 0;
    const uint8_t I_index = lo;
    const uint8_t Q_index = lo + PI2;
    const float loI = UTIL::s64[I_index];
    const float loQ = UTIL::s64[Q_index];
    lo++;

    // remove DC
    float ii = FILTER::dc1f((float)in_i / 32768.0f);
    float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // noise blanker
    noise_blanker(ii,qq,nb_level);

    // quadrature down-convert
    const float a = ii * loQ;
    const float b = qq * loI;
    const float c = qq * loQ;
    const float d = ii * loI;
    const float mxI = a - b;
    const float mxQ = c + d;

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(mxI);
    const float n45 = FILTER::fap2f(mxQ);

    // reject image
    const float ssb = p45 - n45;

    // LPF
    const float audio_lpf = FILTER::bwf[bw](ssb);

    // HPF
    const float audio_raw = FILTER::hpf_200f(audio_lpf);

    // JNR
    const float audio_out = FILTER::jnr(audio_raw,jnr_level);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_cw)(const int16_t in_i,const int16_t in_q)
  {
    // remove DC
    const float ii = FILTER::dc1f((float)in_i / 32768.0f);
    const float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(ii);
    const float n45 = FILTER::fap2f(qq);

    // reject image
    const float ssb = p45 - n45;

    // BPF for CW
    const float audio_out = FILTER::bpf_700f(ssb);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_am)(const int16_t in_i,const int16_t in_q,const uint32_t jnr_level)
  {
    // BPF I with +45 phase shift
    // BPF Q with -45 phase shift
    // SUM (USB)
    // ABS
    // AGC
    // LPF
    // Remove DC

    // AGC settings
    static const float max_gain = 40.0f;
    static const float k = 0.99996f;

    // extract AM signal from USB @ FS/4
    const float ii = FILTER::bpf_45p((float)in_i / 32768.0f);
    const float qq = FILTER::bpf_45n((float)in_q / 32768.0f);
    const float ssb = ii + qq;
    const float magnitude = fabsf(ssb);
    const float rectified = ssb * ssb;

    // AGC
    const float agc_magnitude = magnitude * 32768.0f;
    if (agc_magnitude > agc_peak)
    {
      agc_peak = agc_magnitude;
    }
    else
    {
      agc_peak *= k;
    }

    // set maximum gain possible for 12 bit DAC
    float gain = max_gain;
    if (agc_peak > 1.0f)
    {
      gain = 2047.0f / agc_peak;
    }
    gain = fminf(gain, max_gain);

    // extract audio from rectified AM signal
    const float audio_raw = FILTER::dcf(sqrtf(fabsf(FILTER::lpf_3000f(rectified))));
    const float audio_out = FILTER::jnr(audio_raw,jnr_level);
    return (int16_t)(audio_out * 32768.0f * gain);
  }

  static const uint32_t __not_in_flash_func(get_mic_peak_level)(const int16_t mic_in)
  {
    static const uint32_t MIC_LEVEL_DECAY_RATE = 50ul;
    static const uint32_t MIC_LEVEL_HANG_TIME = 500ul;
    static uint32_t mic_peak_level = 0;
    static uint32_t mic_level_update = 0;
    static uint32_t mic_hangtime_update = 0;
    const uint32_t now = millis();
    const uint32_t mic_level = abs(mic_in)>>5;
    if (mic_level>mic_peak_level)
    {
      mic_peak_level = mic_level;
      mic_level_update = now + MIC_LEVEL_DECAY_RATE;
      mic_hangtime_update = now + MIC_LEVEL_HANG_TIME;
    }
    else
    {
      if (now>mic_hangtime_update)
      {
        if (now>mic_level_update)
        {
          if (mic_peak_level) mic_peak_level--;
          mic_level_update = now + MIC_LEVEL_DECAY_RATE;
        }
      }
    }
    return mic_peak_level;
  }

  static void __not_in_flash_func(cessb)(float& ii, float& qq)
  {
    const float mag_raw = sqrtf(ii*ii + qq*qq);
    const float mag_max = fmaxf(mag_raw, 1.0f);
    ii = FILTER::lpf_2600if_tx(ii / mag_max);
    qq = FILTER::lpf_2600qf_tx(qq / mag_max);
  }

  static const void __not_in_flash_func(process_mic)(const int16_t s,int16_t &out_i,int16_t &out_q,const float mic_gain,const bool cessb_on)
  {
    // input is 12 bits
    // convert to float
    // remove Mic DC
    // 2400 LPF 
    // phase shift I
    // phase shift Q
    // convert to int
    // output is 10 bits
    const float ac_sig = FILTER::dcf(((float)s)*(1.0f/2048.0f));
    const float mic_sig = FILTER::lpf_2600f_tx(ac_sig);
    const float ii1 = FILTER::fap1f(mic_sig);
    const float qq1 = FILTER::fap2f(mic_sig);
    float ii2 = ii1 * mic_gain;
    float qq2 = qq1 * mic_gain;
    if (cessb_on) cessb(ii2,qq2);
    out_i = (int16_t)(ii2 * 512.0f);
    out_q = (int16_t)(qq2 * 512.0f);
  }
}

#endif