#ifndef DSP_H
#define DSP_H

#include "filter.h"
#include "notch.h"
#include "util.h"

namespace DSP
{
  static notch_t notch;
  volatile static float agc_peak = 0.0f;

  static const int16_t __not_in_flash_func(agc)(const float in)
  {
    // limit gain to max of 100 (40db)
    static const float max_gain = 100.0f;
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
    static constexpr float GAIN_3dB = 1.5f;
    static constexpr float GAIN_6dB = 2.0f;
    static constexpr float GAIN_9dB = 3.0f;
    static constexpr float GAIN_12dB = 4.0f;
    static constexpr float GAIN_15dB = 6.0f;
    static constexpr float GAIN_18dB = 8.0f;
    float sensitivity = 1.0f;
    if (f > 22000000ul)
    {
      sensitivity = GAIN_18dB;
    }
    else if (f > 18000000ul)
    {
      sensitivity = GAIN_12dB;
    }
    else if (f > 10000000ul && f < 11000000ul)
    {
      sensitivity = GAIN_6dB;
    }
    // add compensation gain to agc_peak
    const float comp_agc_peak = agc_peak * sensitivity;
    if (comp_agc_peak<1.0f)
    {
      return 0u;
    }
    static constexpr float S0_sig = 100.0f;
    static constexpr float S9_sig = 300.0f;
    static constexpr float S9p_sig = 8192.0f;
    static constexpr uint32_t S9_from_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
    static constexpr uint32_t S9_from_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static constexpr uint32_t S9_min = 0ul;
    static constexpr uint32_t S9_max = 10ul;
    static constexpr uint32_t S9p_from_min = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static constexpr uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig) * 1024.0f);
    static constexpr uint32_t S9p_min = 11ul;
    static constexpr uint32_t S9p_max = 15ul;
    const uint32_t log_peak = (uint32_t)(log10f(comp_agc_peak) * 1024.0f);
    if (comp_agc_peak>S9_sig)
    {
      return (uint8_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
    }
    return (uint8_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
  }

  static void __not_in_flash_func(noise_blanker)(float &I, float &Q, const uint8_t level)
  {
    // Impulse ("pop") blanker working on the 31.25 kHz I/Q, before any
    // narrow filtering.
    //
    // Small events are blanked exactly as the original blanker did:
    // GUARD_PRE samples before + the samples over the threshold. Most
    // triggers at levels 3-5 are noise peaks and voice transients, so this
    // keeps the damage to signals the same as before.
    //
    // Big pops get more. RING_DB_PER_SAMPLE is not calibrated.
    // If big pops still click at levels 3–5, lower RING_DB_PER_SAMPLE or
    // raise GUARD_EXTRA_MAX. If the noise blanker audibly chews up
    // signals on big pops, try asymmetric guards:
    //   keep only GUARD_PRE before the pop and apply extra after it.
    //
    // Blanking only the samples over the threshold leaves that ringing, which
    // still clicks. So a pop more than BIG_POP_DB over the threshold also
    // gets 'extra' samples blanked on each side, one per 2.2 dB it rose above
    // that, enough for the ringing to decay back to the threshold:
    //
    //   GUARD_PRE + extra  |  pop  |  extra
    //
    // States:
    //   NB_IDLE    watching; average tracks the signal
    //   NB_EVENT   amplitude above threshold, peak recorded
    //   NB_POST    big pop over; waiting 'extra' samples for its trailing
    //              ringing (a new pop inside this window is merged)
    //   NB_SIGNAL  ran longer than MAX_BLANK_RUN: a real signal (voice peak,
    //              nearby station), passed untouched while the average adapts
    //
    // Delay: NB_LEN samples (64 / 31250 = ~2 ms) while enabled, none when off.

    static constexpr uint32_t NB_LEN = 64u;          // delay line, power of two
    static constexpr uint32_t NB_MASK = NB_LEN - 1u;
    static constexpr uint32_t MAX_BLANK_RUN = 32u;   // ~1 ms: longer = signal, not a pop
    static constexpr uint32_t GUARD_PRE = 2u;        // always blanked before an event (as before)
    static constexpr float BIG_POP_DB = 6.0f;        // pops this far over the threshold get extra guard
    static constexpr uint32_t GUARD_EXTRA_MAX = 12u; // extra samples each side, biggest pops (~380 us)
    static constexpr float RING_DB_PER_SAMPLE = 2.2f;// DDC FIR ringing decay (measured)
    static constexpr float alpha = 0.002f;           // average, ~16 ms at 31250 Hz
    static_assert(GUARD_PRE + 2u*GUARD_EXTRA_MAX + MAX_BLANK_RUN <= NB_LEN - 2u, "blank window must fit in the delay line");

    // thresholds, number of times greater than
    // average to trigger a blanking event
    static const float thresholds[] =
    {
      0.0f, // level 0 — unused (returns early)
      9.0f, // level 1 — only very large spikes
      7.0f, // level 2 — conservative
      5.0f, // level 3 — moderate
      3.0f, // level 4 — fairly aggressive
      2.0f  // level 5 — maximum blanking
    };

    enum nb_state_t { NB_IDLE, NB_EVENT, NB_POST, NB_SIGNAL };
    static struct
    {
      float bufI[NB_LEN];
      float bufQ[NB_LEN];
      float avg_amp;
      float peak;       // largest amplitude in the current event
      uint32_t w;       // delay line write index
      uint32_t run;     // samples in the event
      uint32_t post;    // samples since the event ended
      uint32_t extra;   // extra guard for this event, each side (0 for small pops)
      nb_state_t state;
      uint32_t active;
    } s = {};

    if ((level == 0u) || (level > 5u))
    {
      s.active = 0u;     // bypass; state and delay line refill on re-enable
      return;
    }
    const float threshold = thresholds[level];

    if (s.active == 0u)
    {
      for (uint32_t i = 0u; i < NB_LEN; i++)
      {
        s.bufI[i] = I;
        s.bufQ[i] = Q;
      }
      s.avg_amp = 0.01f;
      s.w = 0u;
      s.run = 0u;
      s.post = 0u;
      s.state = NB_IDLE;
      s.active = 1u;
    }

    // wideband instantaneous amplitude (fast L1 estimate)
    const float amp = fabsf(I) + fabsf(Q);
    const float on  = threshold * s.avg_amp;   // event while above this

    // replace the last 'gap' stored samples by a straight line from the
    // last good sample before them to the current (good) sample
    auto blank = [&](const uint32_t gap)
    {
      const uint32_t ia = (s.w - gap - 1u) & NB_MASK;  // last good sample
      const float aI = s.bufI[ia];
      const float aQ = s.bufQ[ia];
      const float stepI = (I - aI) / (float)(gap + 1u);
      const float stepQ = (Q - aQ) / (float)(gap + 1u);
      for (uint32_t p = 1u; p <= gap; p++)
      {
        const uint32_t idx = (s.w - (gap + 1u - p)) & NB_MASK;
        s.bufI[idx] = aI + stepI * (float)p;
        s.bufQ[idx] = aQ + stepQ * (float)p;
      }
    };

    switch (s.state)
    {
      case NB_IDLE:
      {
        if (amp > on)
        {
          s.state = NB_EVENT;
          s.run = 1u;
          s.peak = amp;
        }
        else
        {
          s.avg_amp += alpha * (amp - s.avg_amp);
        }
        break;
      }
      case NB_EVENT:
      {
        if (amp > on)
        {
          if (amp > s.peak) s.peak = amp;
          if (++s.run > MAX_BLANK_RUN)
          {
            s.state = NB_SIGNAL;             // too long for a pop
          }
        }
        else
        {
          // pop over. A pop only just over the threshold (a noise peak or
          // voice transient, most false triggers) is blanked exactly as
          // before: GUARD_PRE + the samples over the threshold. A pop well
          // over the threshold also gets 'extra' samples on BOTH sides, enough
          // for its ringing to decay back down to the threshold.
          const float over_db = 20.0f * log10f(s.peak / on);
          uint32_t extra = 0u;
          if (over_db > BIG_POP_DB)
          {
            extra = (uint32_t)((over_db - BIG_POP_DB) / RING_DB_PER_SAMPLE) + 1u;
            if (extra > GUARD_EXTRA_MAX) extra = GUARD_EXTRA_MAX;
          }
          if (extra == 0u)
          {
            blank(GUARD_PRE + s.run);        // this sample is clean: blank now
            s.avg_amp += alpha * (amp - s.avg_amp);
            s.state = NB_IDLE;
          }
          else
          {
            s.extra = extra;
            s.post = 1u;                     // this sample is the first trailing one
            s.state = NB_POST;
          }
        }
        break;
      }
      case NB_POST:
      {
        if (amp > on)
        {
          if (s.run + s.post + 1u <= MAX_BLANK_RUN)
          {
            // another pop inside the trailing guard: merge it
            s.run += s.post + 1u;
            s.post = 0u;
            if (amp > s.peak) s.peak = amp;
            s.state = NB_EVENT;
          }
          else
          {
            s.state = NB_SIGNAL;             // a dense train: treat as signal
          }
        }
        else if (s.post < s.extra)
        {
          s.post++;                          // still inside the trailing guard
        }
        else
        {
          // this sample is clean: blank pre-guard + pop + trailing guard
          blank(GUARD_PRE + s.extra + s.run + s.post);
          s.avg_amp += alpha * (amp - s.avg_amp);
          s.state = NB_IDLE;
        }
        break;
      }
      case NB_SIGNAL:
      {
        s.avg_amp += alpha * (amp - s.avg_amp);   // follow the signal
        if (amp <= on)
        {
          s.state = NB_IDLE;
        }
        break;
      }
    }

    // delay line: emit the sample from NB_LEN calls ago, store the current one
    const float outI = s.bufI[s.w];
    const float outQ = s.bufQ[s.w];
    s.bufI[s.w] = I;
    s.bufQ[s.w] = Q;
    s.w = (s.w + 1u) & NB_MASK;
    I = outI;
    Q = outQ;
  }

  static const int16_t __not_in_flash_func(process_ssb)(
    const int16_t in_i,
    const int16_t in_q,
    const uint32_t jnr_level,
    const uint8_t bw,
    const uint8_t nb_level)
  {
    // quadrature mixer

    // quadrature LO = 7812 (quadrature: 31250/4/4 = 1)
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

    // notch
    const float audio_notch = notch.process(audio_raw);

    // JNR
    const float audio_out = FILTER::jnr(audio_notch,jnr_level);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_dig)(const int16_t in_i,const int16_t in_q)
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
    const float audio_out = FILTER::lpf_3000f_rx(ssb);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_ft8)(const int16_t in_i,const int16_t in_q,float &ft8_audio)
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
    ft8_audio = FILTER::lpf_3000f_rx(ssb);

    // AGC for audio monitoring and s-meter
    return agc(ft8_audio * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_cw)(const int16_t in_i,const int16_t in_q,float &sig)
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
    sig = FILTER::bpf_700f(ssb);

    // AGC returns 12 bit value
    return agc(sig * 32768.0f);
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
    static const float max_gain = 200.0f;
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

  static float __not_in_flash_func(mic_process)(const float sample, const uint8_t level)
  {
    // use tanh for soft clipping

    if (level == 0) return sample;
    if (level > 5) return sample;

    // drive values mapped to processing levels 1–5
    // level 0 = bypass (filter state is left untouched)
    static const float DRIVE_TABLE[6] =
    {
      0.0f,   // 0 — bypass
      1.5f,   // 1 — subtle warmth
      3.0f,   // 2 — light SSB processing
      5.0f,   // 3 — typical SSB (good starting point)
      7.5f,   // 4 — aggressive
      12.0f,  // 5 — contest/heavy
    };
  
    const float drive = DRIVE_TABLE[level];
    const float clipped = tanhf(sample * drive) / tanhf(drive);
    return clipped;
  }

  static void __not_in_flash_func(cessb)(float& ii, float& qq)
  {
    ii *= 1.5f;
    qq *= 1.5f;
    const float mag_raw = sqrtf(ii*ii + qq*qq);
    const float mag_max = fmaxf(mag_raw, 1.0f);
    ii = FILTER::lpf_2600if_tx(ii / mag_max);
    qq = FILTER::lpf_2600qf_tx(qq / mag_max);
  }

  static const void __not_in_flash_func(process_mic)(
    const int16_t s,
    int16_t &out_i,
    int16_t &out_q,
    const float mic_gain,
    const uint8_t proc_level,
    const bool cessb_on)
  {
    // input is 12 bits
    // convert to float
    // remove Mic DC
    // mic processing
    // 2400 LPF 
    // phase shift I
    // phase shift Q
    // convert to int
    // output is 10 bits
    const float ac_sig = FILTER::dcf(((float)s)*(1.0f/2048.0f)) * mic_gain;
    const float mic_proc = mic_process(ac_sig,proc_level);
    const float mic_sig = FILTER::lpf_2600f_tx(mic_proc);
    float ii = FILTER::fap1f(mic_sig);
    float qq = FILTER::fap2f(mic_sig);
    if (cessb_on) cessb(ii,qq);
    out_i = (int16_t)(ii * 512.0f);
    out_q = (int16_t)(qq * 512.0f);
  }

  static const void __not_in_flash_func(process_dig)(const int16_t s,int16_t &out_i,int16_t &out_q)
  {
    // input is 12 bits
    // convert to float
    // remove DC
    // 3000 Hz LPF 
    // phase shift I
    // phase shift Q
    // convert to int
    // output is 10 bits
    const float ac_sig = FILTER::dcf(((float)s)*(1.0f/2048.0f));
    const float lp_sig = FILTER::lpf_3000f_tx(ac_sig);
    float ii = FILTER::fap1f(lp_sig);
    float qq = FILTER::fap2f(lp_sig);
    out_i = (int16_t)(ii * 512.0f);
    out_q = (int16_t)(qq * 512.0f);
  }
}

#endif