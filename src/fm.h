#ifndef FM_H
#define FM_H

// ---------------------------------------------------------------------------
//  Narrow band FM for MBPTRX                                          VK7IAN
// ---------------------------------------------------------------------------
//
//  +/-2.5 kHz deviation, 300 to 3000 Hz audio, 6 dB/octave pre-emphasis on
//  transmit and de-emphasis on receive (unity at 1 kHz at both ends).
//
//  RECEIVE     process_rx(I, Q, squelch_level)
//
//    I and Q are the full 24 bit ADC samples (not the 16 bits the other
//    modes use) so signals below one 16 bit step are not lost.
//
//    The LO is set FS/4 below the dial frequency, exactly as for AM, so the
//    channel sits at 7812.5 Hz, clear of the DC offset and the 1/f pedestal
//    at the centre of the baseband. With (I + jQ) as passed here a signal
//    above the LO appears at a NEGATIVE frequency, so the channel is at
//    -FS/4 and is shifted to zero by multiplying by j^n (no multiplies).
//
//      DC block -> shift by FS/4 -> 95 tap channel filter on I and on Q
//      -> phase difference discriminator -> noise squelch
//      -> de-emphasis -> 300 Hz HPF -> 3000 Hz LPF
//
//    There is no AGC, the audio level depends only on the deviation.
//    DSP::agc_peak is still updated from the channel level so the S meter
//    works as in the other modes.
//
//  TRANSMIT    process_tx(mic, I, Q, mic_gain)
//
//    The LO is on the dial frequency and the carrier is a vector that is
//    rotated by the audio, one phase step per sample.
//
//      mic -> 300 Hz HPF -> pre-emphasis -> limiter -> 3000 Hz LPF
//      -> phase rotation of the carrier vector -> RC filter equaliser
//
//    The equaliser compensates the two RC filters after the TX PWM outputs
//    (3400 Hz and 7200 Hz). Without it the deviation is about 2.7 dB down
//    at 3 kHz and the envelope has about 15% ripple.
//
//  NOTE: everything is float, do not let a double sneak in.
// ---------------------------------------------------------------------------

#include <math.h>
#include <stdint.h>
#include "dsp.h"

// peak deviation, Hz
#define FM_DEVIATION 2500.0f

// carrier amplitude on the TX PWM outputs, 511 is full scale
// FM is key down for the whole over, 180 is the same carrier as FT8
// (127 on I and on Q). The equaliser needs headroom: 350 is the limit
#define FM_TX_MAX 180.0f

// TX PWM filter poles as seen at the sample rate: exp(-2*pi*fc/FS)
#define FM_EQ_P1 0.50479f // 3400 Hz
#define FM_EQ_P2 0.23512f // 7200 Hz

namespace FM
{
  // phase step per sample at full deviation, radians
  static constexpr float DPHI_MAX = 2.0f * 3.14159265f * FM_DEVIATION / (float)SAMPLERATE;

  // squelch noise level, for display or debugging
  // about 1.2 on noise alone, below 0.05 on a strong signal
  volatile static float squelch_noise = 2.0f;
  volatile static bool squelch_open = false;

  struct biquad_t
  {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float s1;
    float s2;
  };

  static inline float __not_in_flash_func(biquad)(biquad_t &f, const float x)
  {
    // transposed direct form II
    const float y = f.b0 * x + f.s1;
    f.s1 = f.b1 * x - f.a1 * y + f.s2;
    f.s2 = f.b2 * x - f.a2 * y;
    return y;
  }

  static inline float __not_in_flash_func(fast_atan2)(const float y, const float x)
  {
    // maximum error about 0.00001 radians, no library call
    const float ax = fabsf(x);
    const float ay = fabsf(y);
    const float mx = (ax > ay) ? ax : ay;
    const float mn = (ax > ay) ? ay : ax;
    if (mx < 1.0e-30f)
    {
      return 0.0f;
    }
    const float a = mn / mx;
    const float s = a * a;
    float r = -0.014405605f;
    r = r * s + 0.059598353f;
    r = r * s - 0.122920769f;
    r = r * s + 0.196171683f;
    r = r * s - 0.333053072f;
    r = r * s + 0.999996620f;
    r = r * a;
    if (ay > ax)
    {
      r = 1.57079633f - r;
    }
    if (x < 0.0f)
    {
      r = 3.14159265f - r;
    }
    if (y < 0.0f)
    {
      r = -r;
    }
    return r;
  }

  // channel filter
  // 31250
  // LPF: -6dB at 6100 Hz, flat to 5500 Hz
  // att: 65dB from 7000 Hz
  // 95 taps
  static constexpr uint32_t CHAN_TAPS = 95u;
  static constexpr uint32_t CHAN_LEN = 128u;
  static constexpr uint32_t CHAN_MASK = CHAN_LEN - 1u;
  static const float __not_in_flash("fast_access_sram") chan_fir[CHAN_TAPS] =
  {
    1.005125898e-04f,
    -2.103085340e-05f,
    -2.143761285e-04f,
    -1.525123270e-04f,
    2.284864630e-04f,
    4.381906765e-04f,
    1.146561763e-05f,
    -6.484532952e-04f,
    -5.434331277e-04f,
    4.921729547e-04f,
    1.156637234e-03f,
    2.340112735e-04f,
    -1.393473906e-03f,
    -1.403225068e-03f,
    7.644849316e-04f,
    2.438612462e-03f,
    8.788572339e-04f,
    -2.480404859e-03f,
    -3.021145857e-03f,
    8.620117646e-04f,
    4.478570445e-03f,
    2.302484127e-03f,
    -3.870641931e-03f,
    -5.784004620e-03f,
    4.583369107e-04f,
    7.499065965e-03f,
    5.053767115e-03f,
    -5.445821011e-03f,
    -1.028321545e-02f,
    -1.005722031e-03f,
    1.186768164e-02f,
    1.011606532e-02f,
    -7.018957939e-03f,
    -1.774342300e-02f,
    -4.654058625e-03f,
    1.858393520e-02f,
    1.992378094e-02f,
    -8.367059122e-03f,
    -3.201045612e-02f,
    -1.389228253e-02f,
    3.188647772e-02f,
    4.468469524e-02f,
    -9.278155374e-03f,
    -7.659395987e-02f,
    -5.376693744e-02f,
    1.006547037e-01f,
    2.992724971e-01f,
    3.904104917e-01f,
    2.992724971e-01f,
    1.006547037e-01f,
    -5.376693744e-02f,
    -7.659395987e-02f,
    -9.278155374e-03f,
    4.468469524e-02f,
    3.188647772e-02f,
    -1.389228253e-02f,
    -3.201045612e-02f,
    -8.367059122e-03f,
    1.992378094e-02f,
    1.858393520e-02f,
    -4.654058625e-03f,
    -1.774342300e-02f,
    -7.018957939e-03f,
    1.011606532e-02f,
    1.186768164e-02f,
    -1.005722031e-03f,
    -1.028321545e-02f,
    -5.445821011e-03f,
    5.053767115e-03f,
    7.499065965e-03f,
    4.583369107e-04f,
    -5.784004620e-03f,
    -3.870641931e-03f,
    2.302484127e-03f,
    4.478570445e-03f,
    8.620117646e-04f,
    -3.021145857e-03f,
    -2.480404859e-03f,
    8.788572339e-04f,
    2.438612462e-03f,
    7.644849316e-04f,
    -1.403225068e-03f,
    -1.393473906e-03f,
    2.340112735e-04f,
    1.156637234e-03f,
    4.921729547e-04f,
    -5.434331277e-04f,
    -6.484532952e-04f,
    1.146561763e-05f,
    4.381906765e-04f,
    2.284864630e-04f,
    -1.525123270e-04f,
    -2.143761285e-04f,
    -2.103085340e-05f,
    1.005125898e-04f
  };

  static const int16_t __not_in_flash_func(process_rx)(const int32_t in_i, const int32_t in_q, const uint8_t squelch_level)
  {
    // input is 24 bits
    // remove DC
    // shift the channel from -FS/4 to zero
    // channel filter I and Q
    // discriminator
    // squelch
    // de-emphasis
    // 300 Hz HPF
    // 3000 Hz LPF

    // noise level below which the squelch opens
    // (opens at about 2, 4, 6, 8 and 12 dB carrier to noise)
    static const float squelch_threshold[6] =
    {
      0.0f,  // 0 - squelch off
      0.28f, // 1 - opens on the weakest signals
      0.20f, // 2
      0.14f, // 3
      0.10f, // 4
      0.06f  // 5 - only fully quieting signals
    };
    // the squelch closes again at this much more noise
    static const float squelch_hysteresis = 1.4f;

    // de-emphasis, 530uS (6 dB/octave from 300 Hz), unity at 1 kHz
    static const float deemph_k = 0.058590782f;
    static const float deemph_gain = 3.471141367f;

    // audio out is 12 bits, full deviation at 1 kHz is 80% of full scale
    static const float audio_gain = 0.8f * 2047.0f;

    // S meter decay, about 10dB per second (same as the AGC)
    static const float level_decay = 0.99996f;

    static float buf_i[CHAN_LEN] = { 0.0f };
    static float buf_q[CHAN_LEN] = { 0.0f };
    static uint32_t buf_p = 0;
    static uint32_t rotate = 0;
    static float dc_xi = 0.0f;
    static float dc_yi = 0.0f;
    static float dc_xq = 0.0f;
    static float dc_yq = 0.0f;
    static float last_i = 0.0f;
    static float last_q = 0.0f;
    static float deemph = 0.0f;
    static float noise = 2.0f;
    static float gate = 0.0f;
    static bool is_open = false;

    // squelch noise filter, 4th order HPF 5000 Hz
    static biquad_t sq1 = { 0.250377014f, -0.500754029f, 0.250377014f, -0.602033202f, 0.123559344f, 0.0f, 0.0f };
    static biquad_t sq2 = { 1.0f, -2.0f, 1.0f, -0.809950299f, 0.511589765f, 0.0f, 0.0f };

    // audio filters, 2nd order HPF 300 Hz, 4th order LPF 3000 Hz
    static biquad_t hpf = { 0.958244728f, -1.916489456f, 0.958244728f, -1.914745193f, 0.918233720f, 0.0f, 0.0f };
    static biquad_t lp1 = { 0.004196993f, 0.008393985f, 0.004196993f, -1.080688917f, 0.312260037f, 0.0f, 0.0f };
    static biquad_t lp2 = { 1.0f, 2.0f, 1.0f, -1.353287542f, 0.643271373f, 0.0f, 0.0f };

    // remove DC, the offset would land on FS/4 after the shift
    const float xi = (float)in_i * (1.0f / 8388608.0f);
    const float xq = (float)in_q * (1.0f / 8388608.0f);
    dc_yi = xi - dc_xi + 0.995f * dc_yi;
    dc_xi = xi;
    dc_yq = xq - dc_xq + 0.995f * dc_yq;
    dc_xq = xq;

    // shift up by FS/4: multiply (I + jQ) by 1, j, -1, -j
    float si = 0.0f;
    float sq = 0.0f;
    switch (rotate)
    {
      case 0:
      {
        si = dc_yi;
        sq = dc_yq;
        break;
      }
      case 1:
      {
        si = -dc_yq;
        sq = dc_yi;
        break;
      }
      case 2:
      {
        si = -dc_yi;
        sq = -dc_yq;
        break;
      }
      default:
      {
        si = dc_yq;
        sq = -dc_yi;
        break;
      }
    }
    rotate = (rotate + 1u) & 3u;

    // channel filter
    buf_p = (buf_p - 1u) & CHAN_MASK;
    buf_i[buf_p] = si;
    buf_q[buf_p] = sq;
    float zi = 0.0f;
    float zq = 0.0f;
    uint32_t p = buf_p;
    for (uint32_t k = 0u; k < CHAN_TAPS; k++)
    {
      const float h = chan_fir[k];
      zi += h * buf_i[p];
      zq += h * buf_q[p];
      p = (p + 1u) & CHAN_MASK;
    }

    // signal level for the S meter
    // in 16 bit units, times 2 to match the level the other modes give
    const float magnitude = sqrtf(zi * zi + zq * zq) * (2.0f * 32768.0f);
    if (magnitude > DSP::agc_peak)
    {
      DSP::agc_peak = magnitude;
    }
    else
    {
      DSP::agc_peak *= level_decay;
    }

    // discriminator: phase change since the last sample
    const float re = zi * last_i + zq * last_q;
    const float im = zq * last_i - zi * last_q;
    last_i = zi;
    last_q = zq;
    const float dphi = fast_atan2(im, re);

    // noise squelch: on a signal the discriminator output above the
    // audio band goes quiet, on noise alone it does not
    const float hf = biquad(sq2, biquad(sq1, dphi));
    noise += 0.001f * (fabsf(hf) - noise);
    if (squelch_level == 0u || squelch_level > 5u)
    {
      is_open = true;
    }
    else
    {
      const float threshold = squelch_threshold[squelch_level];
      if (is_open)
      {
        if (noise > threshold * squelch_hysteresis)
        {
          is_open = false;
        }
      }
      else
      {
        if (noise < threshold)
        {
          is_open = true;
        }
      }
    }
    squelch_noise = noise;
    squelch_open = is_open;

    // fade in and out over a few milliseconds
    gate += 0.01f * ((is_open ? 1.0f : 0.0f) - gate);

    // a signal above the LO is at a negative frequency here
    // so negate to make the audio follow the RF
    const float demod = -dphi * (1.0f / DPHI_MAX);

    // de-emphasis
    deemph += deemph_k * (demod - deemph);

    // 300 to 3000 Hz
    const float audio_hpf = biquad(hpf, deemph * deemph_gain);
    const float audio_out = biquad(lp2, biquad(lp1, audio_hpf));

    return (int16_t)(audio_out * gate * audio_gain);
  }

  static const void __not_in_flash_func(process_tx)(const int16_t s, int16_t &out_i, int16_t &out_q, const float mic_gain)
  {
    // input is 12 bits
    // convert to float
    // 300 Hz HPF (also removes the Mic DC)
    // pre-emphasis
    // limiter
    // 3000 Hz LPF
    // rotate the carrier vector
    // equalise the PWM RC filters
    // convert to int
    // output is 10 bits

    // pre-emphasis, 530uS (6 dB/octave from 300 Hz), unity at 1 kHz
    static const float preemph_a = 0.941409218f;
    static const float preemph_gain = 4.916979557f;

    // deviation at the limiter threshold, as a fraction of FM_DEVIATION
    // the 3000 Hz LPF overshoots on clipped audio, this leaves room for it
    static const float limit_deviation = 0.80f;

    // equaliser: (1 - P1/z)(1 - P2/z) scaled for unity gain at DC
    static const float eq_k = 1.0f / ((1.0f - FM_EQ_P1) * (1.0f - FM_EQ_P2));
    static const float eq_0 = eq_k;
    static const float eq_1 = -(FM_EQ_P1 + FM_EQ_P2) * eq_k;
    static const float eq_2 = FM_EQ_P1 * FM_EQ_P2 * eq_k;

    static float pre_x = 0.0f;
    // carrier vector and its last two values (for the equaliser)
    static float car_i = 1.0f;
    static float car_q = 0.0f;
    static float i1 = 1.0f;
    static float i2 = 1.0f;
    static float q1 = 0.0f;
    static float q2 = 0.0f;

    // audio filters, 2nd order HPF 300 Hz, 4th order LPF 3000 Hz
    static biquad_t hpf = { 0.958244728f, -1.916489456f, 0.958244728f, -1.914745193f, 0.918233720f, 0.0f, 0.0f };
    static biquad_t lp1 = { 0.004196993f, 0.008393985f, 0.004196993f, -1.080688917f, 0.312260037f, 0.0f, 0.0f };
    static biquad_t lp2 = { 1.0f, 2.0f, 1.0f, -1.353287542f, 0.643271373f, 0.0f, 0.0f };

    // mic audio, 300 Hz HPF
    const float mic_sig = biquad(hpf, (float)s * (1.0f / 2048.0f) * mic_gain);

    // pre-emphasis
    const float pre_sig = (mic_sig - preemph_a * pre_x) * preemph_gain;
    pre_x = mic_sig;

    // limiter
    float lim_sig = pre_sig;
    if (lim_sig > 1.0f)
    {
      lim_sig = 1.0f;
    }
    if (lim_sig < -1.0f)
    {
      lim_sig = -1.0f;
    }

    // 3000 Hz LPF
    const float mod_sig = biquad(lp2, biquad(lp1, lim_sig));

    // phase step for this sample, radians
    float d = mod_sig * (limit_deviation * DPHI_MAX);
    if (d > 0.7f)
    {
      d = 0.7f;
    }
    if (d < -0.7f)
    {
      d = -0.7f;
    }

    // cos and sin of a small angle
    const float d2 = d * d;
    const float c = 1.0f - d2 * (0.5f - d2 * (1.0f / 24.0f - d2 * (1.0f / 720.0f)));
    const float n = d * (1.0f - d2 * (1.0f / 6.0f - d2 * (1.0f / 120.0f - d2 * (1.0f / 5040.0f))));

    // rotate the carrier vector
    const float ni = car_i * c - car_q * n;
    const float nq = car_i * n + car_q * c;

    // hold the amplitude at one
    const float norm = 1.5f - 0.5f * (ni * ni + nq * nq);
    car_i = ni * norm;
    car_q = nq * norm;

    // equalise the RC filters
    const float ii = eq_0 * car_i + eq_1 * i1 + eq_2 * i2;
    const float qq = eq_0 * car_q + eq_1 * q1 + eq_2 * q2;
    i2 = i1;
    i1 = car_i;
    q2 = q1;
    q1 = car_q;

    // round to nearest
    const float fi = ii * FM_TX_MAX;
    const float fq = qq * FM_TX_MAX;
    out_i = (int16_t)(fi + (fi < 0.0f ? -0.5f : 0.5f));
    out_q = (int16_t)(fq + (fq < 0.0f ? -0.5f : 0.5f));
  }
}

#endif
