#ifndef FILTER_H
#define FILTER_H

#define JNR_FILTER_LENGTH 8
#define BANDWIDTH_2200 1u
#define BANDWIDTH_2400 2u
#define BANDWIDTH_2600 3u
#define BANDWIDTH_2800 4u
#define BANDWIDTH_3000 5u
#define FIR_LENGTH 256
#define MA_FILTER_LENGTH 8
#define ADC_FILTER_LENGTH 16
#define MA_FILTER_MASK (MA_FILTER_LENGTH-1)
#define __mac_tap(_h) acc += (_h)*x[i++]
#define __mac_zap(_h) i++

namespace FILTER
{
  // --------------------------------------------------------------------------
  // rolled FIR filters
  //
  // The FIRs used to be fully unrolled. Every tap recomputed an 8 bit wrapped
  // index, which cost more than the multiply. These loop versions keep the
  // delay line twice, at pos and pos + L, so the newest L samples are always
  // contiguous and the loops need no wrap. Four accumulators are used so that
  // each multiply-add does not wait for the one before it.
  //
  // Linear phase filters are symmetric: h[k] == h[N-1-k]. fir_sym() adds the
  // two samples that share a tap and does one multiply per pair. Its table
  // holds taps 0..(N-1)/2 with the centre tap halved (at the centre both
  // samples are the same sample, so 2*x times h/2 is the centre tap exactly),
  // padded with zeros to a multiple of 4.
  //
  // Half-band filters (fs/4) also have every other tap zero. fir_halfband()
  // only does the non-zero pairs, plus the centre tap.
  //
  // fir_gen() is a plain FIR for filters that are not symmetric.
  // Its table is padded with zeros to a multiple of 4.
  //
  // Every filter has its own delay line, except the receive bandwidth filters
  // which share one (only one of them is used per sample).
  // --------------------------------------------------------------------------

  template <uint32_t L>
  struct fir_state_t
  {
    float x[2u * L];
    uint32_t pos;
  };

  template <uint32_t L>
  static inline const float * __not_in_flash_func(fir_push)(fir_state_t<L> &s, const float sample)
  {
    static_assert((L & (L - 1u)) == 0u, "L must be a power of two");
    // newest sample at [0], older samples at higher addresses
    s.pos = (s.pos - 1u) & (L - 1u);
    s.x[s.pos] = sample;
    s.x[s.pos + L] = sample;
    return &s.x[s.pos];
  }

  template <uint32_t N, uint32_t L>
  static __attribute__((noinline)) const float __not_in_flash_func(fir_sym)(fir_state_t<L> &s, const float sample, const float * const h)
  {
    static_assert((N & 1u) == 1u, "N must be odd");
    static_assert(N <= L, "delay line too short");
    constexpr uint32_t P = (((N + 1u) / 2u) + 3u) & ~3u;
    const float * a = fir_push<L>(s, sample);
    const float * b = a + (N - 1u);
    float acc0 = 0.0f;
    float acc1 = 0.0f;
    float acc2 = 0.0f;
    float acc3 = 0.0f;
    for (uint32_t k = 0; k < P; k += 4u)
    {
      acc0 += h[k + 0u] * (a[0] + b[0]);
      acc1 += h[k + 1u] * (a[1] + b[-1]);
      acc2 += h[k + 2u] * (a[2] + b[-2]);
      acc3 += h[k + 3u] * (a[3] + b[-3]);
      a += 4;
      b -= 4;
    }
    return (acc0 + acc1) + (acc2 + acc3);
  }

  template <uint32_t N, uint32_t L>
  static __attribute__((noinline)) const float __not_in_flash_func(fir_halfband)(fir_state_t<L> &s, const float sample, const float * const h, const float centre)
  {
    static_assert((N & 3u) == 3u, "half-band N must be 4M+3");
    static_assert(N <= L, "delay line too short");
    // non-zero pairs at k = 0, 2, 4 ... (N-3)/2
    constexpr uint32_t P = (N + 1u) / 4u;
    static_assert((P & 3u) == 0u, "pairs must be a multiple of 4");
    const float * const x = fir_push<L>(s, sample);
    const float * a = x;
    const float * b = x + (N - 1u);
    float acc0 = 0.0f;
    float acc1 = 0.0f;
    float acc2 = 0.0f;
    float acc3 = 0.0f;
    for (uint32_t k = 0; k < P; k += 4u)
    {
      acc0 += h[k + 0u] * (a[0] + b[0]);
      acc1 += h[k + 1u] * (a[2] + b[-2]);
      acc2 += h[k + 2u] * (a[4] + b[-4]);
      acc3 += h[k + 3u] * (a[6] + b[-6]);
      a += 8;
      b -= 8;
    }
    return (acc0 + acc1) + (acc2 + acc3) + (centre * x[(N - 1u) / 2u]);
  }

  template <uint32_t N, uint32_t L>
  static __attribute__((noinline)) const float __not_in_flash_func(fir_gen)(fir_state_t<L> &s, const float sample, const float * const h)
  {
    constexpr uint32_t P = (N + 3u) & ~3u;
    static_assert(P <= 2u * L - (L - 1u), "delay line too short");
    const float * a = fir_push<L>(s, sample);
    float acc0 = 0.0f;
    float acc1 = 0.0f;
    float acc2 = 0.0f;
    float acc3 = 0.0f;
    for (uint32_t k = 0; k < P; k += 4u)
    {
      acc0 += h[k + 0u] * a[0];
      acc1 += h[k + 1u] * a[1];
      acc2 += h[k + 2u] * a[2];
      acc3 += h[k + 3u] * a[3];
      a += 4;
    }
    return (acc0 + acc1) + (acc2 + acc3);
  }

  // shared by the receive bandwidth filters
  static fir_state_t<FIR_LENGTH> rx_bw_state = {};

  static const float __not_in_flash_func(ma4i)(const float v1)
  {
    static float delay1[MA_FILTER_LENGTH] = { 0 };
    static float delay2[MA_FILTER_LENGTH] = { 0 };
    static float delay3[MA_FILTER_LENGTH] = { 0 };
    static float delay4[MA_FILTER_LENGTH] = { 0 };
    static float sum1 = 0;
    static float sum2 = 0;
    static float sum3 = 0;
    static float sum4 = 0;
    static uint8_t p = 0;
    sum1 = sum1 - delay1[p] + v1;
    delay1[p] = v1;
    const float v2 = sum1 / (float)MA_FILTER_LENGTH;
    sum2 = sum2 - delay2[p] + v2;
    delay2[p] = v2;
    const float v3 = sum2 / (float)MA_FILTER_LENGTH;
    sum3 = sum3 - delay3[p] + v3;
    delay3[p] = v3;
    const float v4 = sum3 / (float)MA_FILTER_LENGTH;
    sum4 = sum4 - delay4[p] + v4;
    delay4[p] = v4;
    p++;
    p &= MA_FILTER_MASK;
    return sum4 / (float)MA_FILTER_LENGTH;
  }

  static const float __not_in_flash_func(ma4q)(const float v1)
  {
    static float delay1[MA_FILTER_LENGTH] = { 0 };
    static float delay2[MA_FILTER_LENGTH] = { 0 };
    static float delay3[MA_FILTER_LENGTH] = { 0 };
    static float delay4[MA_FILTER_LENGTH] = { 0 };
    static float sum1 = 0;
    static float sum2 = 0;
    static float sum3 = 0;
    static float sum4 = 0;
    static uint8_t p = 0;
    sum1 = sum1 - delay1[p] + v1;
    delay1[p] = v1;
    const float v2 = sum1 / (float)MA_FILTER_LENGTH;
    sum2 = sum2 - delay2[p] + v2;
    delay2[p] = v2;
    const float v3 = sum2 / (float)MA_FILTER_LENGTH;
    sum3 = sum3 - delay3[p] + v3;
    delay3[p] = v3;
    const float v4 = sum3 / (float)MA_FILTER_LENGTH;
    sum4 = sum4 - delay4[p] + v4;
    delay4[p] = v4;
    p++;
    p &= MA_FILTER_MASK;
    return sum4 / (float)MA_FILTER_LENGTH;
  }

  static float __not_in_flash_func(dcf)(const float in)
  {
    // single pole IIR high-pass filter
    //static const float k = 0.004f; // <100Hz
    //static const float k = 0.01f; // ~100Hz
    //static const float k = 0.1f; // ~300Hz
    static const float k = 0.05f; // ~200Hz
    static float s = 0;
    static float x1 = 0;
    static float y1 = 0;
    s -= x1;
    x1 = in;
    s += x1 - y1 * k;
    return (y1 = s);
  }

  static float __not_in_flash_func(dc1f)(const float in)
  {
    static const float k = 0.05f;
    static float s = 0;
    static float x1 = 0;
    static float y1 = 0;
    s -= x1;
    x1 = in;
    s += x1 - y1 * k;
    return (y1 = s);
  }

  static float __not_in_flash_func(dc2f)(const float in)
  {
    static const float k = 0.05f;
    static float s = 0;
    static float x1 = 0;
    static float y1 = 0;
    s -= x1;
    x1 = in;
    s += x1 - y1 * k;
    return (y1 = s);
  }

  // 3-section all-pass pair, 250-2700 Hz @ 31250 Hz
  // phase(fap1f) - phase(fap2f) = -90 degrees, within 0.092 degrees
  // worst-case opposite-sideband suppression 61.9 dB
  static const float __not_in_flash_func(fap1f)(const float s)
  {
    // all pass 80.4 @ 31250
    // all pass 587 @ 31250
    // all pass 2465 @ 31250
    static const float k1 = 0.9839554f;
    static const float k2 = 0.8883701f;
    static const float k3 = 0.5962229f;
    static float x1 = 0.0f;
    static float y1 = 0.0f;
    static float x2 = 0.0f;
    static float y2 = 0.0f;
    static float x3 = 0.0f;
    static float y3 = 0.0f;
    y1 = (k1 * (s + y1)) - x1;
    x1 = s;
    y2 = (k2 * (y1 + y2)) - x2;
    x2 = y1;
    y3 = (k3 * (y2 + y3)) - x3;
    x3 = y2;
    return y3;
  }

  static const float __not_in_flash_func(fap2f)(const float s)
  {
    // all pass 275 @ 31250
    // all pass 1172 @ 31250
    // all pass 7094 @ 31250
    static const float k1 = 0.9461753f;
    static const float k2 = 0.7883548f;
    static const float k3 = 0.0723467f;
    static float x1 = 0.0f;
    static float y1 = 0.0f;
    static float x2 = 0.0f;
    static float y2 = 0.0f;
    static float x3 = 0.0f;
    static float y3 = 0.0f;
    y1 = (k1 * (s + y1)) - x1;
    x1 = s;
    y2 = (k2 * (y1 + y2)) - x2;
    x2 = y1;
    y3 = (k3 * (y2 + y3)) - x3;
    x3 = y2;
    return y3;
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_2200f_rx[128] =
  {
    // 31250
    // 2200 Hz
    // att: 60dB
    // 255 taps
    -0.000019f,
    -0.000042f,
    -0.000062f,
    -0.000073f,
    -0.000069f,
    -0.000048f,
    -0.000012f,
    0.000036f,
    0.000085f,
    0.000125f,
    0.000145f,
    0.000137f,
    0.000098f,
    0.00003f,
    -0.000056f,
    -0.000143f,
    -0.000214f,
    -0.00025f,
    -0.000238f,
    -0.000174f,
    -0.000063f,
    0.000077f,
    0.000219f,
    0.000335f,
    0.000396f,
    0.000381f,
    0.000285f,
    0.000118f,
    -0.000095f,
    -0.000313f,
    -0.000493f,
    -0.00059f,
    -0.000577f,
    -0.000443f,
    -0.000203f,
    0.000106f,
    0.000426f,
    0.000692f,
    0.000844f,
    0.000838f,
    0.00066f,
    0.000328f,
    -0.000103f,
    -0.000556f,
    -0.000938f,
    -0.001166f,
    -0.001178f,
    -0.00095f,
    -0.000508f,
    0.000078f,
    0.000701f,
    0.001237f,
    0.00157f,
    0.001613f,
    0.001333f,
    0.000758f,
    -0.000019f,
    -0.000858f,
    -0.001594f,
    -0.002069f,
    -0.002165f,
    -0.001831f,
    -0.0011f,
    -0.000088f,
    0.001024f,
    0.002018f,
    0.002685f,
    0.002862f,
    0.002477f,
    0.001562f,
    0.000261f,
    -0.001194f,
    -0.002522f,
    -0.003445f,
    -0.003746f,
    -0.003316f,
    -0.002186f,
    -0.000528f,
    0.001363f,
    0.003126f,
    0.004396f,
    0.00488f,
    0.00442f,
    0.003035f,
    0.000931f,
    -0.001524f,
    -0.003866f,
    -0.005615f,
    -0.006373f,
    -0.005909f,
    -0.004217f,
    -0.001537f,
    0.001673f,
    0.004815f,
    0.007251f,
    0.008432f,
    0.008014f,
    0.00594f,
    0.002478f,
    -0.001804f,
    -0.006122f,
    -0.009613f,
    -0.011496f,
    -0.011234f,
    -0.008658f,
    -0.004042f,
    0.001911f,
    0.008158f,
    0.013479f,
    0.016695f,
    0.016891f,
    0.013623f,
    0.007056f,
    -0.001991f,
    -0.012107f,
    -0.02146f,
    -0.028052f,
    -0.030029f,
    -0.025986f,
    -0.015232f,
    0.00204f,
    0.024712f,
    0.050827f,
    0.077816f,
    0.102819f,
    0.123061f,
    0.136232f,
    0.5f * 0.1408f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_2200f_rx)(const float sample)
  {
    return fir_sym<255u, FIR_LENGTH>(rx_bw_state, sample, h_lpf_2200f_rx);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_2400f_rx[128] =
  {
    // 31250
    // 2400 Hz
    // att: 60dB
    // 255 taps
    -0.000051f,
    -0.000052f,
    -0.000039f,
    -0.000011f,
    0.000027f,
    0.000067f,
    0.000097f,
    0.000109f,
    0.000094f,
    0.000051f,
    -0.000013f,
    -0.000086f,
    -0.00015f,
    -0.000186f,
    -0.000181f,
    -0.000129f,
    -0.000036f,
    0.00008f,
    0.000194f,
    0.000276f,
    0.000301f,
    0.000254f,
    0.000137f,
    -0.00003f,
    -0.000211f,
    -0.000364f,
    -0.000446f,
    -0.000428f,
    -0.000302f,
    -0.000087f,
    0.000174f,
    0.000424f,
    0.000598f,
    0.000646f,
    0.000541f,
    0.000292f,
    -0.000054f,
    -0.000424f,
    -0.000729f,
    -0.000889f,
    -0.000849f,
    -0.000599f,
    -0.00018f,
    0.000325f,
    0.000799f,
    0.001126f,
    0.001212f,
    0.001013f,
    0.000552f,
    -0.000085f,
    -0.000758f,
    -0.00131f,
    -0.001595f,
    -0.001522f,
    -0.001077f,
    -0.000335f,
    0.000551f,
    0.001379f,
    0.001947f,
    0.002095f,
    0.001754f,
    0.000964f,
    -0.00012f,
    -0.001264f,
    -0.002198f,
    -0.002681f,
    -0.002561f,
    -0.001821f,
    -0.000587f,
    0.000884f,
    0.002259f,
    0.003202f,
    0.003455f,
    0.002902f,
    0.001615f,
    -0.000157f,
    -0.002029f,
    -0.00356f,
    -0.004361f,
    -0.004184f,
    -0.002996f,
    -0.001001f,
    0.001386f,
    0.003629f,
    0.005181f,
    0.005619f,
    0.004751f,
    0.002682f,
    -0.000192f,
    -0.003251f,
    -0.005781f,
    -0.007138f,
    -0.006902f,
    -0.004996f,
    -0.001735f,
    0.00222f,
    0.005989f,
    0.008657f,
    0.00949f,
    0.008124f,
    0.004679f,
    -0.000221f,
    -0.005549f,
    -0.010079f,
    -0.012649f,
    -0.012438f,
    -0.009194f,
    -0.003348f,
    0.00401f,
    0.011309f,
    0.0168f,
    0.018929f,
    0.016713f,
    0.010029f,
    -0.00024f,
    -0.01226f,
    -0.023497f,
    -0.031128f,
    -0.032559f,
    -0.025935f,
    -0.010561f,
    0.012861f,
    0.042231f,
    0.074304f,
    0.105149f,
    0.130759f,
    0.147685f,
    0.5f * 0.1536f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_2400f_rx)(const float sample)
  {
    return fir_sym<255u, FIR_LENGTH>(rx_bw_state, sample, h_lpf_2400f_rx);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_2600f_rx[128] =
  {
    // 31250
    // 2600 Hz
    // att: 60dB
    // 255 taps
    -0.000021f,
    0.000006f,
    0.000039f,
    0.000067f,
    0.000082f,
    0.000074f,
    0.000041f,
    -0.000011f,
    -0.000071f,
    -0.000122f,
    -0.000145f,
    -0.000129f,
    -0.000071f,
    0.000018f,
    0.000116f,
    0.000197f,
    0.000232f,
    0.000204f,
    0.000112f,
    -0.000026f,
    -0.000177f,
    -0.000297f,
    -0.000348f,
    -0.000305f,
    -0.000168f,
    0.000036f,
    0.000255f,
    0.000428f,
    0.000499f,
    0.000435f,
    0.00024f,
    -0.000048f,
    -0.000354f,
    -0.000594f,
    -0.000691f,
    -0.000602f,
    -0.000332f,
    0.000061f,
    0.000477f,
    0.000802f,
    0.000932f,
    0.000811f,
    0.000449f,
    -0.000076f,
    -0.000629f,
    -0.001058f,
    -0.001229f,
    -0.00107f,
    -0.000594f,
    0.000092f,
    0.000814f,
    0.001373f,
    0.001595f,
    0.001389f,
    0.000775f,
    -0.000109f,
    -0.001038f,
    -0.001755f,
    -0.00204f,
    -0.001778f,
    -0.000997f,
    0.000127f,
    0.001308f,
    0.002219f,
    0.002583f,
    0.002255f,
    0.001271f,
    -0.000146f,
    -0.001633f,
    -0.002782f,
    -0.003245f,
    -0.002839f,
    -0.001609f,
    0.000164f,
    0.002029f,
    0.003472f,
    0.004059f,
    0.00356f,
    0.002031f,
    -0.000183f,
    -0.002516f,
    -0.004328f,
    -0.005075f,
    -0.004466f,
    -0.002565f,
    0.0002f,
    0.003127f,
    0.005413f,
    0.006372f,
    0.005631f,
    0.003259f,
    -0.000216f,
    -0.003918f,
    -0.006835f,
    -0.008088f,
    -0.007188f,
    -0.004199f,
    0.000231f,
    0.004994f,
    0.008795f,
    0.010485f,
    0.009391f,
    0.005549f,
    -0.000243f,
    -0.006569f,
    -0.011718f,
    -0.014124f,
    -0.012803f,
    -0.007688f,
    0.000253f,
    0.00916f,
    0.016666f,
    0.020472f,
    0.018954f,
    0.011692f,
    -0.000261f,
    -0.014419f,
    -0.027262f,
    -0.034914f,
    -0.033976f,
    -0.02233f,
    0.000265f,
    0.031935f,
    0.068874f,
    0.105951f,
    0.137611f,
    0.158899f,
    0.5f * 0.1664f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_2600f_rx)(const float sample)
  {
    return fir_sym<255u, FIR_LENGTH>(rx_bw_state, sample, h_lpf_2600f_rx);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_2800f_rx[128] =
  {
    // 31250
    // 2800 Hz
    // att: 60dB
    // 255 taps
    0.000035f,
    0.000056f,
    0.000062f,
    0.000047f,
    0.000011f,
    -0.000038f,
    -0.000085f,
    -0.000111f,
    -0.000104f,
    -0.000059f,
    0.000015f,
    0.000098f,
    0.000162f,
    0.000181f,
    0.000142f,
    0.000048f,
    -0.000078f,
    -0.000197f,
    -0.000267f,
    -0.000257f,
    -0.00016f,
    0.000005f,
    0.000191f,
    0.000338f,
    0.000392f,
    0.000323f,
    0.000137f,
    -0.000118f,
    -0.000364f,
    -0.00052f,
    -0.000522f,
    -0.000352f,
    -0.000047f,
    0.000308f,
    0.000601f,
    0.000729f,
    0.000631f,
    0.000316f,
    -0.000136f,
    -0.00059f,
    -0.000897f,
    -0.000942f,
    -0.000684f,
    -0.000178f,
    0.000435f,
    0.000965f,
    0.001231f,
    0.001121f,
    0.000637f,
    -0.000099f,
    -0.000869f,
    -0.001426f,
    -0.00157f,
    -0.001217f,
    -0.00044f,
    0.000546f,
    0.00144f,
    0.001944f,
    0.001862f,
    0.001171f,
    0.000044f,
    -0.001189f,
    -0.002139f,
    -0.002476f,
    -0.002042f,
    -0.000914f,
    0.000599f,
    0.002038f,
    0.002934f,
    0.002956f,
    0.002026f,
    0.000372f,
    -0.00153f,
    -0.003088f,
    -0.003772f,
    -0.003295f,
    -0.001721f,
    0.000528f,
    0.002781f,
    0.004312f,
    0.004574f,
    0.003385f,
    0.001014f,
    -0.001867f,
    -0.004372f,
    -0.005666f,
    -0.005238f,
    -0.00309f,
    0.000224f,
    0.003732f,
    0.006321f,
    0.007085f,
    0.005627f,
    0.002233f,
    -0.002169f,
    -0.006248f,
    -0.008664f,
    -0.008492f,
    -0.005557f,
    -0.000554f,
    0.005098f,
    0.009639f,
    0.011496f,
    0.009802f,
    0.00474f,
    -0.00241f,
    -0.009566f,
    -0.014413f,
    -0.015113f,
    -0.01093f,
    -0.002597f,
    0.007739f,
    0.017005f,
    0.022029f,
    0.020501f,
    0.011799f,
    -0.002565f,
    -0.019046f,
    -0.032746f,
    -0.038532f,
    -0.032343f,
    -0.012346f,
    0.020351f,
    0.061665f,
    0.105213f,
    0.143574f,
    0.169856f,
    0.5f * 0.1792f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_2800f_rx)(const float sample)
  {
    return fir_sym<255u, FIR_LENGTH>(rx_bw_state, sample, h_lpf_2800f_rx);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_3000f_rx[128] =
  {
    // 31250
    // 3000 Hz
    // att: 60dB
    // 255 taps
    0.000048f,
    0.000033f,
    0.0f,
    -0.000042f,
    -0.000077f,
    -0.000089f,
    -0.000067f,
    -0.000014f,
    0.000056f,
    0.000118f,
    0.000145f,
    0.00012f,
    0.000043f,
    -0.000064f,
    -0.000164f,
    -0.000217f,
    -0.000194f,
    -0.000092f,
    0.00006f,
    0.000212f,
    0.000304f,
    0.000293f,
    0.000168f,
    -0.000037f,
    -0.000256f,
    -0.000406f,
    -0.000421f,
    -0.000278f,
    -0.000013f,
    0.000289f,
    0.000518f,
    0.000578f,
    0.000427f,
    0.000099f,
    -0.000303f,
    -0.000635f,
    -0.000765f,
    -0.000622f,
    -0.000231f,
    0.000286f,
    0.000749f,
    0.000979f,
    0.000867f,
    0.000421f,
    -0.000225f,
    -0.000849f,
    -0.001216f,
    -0.001166f,
    -0.000678f,
    0.000106f,
    0.000921f,
    0.001468f,
    0.00152f,
    0.001013f,
    0.000087f,
    -0.00095f,
    -0.001725f,
    -0.001928f,
    -0.001436f,
    -0.000372f,
    0.000917f,
    0.001974f,
    0.002389f,
    0.001957f,
    0.000768f,
    -0.000798f,
    -0.002196f,
    -0.002895f,
    -0.002584f,
    -0.001296f,
    0.000569f,
    0.00237f,
    0.003439f,
    0.003326f,
    0.001981f,
    -0.000197f,
    -0.00247f,
    -0.00401f,
    -0.004195f,
    -0.002851f,
    -0.000355f,
    0.002464f,
    0.004595f,
    0.005202f,
    0.003943f,
    0.001135f,
    -0.002311f,
    -0.00518f,
    -0.006371f,
    -0.005312f,
    -0.002212f,
    0.001957f,
    0.005748f,
    0.007738f,
    0.007041f,
    0.003686f,
    -0.001321f,
    -0.006284f,
    -0.00937f,
    -0.009272f,
    -0.00573f,
    0.000276f,
    0.006771f,
    0.011405f,
    0.012276f,
    0.008665f,
    0.001418f,
    -0.007193f,
    -0.014136f,
    -0.016632f,
    -0.013187f,
    -0.004283f,
    0.007537f,
    0.018304f,
    0.023833f,
    0.021163f,
    0.009783f,
    -0.007791f,
    -0.026335f,
    -0.039134f,
    -0.039809f,
    -0.024242f,
    0.007947f,
    0.052865f,
    0.102946f,
    0.148609f,
    0.180539f,
    0.5f * 0.192f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_3000f_rx)(const float sample)
  {
    return fir_sym<255u, FIR_LENGTH>(rx_bw_state, sample, h_lpf_3000f_rx);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_2600f_tx[128] =
  {
    // 31250
    // att: 60dB
    -0.000021f,
    0.000006f,
    0.000039f,
    0.000067f,
    0.000082f,
    0.000074f,
    0.000041f,
    -0.000011f,
    -0.000071f,
    -0.000122f,
    -0.000145f,
    -0.000129f,
    -0.000071f,
    0.000018f,
    0.000116f,
    0.000197f,
    0.000232f,
    0.000204f,
    0.000112f,
    -0.000026f,
    -0.000177f,
    -0.000297f,
    -0.000348f,
    -0.000305f,
    -0.000168f,
    0.000036f,
    0.000255f,
    0.000428f,
    0.000499f,
    0.000435f,
    0.00024f,
    -0.000048f,
    -0.000354f,
    -0.000594f,
    -0.000691f,
    -0.000602f,
    -0.000332f,
    0.000061f,
    0.000477f,
    0.000802f,
    0.000932f,
    0.000811f,
    0.000449f,
    -0.000076f,
    -0.000629f,
    -0.001058f,
    -0.001229f,
    -0.00107f,
    -0.000594f,
    0.000092f,
    0.000814f,
    0.001373f,
    0.001595f,
    0.001389f,
    0.000775f,
    -0.000109f,
    -0.001038f,
    -0.001755f,
    -0.00204f,
    -0.001778f,
    -0.000997f,
    0.000127f,
    0.001308f,
    0.002219f,
    0.002583f,
    0.002255f,
    0.001271f,
    -0.000146f,
    -0.001633f,
    -0.002782f,
    -0.003245f,
    -0.002839f,
    -0.001609f,
    0.000164f,
    0.002029f,
    0.003472f,
    0.004059f,
    0.00356f,
    0.002031f,
    -0.000183f,
    -0.002516f,
    -0.004328f,
    -0.005075f,
    -0.004466f,
    -0.002565f,
    0.0002f,
    0.003127f,
    0.005413f,
    0.006372f,
    0.005631f,
    0.003259f,
    -0.000216f,
    -0.003918f,
    -0.006835f,
    -0.008088f,
    -0.007188f,
    -0.004199f,
    0.000231f,
    0.004994f,
    0.008795f,
    0.010485f,
    0.009391f,
    0.005549f,
    -0.000243f,
    -0.006569f,
    -0.011718f,
    -0.014124f,
    -0.012803f,
    -0.007688f,
    0.000253f,
    0.00916f,
    0.016666f,
    0.020472f,
    0.018954f,
    0.011692f,
    -0.000261f,
    -0.014419f,
    -0.027262f,
    -0.034914f,
    -0.033976f,
    -0.02233f,
    0.000265f,
    0.031935f,
    0.068874f,
    0.105951f,
    0.137611f,
    0.158899f,
    0.5f * 0.1664f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_2600f_tx)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_sym<255u, FIR_LENGTH>(s, sample, h_lpf_2600f_tx);
  }
  
  static const float __not_in_flash("fast_access_sram") h_lpf_3000f_tx[128] =
  {
    // 31250
    // 3000 Hz
    // att: 60dB
    // 255 taps
    0.000048f,
    0.000033f,
    0.0f,
    -0.000042f,
    -0.000077f,
    -0.000089f,
    -0.000067f,
    -0.000014f,
    0.000056f,
    0.000118f,
    0.000145f,
    0.00012f,
    0.000043f,
    -0.000064f,
    -0.000164f,
    -0.000217f,
    -0.000194f,
    -0.000092f,
    0.00006f,
    0.000212f,
    0.000304f,
    0.000293f,
    0.000168f,
    -0.000037f,
    -0.000256f,
    -0.000406f,
    -0.000421f,
    -0.000278f,
    -0.000013f,
    0.000289f,
    0.000518f,
    0.000578f,
    0.000427f,
    0.000099f,
    -0.000303f,
    -0.000635f,
    -0.000765f,
    -0.000622f,
    -0.000231f,
    0.000286f,
    0.000749f,
    0.000979f,
    0.000867f,
    0.000421f,
    -0.000225f,
    -0.000849f,
    -0.001216f,
    -0.001166f,
    -0.000678f,
    0.000106f,
    0.000921f,
    0.001468f,
    0.00152f,
    0.001013f,
    0.000087f,
    -0.00095f,
    -0.001725f,
    -0.001928f,
    -0.001436f,
    -0.000372f,
    0.000917f,
    0.001974f,
    0.002389f,
    0.001957f,
    0.000768f,
    -0.000798f,
    -0.002196f,
    -0.002895f,
    -0.002584f,
    -0.001296f,
    0.000569f,
    0.00237f,
    0.003439f,
    0.003326f,
    0.001981f,
    -0.000197f,
    -0.00247f,
    -0.00401f,
    -0.004195f,
    -0.002851f,
    -0.000355f,
    0.002464f,
    0.004595f,
    0.005202f,
    0.003943f,
    0.001135f,
    -0.002311f,
    -0.00518f,
    -0.006371f,
    -0.005312f,
    -0.002212f,
    0.001957f,
    0.005748f,
    0.007738f,
    0.007041f,
    0.003686f,
    -0.001321f,
    -0.006284f,
    -0.00937f,
    -0.009272f,
    -0.00573f,
    0.000276f,
    0.006771f,
    0.011405f,
    0.012276f,
    0.008665f,
    0.001418f,
    -0.007193f,
    -0.014136f,
    -0.016632f,
    -0.013187f,
    -0.004283f,
    0.007537f,
    0.018304f,
    0.023833f,
    0.021163f,
    0.009783f,
    -0.007791f,
    -0.026335f,
    -0.039134f,
    -0.039809f,
    -0.024242f,
    0.007947f,
    0.052865f,
    0.102946f,
    0.148609f,
    0.180539f,
    0.5f * 0.192f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_3000f_tx)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_sym<255u, FIR_LENGTH>(s, sample, h_lpf_3000f_tx);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_2600if_tx[64] =
  {
    // 31250
    // 2600 Hz
    // att: 60dB
    // 125 taps
    0.000088f,
    0.000062f,
    -0.000009f,
    -0.000114f,
    -0.000226f,
    -0.000304f,
    -0.000303f,
    -0.000194f,
    0.000022f,
    0.000305f,
    0.000576f,
    0.00074f,
    0.000709f,
    0.00044f,
    -0.000043f,
    -0.000637f,
    -0.001178f,
    -0.00148f,
    -0.001391f,
    -0.000851f,
    0.00007f,
    0.001167f,
    0.002135f,
    0.00265f,
    0.002464f,
    0.001497f,
    -0.000104f,
    -0.001971f,
    -0.003589f,
    -0.004425f,
    -0.00409f,
    -0.002481f,
    0.000141f,
    0.003166f,
    0.005762f,
    0.007088f,
    0.006543f,
    0.003978f,
    -0.000179f,
    -0.004967f,
    -0.009082f,
    -0.011206f,
    -0.010386f,
    -0.006369f,
    0.000214f,
    0.007885f,
    0.014599f,
    0.018227f,
    0.017134f,
    0.010719f,
    -0.000242f,
    -0.013552f,
    -0.025902f,
    -0.033499f,
    -0.032885f,
    -0.02178f,
    0.00026f,
    0.031531f,
    0.068316f,
    0.105468f,
    0.137332f,
    0.158818f,
    0.5f * 0.1664f,  // centre tap, halved
    0.0f  // padding
  };

  static const float __not_in_flash_func(lpf_2600if_tx)(const float sample)
  {
    static fir_state_t<(FIR_LENGTH / 2u)> s = {};
    return fir_sym<125u, (FIR_LENGTH / 2u)>(s, sample, h_lpf_2600if_tx);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_2600qf_tx[64] =
  {
    // 31250
    // 2600 Hz
    // att: 60dB
    // 125 taps
    0.000088f,
    0.000062f,
    -0.000009f,
    -0.000114f,
    -0.000226f,
    -0.000304f,
    -0.000303f,
    -0.000194f,
    0.000022f,
    0.000305f,
    0.000576f,
    0.00074f,
    0.000709f,
    0.00044f,
    -0.000043f,
    -0.000637f,
    -0.001178f,
    -0.00148f,
    -0.001391f,
    -0.000851f,
    0.00007f,
    0.001167f,
    0.002135f,
    0.00265f,
    0.002464f,
    0.001497f,
    -0.000104f,
    -0.001971f,
    -0.003589f,
    -0.004425f,
    -0.00409f,
    -0.002481f,
    0.000141f,
    0.003166f,
    0.005762f,
    0.007088f,
    0.006543f,
    0.003978f,
    -0.000179f,
    -0.004967f,
    -0.009082f,
    -0.011206f,
    -0.010386f,
    -0.006369f,
    0.000214f,
    0.007885f,
    0.014599f,
    0.018227f,
    0.017134f,
    0.010719f,
    -0.000242f,
    -0.013552f,
    -0.025902f,
    -0.033499f,
    -0.032885f,
    -0.02178f,
    0.00026f,
    0.031531f,
    0.068316f,
    0.105468f,
    0.137332f,
    0.158818f,
    0.5f * 0.1664f,  // centre tap, halved
    0.0f  // padding
  };

  static const float __not_in_flash_func(lpf_2600qf_tx)(const float sample)
  {
    static fir_state_t<(FIR_LENGTH / 2u)> s = {};
    return fir_sym<125u, (FIR_LENGTH / 2u)>(s, sample, h_lpf_2600qf_tx);
  }

  static const float __not_in_flash("fast_access_sram") h_bpf_700f[128] =
  {
    // 31250
    // att: 60dB
    // Lo: 600
    // Hi: 800
    0.000032f,
    0.000029f,
    0.000024f,
    0.000015f,
    0.000003f,
    -0.000013f,
    -0.000032f,
    -0.000056f,
    -0.000084f,
    -0.000116f,
    -0.00015f,
    -0.000187f,
    -0.000225f,
    -0.000264f,
    -0.000301f,
    -0.000336f,
    -0.000367f,
    -0.000391f,
    -0.000408f,
    -0.000414f,
    -0.000409f,
    -0.000391f,
    -0.000358f,
    -0.00031f,
    -0.000244f,
    -0.000162f,
    -0.000062f,
    0.000054f,
    0.000185f,
    0.000329f,
    0.000485f,
    0.000648f,
    0.000816f,
    0.000984f,
    0.001148f,
    0.001302f,
    0.001441f,
    0.00156f,
    0.001654f,
    0.001717f,
    0.001744f,
    0.001731f,
    0.001675f,
    0.001572f,
    0.00142f,
    0.001219f,
    0.000969f,
    0.000672f,
    0.000331f,
    -0.000049f,
    -0.000463f,
    -0.000902f,
    -0.001358f,
    -0.001822f,
    -0.002282f,
    -0.002727f,
    -0.003146f,
    -0.003526f,
    -0.003855f,
    -0.004122f,
    -0.004315f,
    -0.004426f,
    -0.004446f,
    -0.004368f,
    -0.004187f,
    -0.003901f,
    -0.003511f,
    -0.003019f,
    -0.002429f,
    -0.00175f,
    -0.000992f,
    -0.000168f,
    0.000706f,
    0.001615f,
    0.002538f,
    0.003456f,
    0.004347f,
    0.005191f,
    0.005965f,
    0.006649f,
    0.007224f,
    0.007672f,
    0.007977f,
    0.008126f,
    0.008109f,
    0.00792f,
    0.007556f,
    0.007018f,
    0.006311f,
    0.005446f,
    0.004435f,
    0.003295f,
    0.002047f,
    0.000715f,
    -0.000676f,
    -0.002096f,
    -0.003515f,
    -0.004902f,
    -0.006227f,
    -0.00746f,
    -0.00857f,
    -0.009531f,
    -0.010319f,
    -0.010912f,
    -0.011294f,
    -0.011452f,
    -0.011378f,
    -0.011068f,
    -0.010526f,
    -0.009759f,
    -0.008779f,
    -0.007604f,
    -0.006257f,
    -0.004763f,
    -0.003153f,
    -0.00146f,
    0.000282f,
    0.002035f,
    0.003763f,
    0.005429f,
    0.006996f,
    0.008432f,
    0.009704f,
    0.010785f,
    0.011652f,
    0.012285f,
    0.012671f,
    0.5f * 0.0128f  // centre tap, halved
  };

  static const float __not_in_flash_func(bpf_700f)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_sym<255u, FIR_LENGTH>(s, sample, h_bpf_700f);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_fs4f_rx1[64] =
  {
    // 31250
    // Taps: 255 (computationally 128 since cutoff is Fs/4)
    // Att: 45 (60dB+)
    -0.000051f,
    0.000066f,
    -0.000082f,
    0.000101f,
    -0.000122f,
    0.000146f,
    -0.000172f,
    0.000201f,
    -0.000233f,
    0.000268f,
    -0.000307f,
    0.000349f,
    -0.000396f,
    0.000446f,
    -0.000501f,
    0.00056f,
    -0.000624f,
    0.000693f,
    -0.000768f,
    0.000848f,
    -0.000934f,
    0.001027f,
    -0.001126f,
    0.001232f,
    -0.001346f,
    0.001468f,
    -0.001598f,
    0.001737f,
    -0.001885f,
    0.002044f,
    -0.002213f,
    0.002393f,
    -0.002586f,
    0.002792f,
    -0.003013f,
    0.003249f,
    -0.003501f,
    0.003772f,
    -0.004063f,
    0.004376f,
    -0.004714f,
    0.005079f,
    -0.005475f,
    0.005905f,
    -0.006376f,
    0.006892f,
    -0.007461f,
    0.008091f,
    -0.008796f,
    0.009588f,
    -0.010487f,
    0.011519f,
    -0.012716f,
    0.014126f,
    -0.015815f,
    0.01788f,
    -0.020473f,
    0.023835f,
    -0.028385f,
    0.034915f,
    -0.04512f,
    0.06341f,
    -0.105952f,
    0.318259f
  };

  static const float __not_in_flash_func(lpf_fs4f_rx1)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_halfband<255u, FIR_LENGTH>(s, sample, h_lpf_fs4f_rx1, 0.5f);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_fs4f_rx2[64] =
  {
    // 31250
    // Taps: 255 (computationally 128 since cutoff is Fs/4)
    // Att: 45 (60dB+)
    -0.000051f,
    0.000066f,
    -0.000082f,
    0.000101f,
    -0.000122f,
    0.000146f,
    -0.000172f,
    0.000201f,
    -0.000233f,
    0.000268f,
    -0.000307f,
    0.000349f,
    -0.000396f,
    0.000446f,
    -0.000501f,
    0.00056f,
    -0.000624f,
    0.000693f,
    -0.000768f,
    0.000848f,
    -0.000934f,
    0.001027f,
    -0.001126f,
    0.001232f,
    -0.001346f,
    0.001468f,
    -0.001598f,
    0.001737f,
    -0.001885f,
    0.002044f,
    -0.002213f,
    0.002393f,
    -0.002586f,
    0.002792f,
    -0.003013f,
    0.003249f,
    -0.003501f,
    0.003772f,
    -0.004063f,
    0.004376f,
    -0.004714f,
    0.005079f,
    -0.005475f,
    0.005905f,
    -0.006376f,
    0.006892f,
    -0.007461f,
    0.008091f,
    -0.008796f,
    0.009588f,
    -0.010487f,
    0.011519f,
    -0.012716f,
    0.014126f,
    -0.015815f,
    0.01788f,
    -0.020473f,
    0.023835f,
    -0.028385f,
    0.034915f,
    -0.04512f,
    0.06341f,
    -0.105952f,
    0.318259f
  };

  static const float __not_in_flash_func(lpf_fs4f_rx2)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_halfband<255u, FIR_LENGTH>(s, sample, h_lpf_fs4f_rx2, 0.5f);
  }

  static const float __not_in_flash("fast_access_sram") h_hpf_fs4f_rx1[64] =
  {
    // 31250
    // Taps: 255 (computationally 128 since cutoff is Fs/4)
    // Att: 45 (60dB+)
    0.000051f,
    -0.000066f,
    0.000082f,
    -0.000101f,
    0.000122f,
    -0.000146f,
    0.000172f,
    -0.000201f,
    0.000233f,
    -0.000268f,
    0.000307f,
    -0.000349f,
    0.000396f,
    -0.000446f,
    0.000501f,
    -0.00056f,
    0.000624f,
    -0.000693f,
    0.000768f,
    -0.000848f,
    0.000934f,
    -0.001027f,
    0.001126f,
    -0.001232f,
    0.001346f,
    -0.001468f,
    0.001598f,
    -0.001737f,
    0.001885f,
    -0.002044f,
    0.002213f,
    -0.002393f,
    0.002586f,
    -0.002792f,
    0.003013f,
    -0.003249f,
    0.003501f,
    -0.003772f,
    0.004063f,
    -0.004376f,
    0.004714f,
    -0.005079f,
    0.005475f,
    -0.005905f,
    0.006376f,
    -0.006892f,
    0.007461f,
    -0.008091f,
    0.008796f,
    -0.009588f,
    0.010487f,
    -0.011519f,
    0.012716f,
    -0.014126f,
    0.015815f,
    -0.01788f,
    0.020473f,
    -0.023835f,
    0.028385f,
    -0.034915f,
    0.04512f,
    -0.06341f,
    0.105952f,
    -0.318259f
  };

  static const float __not_in_flash_func(hpf_fs4f_rx1)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_halfband<255u, FIR_LENGTH>(s, sample, h_hpf_fs4f_rx1, 0.5f);
  }

  static const float __not_in_flash("fast_access_sram") h_hpf_fs4f_rx2[64] =
  {
    // 31250
    // Taps: 255 (computationally 128 since cutoff is Fs/4)
    // Att: 45 (60dB+)
    0.000051f,
    -0.000066f,
    0.000082f,
    -0.000101f,
    0.000122f,
    -0.000146f,
    0.000172f,
    -0.000201f,
    0.000233f,
    -0.000268f,
    0.000307f,
    -0.000349f,
    0.000396f,
    -0.000446f,
    0.000501f,
    -0.00056f,
    0.000624f,
    -0.000693f,
    0.000768f,
    -0.000848f,
    0.000934f,
    -0.001027f,
    0.001126f,
    -0.001232f,
    0.001346f,
    -0.001468f,
    0.001598f,
    -0.001737f,
    0.001885f,
    -0.002044f,
    0.002213f,
    -0.002393f,
    0.002586f,
    -0.002792f,
    0.003013f,
    -0.003249f,
    0.003501f,
    -0.003772f,
    0.004063f,
    -0.004376f,
    0.004714f,
    -0.005079f,
    0.005475f,
    -0.005905f,
    0.006376f,
    -0.006892f,
    0.007461f,
    -0.008091f,
    0.008796f,
    -0.009588f,
    0.010487f,
    -0.011519f,
    0.012716f,
    -0.014126f,
    0.015815f,
    -0.01788f,
    0.020473f,
    -0.023835f,
    0.028385f,
    -0.034915f,
    0.04512f,
    -0.06341f,
    0.105952f,
    -0.318259f
  };

  static const float __not_in_flash_func(hpf_fs4f_rx2)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_halfband<255u, FIR_LENGTH>(s, sample, h_hpf_fs4f_rx2, 0.5f);
  }

  static const float __not_in_flash("fast_access_sram") h_hpf_fs8f_rx1[128] =
  {
    // 31250
    // Taps: 255
    // HPF: 3906Hz
    // Att: 60
    0.000036f,
    0.000058f,
    0.000046f,
    0.0f,
    -0.000058f,
    -0.000091f,
    -0.000071f,
    0.0f,
    0.000086f,
    0.000134f,
    0.000103f,
    0.0f,
    -0.000122f,
    -0.000186f,
    -0.000142f,
    0.0f,
    0.000165f,
    0.00025f,
    0.00019f,
    0.0f,
    -0.000217f,
    -0.000328f,
    -0.000247f,
    0.0f,
    0.00028f,
    0.00042f,
    0.000315f,
    0.0f,
    -0.000354f,
    -0.00053f,
    -0.000396f,
    0.0f,
    0.000441f,
    0.000658f,
    0.00049f,
    0.0f,
    -0.000543f,
    -0.000807f,
    -0.0006f,
    0.0f,
    0.000661f,
    0.00098f,
    0.000726f,
    0.0f,
    -0.000796f,
    -0.001178f,
    -0.000871f,
    0.0f,
    0.000952f,
    0.001406f,
    0.001038f,
    0.0f,
    -0.00113f,
    -0.001666f,
    -0.001228f,
    0.0f,
    0.001333f,
    0.001963f,
    0.001445f,
    0.0f,
    -0.001565f,
    -0.002302f,
    -0.001692f,
    0.0f,
    0.001829f,
    0.002688f,
    0.001975f,
    0.0f,
    -0.00213f,
    -0.003129f,
    -0.002297f,
    0.0f,
    0.002476f,
    0.003634f,
    0.002667f,
    0.0f,
    -0.002873f,
    -0.004217f,
    -0.003094f,
    0.0f,
    0.003333f,
    0.004893f,
    0.003591f,
    0.0f,
    -0.003871f,
    -0.005685f,
    -0.004176f,
    0.0f,
    0.004508f,
    0.006628f,
    0.004873f,
    0.0f,
    -0.005275f,
    -0.007768f,
    -0.005722f,
    0.0f,
    0.00622f,
    0.00918f,
    0.00678f,
    0.0f,
    -0.007416f,
    -0.010985f,
    -0.008145f,
    0.0f,
    0.008992f,
    0.013391f,
    0.009988f,
    0.0f,
    -0.011183f,
    -0.016793f,
    -0.012643f,
    0.0f,
    0.014477f,
    0.022037f,
    0.016854f,
    0.0f,
    -0.020071f,
    -0.031329f,
    -0.024689f,
    0.0f,
    0.031905f,
    0.052749f,
    0.044837f,
    0.0f,
    -0.074919f,
    -0.159054f,
    -0.225043f,
    0.5f * 0.75f  // centre tap, halved
  };

  static const float __not_in_flash_func(hpf_fs8f_rx1)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_sym<255u, FIR_LENGTH>(s, sample, h_hpf_fs8f_rx1);
  }

  static const float __not_in_flash("fast_access_sram") h_hpf_fs8f_rx2[128] =
  {
    // 31250
    // Taps: 255
    // HPF: 3906Hz
    // Att: 60
    0.000036f,
    0.000058f,
    0.000046f,
    0.0f,
    -0.000058f,
    -0.000091f,
    -0.000071f,
    0.0f,
    0.000086f,
    0.000134f,
    0.000103f,
    0.0f,
    -0.000122f,
    -0.000186f,
    -0.000142f,
    0.0f,
    0.000165f,
    0.00025f,
    0.00019f,
    0.0f,
    -0.000217f,
    -0.000328f,
    -0.000247f,
    0.0f,
    0.00028f,
    0.00042f,
    0.000315f,
    0.0f,
    -0.000354f,
    -0.00053f,
    -0.000396f,
    0.0f,
    0.000441f,
    0.000658f,
    0.00049f,
    0.0f,
    -0.000543f,
    -0.000807f,
    -0.0006f,
    0.0f,
    0.000661f,
    0.00098f,
    0.000726f,
    0.0f,
    -0.000796f,
    -0.001178f,
    -0.000871f,
    0.0f,
    0.000952f,
    0.001406f,
    0.001038f,
    0.0f,
    -0.00113f,
    -0.001666f,
    -0.001228f,
    0.0f,
    0.001333f,
    0.001963f,
    0.001445f,
    0.0f,
    -0.001565f,
    -0.002302f,
    -0.001692f,
    0.0f,
    0.001829f,
    0.002688f,
    0.001975f,
    0.0f,
    -0.00213f,
    -0.003129f,
    -0.002297f,
    0.0f,
    0.002476f,
    0.003634f,
    0.002667f,
    0.0f,
    -0.002873f,
    -0.004217f,
    -0.003094f,
    0.0f,
    0.003333f,
    0.004893f,
    0.003591f,
    0.0f,
    -0.003871f,
    -0.005685f,
    -0.004176f,
    0.0f,
    0.004508f,
    0.006628f,
    0.004873f,
    0.0f,
    -0.005275f,
    -0.007768f,
    -0.005722f,
    0.0f,
    0.00622f,
    0.00918f,
    0.00678f,
    0.0f,
    -0.007416f,
    -0.010985f,
    -0.008145f,
    0.0f,
    0.008992f,
    0.013391f,
    0.009988f,
    0.0f,
    -0.011183f,
    -0.016793f,
    -0.012643f,
    0.0f,
    0.014477f,
    0.022037f,
    0.016854f,
    0.0f,
    -0.020071f,
    -0.031329f,
    -0.024689f,
    0.0f,
    0.031905f,
    0.052749f,
    0.044837f,
    0.0f,
    -0.074919f,
    -0.159054f,
    -0.225043f,
    0.5f * 0.75f  // centre tap, halved
  };

  static const float __not_in_flash_func(hpf_fs8f_rx2)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_sym<255u, FIR_LENGTH>(s, sample, h_hpf_fs8f_rx2);
  }

  static const float __not_in_flash("fast_access_sram") h_hpf_200f[128] =
  {
    // 31250
    // att: 60dB
    // Lo: 300
    0.000047f,
    0.000055f,
    0.000062f,
    0.000071f,
    0.00008f,
    0.00009f,
    0.0001f,
    0.000111f,
    0.000122f,
    0.000134f,
    0.000146f,
    0.000158f,
    0.000171f,
    0.000185f,
    0.000198f,
    0.000212f,
    0.000226f,
    0.00024f,
    0.000254f,
    0.000268f,
    0.000282f,
    0.000295f,
    0.000308f,
    0.000321f,
    0.000333f,
    0.000344f,
    0.000355f,
    0.000364f,
    0.000373f,
    0.00038f,
    0.000385f,
    0.000389f,
    0.000392f,
    0.000392f,
    0.00039f,
    0.000386f,
    0.00038f,
    0.000371f,
    0.000359f,
    0.000344f,
    0.000326f,
    0.000305f,
    0.00028f,
    0.000252f,
    0.000219f,
    0.000183f,
    0.000142f,
    0.000097f,
    0.000047f,
    -0.000007f,
    -0.000066f,
    -0.000131f,
    -0.0002f,
    -0.000275f,
    -0.000355f,
    -0.000441f,
    -0.000533f,
    -0.00063f,
    -0.000733f,
    -0.000842f,
    -0.000957f,
    -0.001078f,
    -0.001205f,
    -0.001339f,
    -0.001478f,
    -0.001623f,
    -0.001775f,
    -0.001932f,
    -0.002095f,
    -0.002265f,
    -0.00244f,
    -0.00262f,
    -0.002806f,
    -0.002998f,
    -0.003195f,
    -0.003397f,
    -0.003604f,
    -0.003815f,
    -0.004031f,
    -0.004252f,
    -0.004476f,
    -0.004704f,
    -0.004935f,
    -0.005169f,
    -0.005406f,
    -0.005646f,
    -0.005887f,
    -0.006131f,
    -0.006376f,
    -0.006622f,
    -0.006868f,
    -0.007115f,
    -0.007361f,
    -0.007607f,
    -0.007852f,
    -0.008096f,
    -0.008338f,
    -0.008577f,
    -0.008814f,
    -0.009047f,
    -0.009278f,
    -0.009504f,
    -0.009726f,
    -0.009943f,
    -0.010154f,
    -0.010361f,
    -0.010561f,
    -0.010754f,
    -0.010941f,
    -0.011121f,
    -0.011293f,
    -0.011458f,
    -0.011614f,
    -0.011761f,
    -0.0119f,
    -0.01203f,
    -0.01215f,
    -0.012261f,
    -0.012362f,
    -0.012453f,
    -0.012534f,
    -0.012604f,
    -0.012664f,
    -0.012713f,
    -0.012751f,
    -0.012778f,
    -0.012795f,
    0.5f * 0.9872f  // centre tap, halved
  };

  static const float __not_in_flash_func(hpf_200f)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_sym<255u, FIR_LENGTH>(s, sample, h_hpf_200f);
  }

  static const float __not_in_flash("fast_access_sram") h_hpf_300f[128] =
  {
    // 31250
    // att: 60dB
    // Lo: 300
    -0.00005f,
    -0.000056f,
    -0.000062f,
    -0.000069f,
    -0.000075f,
    -0.00008f,
    -0.000086f,
    -0.000091f,
    -0.000095f,
    -0.000099f,
    -0.000102f,
    -0.000104f,
    -0.000105f,
    -0.000104f,
    -0.000102f,
    -0.000099f,
    -0.000093f,
    -0.000086f,
    -0.000077f,
    -0.000066f,
    -0.000052f,
    -0.000036f,
    -0.000018f,
    0.000004f,
    0.000028f,
    0.000055f,
    0.000085f,
    0.000118f,
    0.000154f,
    0.000193f,
    0.000235f,
    0.00028f,
    0.000328f,
    0.000379f,
    0.000432f,
    0.000489f,
    0.000548f,
    0.000609f,
    0.000672f,
    0.000737f,
    0.000803f,
    0.000871f,
    0.00094f,
    0.001009f,
    0.001078f,
    0.001146f,
    0.001214f,
    0.00128f,
    0.001344f,
    0.001406f,
    0.001464f,
    0.001519f,
    0.00157f,
    0.001615f,
    0.001654f,
    0.001688f,
    0.001714f,
    0.001732f,
    0.001742f,
    0.001742f,
    0.001733f,
    0.001713f,
    0.001682f,
    0.001639f,
    0.001583f,
    0.001514f,
    0.001431f,
    0.001333f,
    0.001221f,
    0.001093f,
    0.000949f,
    0.000789f,
    0.000613f,
    0.000419f,
    0.000208f,
    -0.00002f,
    -0.000265f,
    -0.000528f,
    -0.000809f,
    -0.001107f,
    -0.001423f,
    -0.001755f,
    -0.002105f,
    -0.00247f,
    -0.002852f,
    -0.003249f,
    -0.00366f,
    -0.004086f,
    -0.004525f,
    -0.004977f,
    -0.00544f,
    -0.005914f,
    -0.006398f,
    -0.00689f,
    -0.007389f,
    -0.007895f,
    -0.008405f,
    -0.008919f,
    -0.009436f,
    -0.009953f,
    -0.01047f,
    -0.010985f,
    -0.011496f,
    -0.012002f,
    -0.012503f,
    -0.012995f,
    -0.013478f,
    -0.01395f,
    -0.014409f,
    -0.014856f,
    -0.015287f,
    -0.015701f,
    -0.016098f,
    -0.016475f,
    -0.016833f,
    -0.017169f,
    -0.017482f,
    -0.017772f,
    -0.018037f,
    -0.018277f,
    -0.01849f,
    -0.018676f,
    -0.018835f,
    -0.018966f,
    -0.019068f,
    -0.019141f,
    -0.019185f,
    0.5f * 0.9808f  // centre tap, halved
  };

  static const float __not_in_flash_func(hpf_300f)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_sym<255u, FIR_LENGTH>(s, sample, h_hpf_300f);
  }

  static const float __not_in_flash("fast_access_sram") h_bpf_45p[256] =
  {
    // 31250
    // BPF: (FS/4) +45
    // att: 60dB
    // 255 taps
    4.07409463058283E-06f,
    -0.000016447161466566f,
    0.000022953003442011f,
    0.000014226267809262f,
    -1.9308012691556E-07f,
    7.50163542366419E-06f,
    -5.36677390485297E-06f,
    4.88565845026947E-12f,
    6.50356490489765E-07f,
    9.24929202087309E-06f,
    -0.000017236787889192f,
    -0.000012805240587691f,
    -7.14094120583584E-06f,
    -0.000031035478366678f,
    0.000039356634284719f,
    0.000020154464297031f,
    0.000018803496895677f,
    0.000052306364517098f,
    -0.000054728199016244f,
    -0.000019902734290428f,
    -0.000031763056643104f,
    -0.000065869769066205f,
    0.000058237173713207f,
    0.000012963804204417f,
    0.000039722602228999f,
    0.000064887906828396f,
    -0.000047795065221122f,
    -3.5399838142591E-06f,
    -0.00003566539866283f,
    -0.000045254984655269f,
    0.000025521506452647f,
    -1.89724918957767E-06f,
    0.000014361740142403f,
    7.64752810049425E-06f,
    2.32514351609675E-06f,
    -3.1771253586531E-06f,
    0.000024975122877038f,
    0.000041512918954899f,
    -0.000026925702654352f,
    0.000022410821466843f,
    -0.000076753659723108f,
    -0.000090762244179198f,
    0.000039790887649232f,
    -0.000053779760565252f,
    0.000128644712259505f,
    0.000126131238129961f,
    -0.000036276045724052f,
    0.000087968037094479f,
    -0.000163367596155609f,
    -0.000135134354937359f,
    0.000018639337610226f,
    -0.000108868583755555f,
    0.000162664267095115f,
    0.000111262332782874f,
    2.59057437753073E-06f,
    0.000096751549230758f,
    -0.000112842531833041f,
    -0.000057619542634549f,
    -9.99507316380037E-06f,
    -0.000033995612513749f,
    0.000010635714828155f,
    -0.000011767195288453f,
    -0.000016065769221918f,
    -0.000087490885193291f,
    0.00013229761948811f,
    0.000074173197159633f,
    0.000089733335878522f,
    0.000258725806258158f,
    -0.000287972250218641f,
    -0.000104496988465305f,
    -0.00020998251491091f,
    -0.000447970237586237f,
    0.000415136208319498f,
    0.000085341598336639f,
    0.000350840976911669f,
    0.000598872819619812f,
    -0.000467376572398773f,
    -0.000019572305309238f,
    -0.000453701180615891f,
    -0.000633886047555917f,
    0.000405827586543635f,
    -0.000056661667430992f,
    0.000423759739373851f,
    0.000463426445710557f,
    -0.00021501623363613f,
    0.000062686581769438f,
    -0.000131816046312957f,
    -1.2641060283446E-10f,
    -0.000080308218556568f,
    0.000134426742364081f,
    -0.000578772501836847f,
    -0.00082474804523782f,
    0.00039950172332052f,
    -0.000723868810014806f,
    0.001882223437617f,
    0.00204308056144851f,
    -0.000592489993089653f,
    0.00195407210492057f,
    -0.00396260627504707f,
    -0.00363775566971369f,
    0.00042626475898081f,
    -0.00414281854800644f,
    0.00701784116616339f,
    0.00553532539445395f,
    0.000437759300565252f,
    0.00771440756196165f,
    -0.0112948662670658f,
    -0.00760831536426848f,
    -0.00250272776307766f,
    -0.0133157714413619f,
    0.0172045152290328f,
    0.00968690507154228f,
    0.00661603505066427f,
    0.0221740299460456f,
    -0.0256744909459179f,
    -0.0115789114951242f,
    -0.0145926361758226f,
    -0.0373643898563604f,
    0.0394518545563261f,
    0.0130951760595795f,
    0.0320635855934223f,
    0.0702037273074379f,
    -0.0706130922333324f,
    -0.0140762357081225f,
    -0.0942881847979189f,
    -0.22338799567117f,
    0.327834999805385f,
    0.367889093242598f,
    -0.327834999567796f,
    -0.223387995953426f,
    0.0942881847003544f,
    -0.014076235874499f,
    0.0706130919383048f,
    0.0702037274917512f,
    -0.0320635856698966f,
    0.0130951763344111f,
    -0.039451854306222f,
    -0.0373643898786103f,
    0.0145926363996961f,
    -0.0115789117830016f,
    0.02567449082765f,
    0.0221740297985223f,
    -0.00661603534417325f,
    0.0096869052724847f,
    -0.0172045152837083f,
    -0.0133157711753357f,
    0.00250272802428804f,
    -0.00760831540847276f,
    0.011294866475703f,
    0.0077144072698581f,
    -0.000437759438762489f,
    0.00553532526655487f,
    -0.00701784145629241f,
    -0.0041428183313026f,
    -0.000426264791783071f,
    -0.00363775541412275f,
    0.00396260654588177f,
    0.00195407203889225f,
    0.000592490185487811f,
    0.00204308026693471f,
    -0.00188222359506996f,
    -0.000723868917591631f,
    -0.000399502008483249f,
    -0.000824747814097347f,
    0.000578772491207143f,
    0.000134426986169875f,
    0.000080308497436607f,
    -2.13886825891296E-10f,
    0.000131816221334766f,
    0.00006268628642775f,
    0.000215016057902944f,
    0.00046342635913026f,
    -0.000423760017988074f,
    -0.000056661423141535f,
    -0.000405827574997402f,
    -0.000633885816989056f,
    0.000453701466038971f,
    -0.000019572413693707f,
    0.000467376729050333f,
    0.000598872525152466f,
    -0.000350841169993883f,
    0.000085341533174916f,
    -0.000415136478761259f,
    -0.000447969981508661f,
    0.000209982548585627f,
    -0.000104496772392718f,
    0.000287972540513952f,
    0.000258725677526313f,
    -0.000089733198455534f,
    0.000074172905222087f,
    -0.000132297828789737f,
    -0.000087490928509013f,
    0.000016065508417582f,
    -0.000011766928916433f,
    -0.000010635659194236f,
    -0.000033995412164415f,
    9.99536672369551E-06f,
    -0.000057619690946321f,
    0.000112842649218884f,
    0.000096751261461196f,
    -2.59079873540673E-06f,
    0.000111262311511683f,
    -0.000162664516726038f,
    -0.000108868308528231f,
    -0.000018639260382797f,
    -0.000135134171469443f,
    0.000163367891320697f,
    0.000087967870015105f,
    0.000036276142447153f,
    0.000126130956188193f,
    -0.000128644950425561f,
    -0.000053779759666917f,
    -0.000039791124726107f,
    -0.000090761961696934f,
    0.00007675375815881f,
    0.000022410987059232f,
    0.000026925997735658f,
    0.000041512734053983f,
    -0.000024975047387659f,
    -3.17739991856813E-06f,
    -2.32539410647041E-06f,
    7.64755119075085E-06f,
    -0.000014361963334134f,
    -1.89696103358086E-06f,
    -0.000025521387395449f,
    -0.000045254837911406f,
    0.000035665692027033f,
    -3.54018547432699E-06f,
    0.000047795119052153f,
    0.000064887641221142f,
    -0.000039722863860513f,
    0.000012963849323122f,
    -0.000058237381735718f,
    -0.000065869476856742f,
    0.000031763195661013f,
    -0.000019902607197749f,
    0.000054728488974741f,
    0.000052306147209124f,
    -0.000018803465012576f,
    0.000020154209139093f,
    -0.00003935690546012f,
    -0.00003103541144438f,
    0.000007140749496755f,
    -0.000012804945996324f,
    0.000017236946088449f,
    9.24939873319727E-06f,
    -6.50071550796398E-07f,
    -2.26814406336466E-10f,
    5.36678363689693E-06f,
    7.50139214627108E-06f,
    1.92800932301732E-07f,
    0.000014226356137083f,
    -0.00002295317772915f,
    -0.000016446866134947f,
    -4.07391815314473E-06f,
    0.0f  // padding
  };

  static const float __not_in_flash_func(bpf_45p)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_gen<255u, FIR_LENGTH>(s, sample, h_bpf_45p);
  }

  static const float __not_in_flash("fast_access_sram") h_bpf_45n[256] =
  {
    // 31250
    // BPF: (FS/4) -45
    // att: 60dB
    // 255 taps
    -4.07391816150334E-06f,
    -0.000016446866129471f,
    -0.000022953177727021f,
    0.00001422635613645f,
    1.92800934214582E-07f,
    7.50139214548739E-06f,
    5.36678363664589E-06f,
    -2.26814652799522E-10f,
    -6.50071552699166E-07f,
    9.24939873328308E-06f,
    0.000017236946088811f,
    -0.000012804945995372f,
    7.14074949888353E-06f,
    -0.000031035411445067f,
    -0.000039356905462125f,
    0.000020154209138908f,
    -0.000018803465012323f,
    0.000052306147208789f,
    0.000054728488975471f,
    -0.000019902607196171f,
    0.000031763195660438f,
    -0.000065869476857938f,
    -0.00005823738173595f,
    0.000012963849322291f,
    -0.000039722863859492f,
    0.000064887641223081f,
    0.000047795119051534f,
    -3.54018547505163E-06f,
    0.000035665692026298f,
    -0.000045254837912633f,
    -0.000025521387394436f,
    -1.89696103188111E-06f,
    -0.000014361963333992f,
    7.64755119035112E-06f,
    -2.32539410716614E-06f,
    -3.17739991983753E-06f,
    -0.000024975047387605f,
    0.000041512734055767f,
    0.000026925997736026f,
    0.000022410987058652f,
    0.000076753758158501f,
    -0.000090761961698407f,
    -0.000039791124725704f,
    -0.000053779759664739f,
    -0.000128644950425538f,
    0.000126130956188401f,
    0.000036276142446189f,
    0.000087967870011344f,
    0.000163367891320603f,
    -0.00013513417146605f,
    -0.000018639260379303f,
    -0.000108868308526596f,
    -0.000162664516729883f,
    0.000111262311507467f,
    -2.59079873777605E-06f,
    0.00009675126146135f,
    0.000112842649226482f,
    -0.000057619690942551f,
    9.99536672242894E-06f,
    -0.000033995412167137f,
    -0.000010635659206602f,
    -0.000011766928911905f,
    0.000016065508434848f,
    -0.000087490928522284f,
    -0.000132297828799506f,
    0.000074172905238675f,
    -0.00008973319845253f,
    0.000258725677519932f,
    0.000287972540510939f,
    -0.000104496772397661f,
    0.000209982548587111f,
    -0.000447969981503573f,
    -0.000415136478757944f,
    0.00008534153317528f,
    -0.000350841169996779f,
    0.000598872525150087f,
    0.000467376729048505f,
    -0.000019572413692678f,
    0.000453701466040881f,
    -0.000633885816987741f,
    -0.000405827574995886f,
    -0.00005666142314427f,
    -0.000423760017990084f,
    0.000463426359131205f,
    0.000215016057903635f,
    0.000062686286429081f,
    0.000131816221335095f,
    -2.13887590224572E-10f,
    0.000080308497435195f,
    0.00013442698617052f,
    0.00057877249120686f,
    -0.000824747814097807f,
    -0.000399502008480938f,
    -0.000723868917595513f,
    -0.00188222359506767f,
    0.00204308026693743f,
    0.000592490185483652f,
    0.00195407203890067f,
    0.00396260654587572f,
    -0.00363775541412819f,
    -0.000426264791776773f,
    -0.00414281833131685f,
    -0.00701784145628023f,
    0.00553532526656182f,
    -0.000437759438770428f,
    0.00771440726988091f,
    0.0112948664756809f,
    -0.00760831540847884f,
    0.00250272802429672f,
    -0.0133157711753718f,
    -0.0172045152836704f,
    0.00968690527248586f,
    -0.00661603534418339f,
    0.0221740297985817f,
    0.0256744908275914f,
    -0.0115789117829962f,
    0.0145926363997025f,
    -0.0373643898787049f,
    -0.039451854306127f,
    0.0130951763343876f,
    -0.0320635856698929f,
    0.0702037274919232f,
    0.0706130919381294f,
    -0.014076235874418f,
    0.0942881847003264f,
    -0.223387995954034f,
    -0.327834999567238f,
    0.367889093243337f,
    0.327834999804629f,
    -0.223387995671459f,
    -0.0942881847975693f,
    -0.0140762357082604f,
    -0.0706130922332246f,
    0.0702037273075478f,
    0.0320635855932978f,
    0.0130951760596562f,
    0.0394518545562623f,
    -0.0373643898564164f,
    -0.0145926361757574f,
    -0.0115789114951762f,
    -0.0256744909458732f,
    0.0221740299460759f,
    0.00661603505062692f,
    0.00968690507157944f,
    0.0172045152290022f,
    -0.0133157714413792f,
    -0.00250272776305869f,
    -0.00760831536429251f,
    -0.0112948662670431f,
    0.00771440756196942f,
    0.000437759300555845f,
    0.00553532539446942f,
    0.00701784116614729f,
    -0.004142818548009f,
    0.000426264758985485f,
    -0.00363775566972297f,
    -0.00396260627503682f,
    0.00195407210492074f,
    -0.000592489993092321f,
    0.00204308056145326f,
    0.00188222343761209f,
    -0.000723868810014055f,
    0.000399501723321945f,
    -0.000824748045239669f,
    -0.000578772501835875f,
    0.00013442674236304f,
    -0.000080308218556993f,
    -1.26409619255088E-10f,
    -0.000131816046311656f,
    0.000062686581769644f,
    -0.000215016233636615f,
    0.000463426445709232f,
    0.000423759739371629f,
    -0.000056661667428751f,
    0.000405827586545969f,
    -0.000633886047555792f,
    -0.000453701180615995f,
    -0.000019572305311949f,
    -0.000467376572400704f,
    0.000598872819619366f,
    0.000350840976913646f,
    0.00008534159834073f,
    0.000415136208321808f,
    -0.000447970237587259f,
    -0.000209982514917342f,
    -0.000104496988468247f,
    -0.000287972250218353f,
    0.000258725806260703f,
    0.000089733335891307f,
    0.000074173197154169f,
    0.000132297619471526f,
    -0.000087490885178879f,
    -0.000016065769213396f,
    -0.00001176719530513f,
    0.000010635714825534f,
    -0.000033995612508122f,
    -9.99507316019907E-06f,
    -0.000057619542628459f,
    -0.00011284253183546f,
    0.000096751549224827f,
    2.59057437473955E-06f,
    0.000111262332781511f,
    0.000162664267098863f,
    -0.000108868583751086f,
    0.000018639337611363f,
    -0.000135134354939094f,
    -0.000163367596159932f,
    0.000087968037093639f,
    -0.000036276045721905f,
    0.000126131238130515f,
    0.000128644712260874f,
    -0.000053779760564932f,
    0.000039790887647083f,
    -0.000090762244179546f,
    -0.000076753659722653f,
    0.000022410821467185f,
    -0.000026925702652975f,
    0.000041512918954646f,
    0.000024975122875302f,
    -3.17712535909833E-06f,
    2.32514351659551E-06f,
    7.64752810109856E-06f,
    0.000014361740143609f,
    -1.89724918907384E-06f,
    0.000025521506450926f,
    -0.000045254984656379f,
    -0.000035665398662506f,
    -3.53998381420356E-06f,
    -0.000047795065219542f,
    0.000064887906829411f,
    0.000039722602227339f,
    0.000012963804203715f,
    0.000058237173712956f,
    -0.000065869769066431f,
    -0.000031763056641425f,
    -0.000019902734289667f,
    -0.000054728199017268f,
    0.000052306364516693f,
    0.000018803496895534f,
    0.000020154464295883f,
    0.000039356634284546f,
    -0.000031035478364245f,
    -7.14094120481181E-06f,
    -0.000012805240588767f,
    -0.000017236787889684f,
    9.24929201931939E-06f,
    6.50356490080239E-07f,
    4.88684594538429E-12f,
    -5.36677390515326E-06f,
    7.50163542525781E-06f,
    -1.9308012630772E-07f,
    0.000014226267808631f,
    0.00002295300344488f,
    -0.000016447161472099f,
    4.07409462305342E-06f,
    0.0f  // padding
  };

  static const float __not_in_flash_func(bpf_45n)(const float sample)
  {
    static fir_state_t<FIR_LENGTH> s = {};
    return fir_gen<255u, FIR_LENGTH>(s, sample, h_bpf_45n);
  }

  static const float __not_in_flash("fast_access_sram") h_lpf_3000f[64] =
  {
    // 31250
    // LPF: 3000 Hz
    // att: 60dB
    // 127 taps
    0.000031f,
    -0.000039f,
    -0.000131f,
    -0.000204f,
    -0.000212f,
    -0.000123f,
    0.000061f,
    0.000287f,
    0.000466f,
    0.0005f,
    0.000329f,
    -0.000036f,
    -0.000491f,
    -0.000866f,
    -0.00098f,
    -0.000718f,
    -0.000096f,
    0.000714f,
    0.00142f,
    0.001711f,
    0.001377f,
    0.00042f,
    -0.000903f,
    -0.002132f,
    -0.002757f,
    -0.002412f,
    -0.001052f,
    0.000973f,
    0.002982f,
    0.00418f,
    0.003955f,
    0.00215f,
    -0.000799f,
    -0.003931f,
    -0.006058f,
    -0.006186f,
    -0.00394f,
    0.000195f,
    0.004923f,
    0.008512f,
    0.009393f,
    0.006788f,
    0.001136f,
    -0.005886f,
    -0.011803f,
    -0.014153f,
    -0.011423f,
    -0.003773f,
    0.006743f,
    0.016616f,
    0.021929f,
    0.019716f,
    0.009219f,
    -0.007418f,
    -0.025311f,
    -0.037928f,
    -0.038867f,
    -0.02382f,
    0.007851f,
    0.052454f,
    0.102496f,
    0.148319f,
    0.180451f,
    0.5f * 0.192f  // centre tap, halved
  };

  static const float __not_in_flash_func(lpf_3000f)(const float sample)
  {
    static fir_state_t<(FIR_LENGTH / 2u)> s = {};
    return fir_sym<127u, (FIR_LENGTH / 2u)>(s, sample, h_lpf_3000f);
  }

  static const float __not_in_flash_func(jnr_maf1)(const float v)
  {
    static float element[JNR_FILTER_LENGTH] = {0.0f};
    static uint8_t p = 0;
    static float sum = 0.0f;
    sum = sum - element[p] + v;
    element[p++] = v;
    p &= (JNR_FILTER_LENGTH-1);
    return sum / ((float)JNR_FILTER_LENGTH);
  }

  static const float __not_in_flash_func(jnr_maf2)(const float v)
  {
    static float element[JNR_FILTER_LENGTH] = {0.0f};
    static uint8_t p = 0;
    static float sum = 0.0f;
    sum = sum - element[p] + v;
    element[p++] = v;
    p &= (JNR_FILTER_LENGTH-1);
    return sum / ((float)JNR_FILTER_LENGTH);
  }

  static const float __not_in_flash_func(jnr_maf3)(const float v)
  {
    static float element[JNR_FILTER_LENGTH] = {0.0f};
    static uint8_t p = 0;
    static float sum = 0.0f;
    sum = sum - element[p] + v;
    element[p++] = v;
    p &= (JNR_FILTER_LENGTH-1);
    return sum / ((float)JNR_FILTER_LENGTH);
  }

  static const float __not_in_flash_func(jnr_maf4)(const float v)
  {
    static float element[JNR_FILTER_LENGTH] = {0.0f};
    static uint8_t p = 0;
    static float sum = 0.0f;
    sum = sum - element[p] + v;
    element[p++] = v;
    p &= (JNR_FILTER_LENGTH-1);
    return sum / ((float)JNR_FILTER_LENGTH);
  }

  static const float __not_in_flash_func(jnr)(const float s,const uint32_t level)
  {
    // just noise reduction (not dynamic)
    switch (level)
    {
      case 1: return jnr_maf1(s);
      case 2: return jnr_maf2(FILTER::jnr_maf1(s));
      case 3: return jnr_maf4(FILTER::jnr_maf3(FILTER::jnr_maf2(FILTER::jnr_maf1(s))));
    }
    return s;
  }

  static const float __not_in_flash_func(ma1)(const float v)
  {
    static int32_t element[ADC_FILTER_LENGTH] = {0};
    static uint8_t p = 0;
    static int32_t sum = 0.0f;
    sum = sum - element[p] + v;
    element[p++] = v;
    p &= (ADC_FILTER_LENGTH-1);
    return sum / ADC_FILTER_LENGTH;
  }

  static const float __not_in_flash_func(ma2)(const float v)
  {
    static int32_t element[ADC_FILTER_LENGTH] = {0};
    static uint8_t p = 0;
    static int32_t sum = 0.0f;
    sum = sum - element[p] + v;
    element[p++] = v;
    p &= (ADC_FILTER_LENGTH-1);
    return sum / ADC_FILTER_LENGTH;
  }

  typedef const float (*bw_func_t)(const float);
  static const bw_func_t bwf[5] =
  {
    lpf_2200f_rx, // BW: 2000Hz
    lpf_2400f_rx, // BW: 2200Hz
    lpf_2600f_rx, // BW: 2400Hz
    lpf_2800f_rx, // BW: 2600Hz
    lpf_3000f_rx  // BW: 2800Hz
  };
}

#endif