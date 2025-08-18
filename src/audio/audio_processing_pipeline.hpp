#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>

class AudioProcessingPipeline {
public:
  explicit AudioProcessingPipeline(int sample_rate_hz = 16000);
  ~AudioProcessingPipeline();

  // returns true if output contains speechy content (gate passed)
  bool process(const int16_t* in, size_t n, std::vector<int16_t>& out);

  // Feed playback reference for simple AEC (same sample-rate as process() input)
  void setPlaybackReference(const int16_t* ref, size_t n);

  void setNoiseGateRms(float rms_thresh);
  void setHighpassCutHz(float hz);

private:
  int   sr_;
  bool  filter_enabled_;
  float gate_rms_;
  float hpf_cut_hz_;

  // one-pole HPF state
  float hpf_prev_x_ = 0.0f;
  float hpf_prev_y_ = 0.0f;
  float hpf_alpha_  = 0.0f;

  // very-light AEC: short reference ring + scalar leakage
  std::vector<float> aec_ref_;
  size_t aec_ref_pos_ = 0;
  float  aec_leak_    = 0.15f;     // how much ref to subtract
  int    aec_delay_   = 2;         // frames of 10 ms (tune if needed)
  size_t aec_frame_ns_ = 0;        // samples per 10 ms

  void  updateHpfAlpha();
  void  applyHPF(const int16_t* in, size_t n, std::vector<float>& tmp);
  float computeRms(const float* x, size_t n) const;
  void  floatToPcm16(const float* x, size_t n, std::vector<int16_t>& out) const;
};
