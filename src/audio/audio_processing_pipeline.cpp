#include "audio/audio_processing_pipeline.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>

static inline float clampf(float v, float mn, float mx){ return std::max(mn, std::min(mx, v)); }

AudioProcessingPipeline::AudioProcessingPipeline(int sample_rate_hz)
: sr_(sample_rate_hz) {
  const char* env = std::getenv("AUDIO_FILTER");
  filter_enabled_ = !(env && std::string(env) == "0");
  gate_rms_ = 0.008f;        // tweak via setNoiseGateRms (≈ 260/32768)
  hpf_cut_hz_ = 120.0f;      // tweak via setHighpassCutHz
  updateHpfAlpha();

  // AEC buffers: keep ~300 ms of reference
  aec_frame_ns_ = (size_t)(sr_ / 100); // 10 ms
  aec_ref_.assign(sr_ * 3 / 10, 0.0f); // 300 ms
}

AudioProcessingPipeline::~AudioProcessingPipeline(){}

void AudioProcessingPipeline::setNoiseGateRms(float t){ gate_rms_ = t; }
void AudioProcessingPipeline::setHighpassCutHz(float hz){ hpf_cut_hz_ = hz; updateHpfAlpha(); }

void AudioProcessingPipeline::updateHpfAlpha(){
  const float rc = 1.0f / (2.0f * float(M_PI) * std::max(10.0f, hpf_cut_hz_));
  const float dt = 1.0f / float(sr_);
  float alpha_lp = dt / (rc + dt);
  hpf_alpha_ = 1.0f - alpha_lp;
}

void AudioProcessingPipeline::applyHPF(const int16_t* in, size_t n, std::vector<float>& tmp){
  tmp.resize(n);
  for (size_t i=0;i<n;i++){
    float x = in[i] / 32768.0f;
    float y = hpf_alpha_ * (hpf_prev_y_ + x - hpf_prev_x_);
    hpf_prev_x_ = x;
    hpf_prev_y_ = y;
    tmp[i] = y;
  }
}

float AudioProcessingPipeline::computeRms(const float* x, size_t n) const{
  if (n==0) return 0.0f;
  double s=0.0; for(size_t i=0;i<n;i++){ s += double(x[i])*double(x[i]); }
  return std::sqrt(float(s / double(n)));
}

void AudioProcessingPipeline::floatToPcm16(const float* x, size_t n, std::vector<int16_t>& out) const{
  out.resize(n);
  for (size_t i=0;i<n;i++){
    float v = clampf(x[i]*32768.0f, -32768.0f, 32767.0f);
    out[i] = static_cast<int16_t>(v);
  }
}

bool AudioProcessingPipeline::process(const int16_t* in, size_t n, std::vector<int16_t>& out){
  if (!in || n==0){
    out.clear();
    return false;
  }
  if (!filter_enabled_){
    out.assign(in, in+n);
    return !out.empty();
  }

  // 1) HPF
  std::vector<float> x;
  applyHPF(in, n, x);

  // 2) Very-light AEC: subtract delayed reference * leak
  if (!aec_ref_.empty()){
    size_t ref_size = aec_ref_.size();
    size_t delay_samp = (size_t)(aec_delay_ * aec_frame_ns_);
    for (size_t i=0;i<n;i++){
      size_t rp = (aec_ref_pos_ + ref_size + i - delay_samp) % ref_size;
      x[i] -= aec_leak_ * aec_ref_[rp];
    }
  }

  // 3) Gate
  float rms = computeRms(x.data(), x.size());
  if (rms < gate_rms_) {
    out.clear();
    return false;
  }

  // 4) Back to PCM
  floatToPcm16(x.data(), x.size(), out);
  return !out.empty();
}

void AudioProcessingPipeline::setPlaybackReference(const int16_t* ref, size_t n){
  // Feed the AEC ring with the current playback block (convert to float, HPF to mimic mic path)
  if (!ref || n == 0 || aec_ref_.empty()) return;
  std::vector<float> rf(n);
  for (size_t i=0;i<n;i++) rf[i] = (ref[i] / 32768.0f);
  // smear a little to simulate room
  for (size_t i=1;i<n;i++) rf[i] = 0.7f*rf[i] + 0.3f*rf[i-1];

  size_t ref_size = aec_ref_.size();
  for (size_t i=0;i<n;i++){
    aec_ref_[aec_ref_pos_] = rf[i];
    aec_ref_pos_ = (aec_ref_pos_ + 1) % ref_size;
  }
}
