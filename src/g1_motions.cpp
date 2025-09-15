#include <yaml-cpp/yaml.h>
#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <filesystem>
#include <limits>
#include <algorithm>
#include <vector>
#include <cstdlib>   // getenv, atoi, atof

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

// High-level motion RPC (keeps balance controller active)
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>

// (Kept for compatibility; no longer used to disable modes)
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

// Project Branding
#include "robot/branding/branding_fingerprint.hpp"
#include "project_branding.hpp"

#ifndef BLIB_DIR
#define BLIB_DIR std::string(PROJECT_SOURCE_DIR) + "/motion_data/"
#endif

using namespace unitree::robot::b2;   // MotionSwitcherClient namespace (still included)
using namespace unitree::common;
using namespace unitree::robot;
namespace fs = std::filesystem;

// IMPORTANT: publish to the arm-blending topic, not lowcmd (whole-body takeover).
// This prevents controller fights with the onboard stabilizer.
static const std::string HG_CMD_TOPIC    = "rt/arm_sdk";
static const std::string HG_STATE_TOPIC  = "rt/lowstate";

const int G1_NUM_MOTOR = 29; // 0..28 joints. Index 29 (special) exists only inside LowCmd_ buffer.

// Small helper buffer for thread-safe sharing
template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }
  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }
  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }
 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

struct ImuState { std::array<float, 3> rpy = {}; std::array<float, 3> omega = {}; };

struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

struct MotorState { std::array<float, G1_NUM_MOTOR> q = {}; std::array<float, G1_NUM_MOTOR> dq = {}; };

enum MotorType { GearboxS = 0, GearboxM = 1, GearboxL = 2 };
std::array<MotorType, G1_NUM_MOTOR> G1MotorType {
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    GearboxM, GearboxS, GearboxS,
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS,
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS
};

// NOTE: these enum indices match your current .seq layout and existing code.
enum G1JointValidIndex {
  LeftShoulderPitch = 15, LeftShoulderRoll = 16, LeftShoulderYaw = 17,
  LeftElbow = 18, LeftWristRoll = 19, LeftWristPitch = 20, LeftWristYaw = 21,
  RightShoulderPitch = 22, RightShoulderRoll = 23, RightShoulderYaw = 24,
  RightElbow = 25, RightWristRoll = 26, RightWristPitch = 27, RightWristYaw = 28
};

float GetMotorKp(MotorType type) { return (type == GearboxL) ? 100 : 40; }
float GetMotorKd(MotorType type) { return 1; }

// CRC (unchanged)
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit, data, CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      CRC32 = (CRC32 & 0x80000000) ? (CRC32 << 1) ^ dwPolynomial : (CRC32 << 1);
      if (data & xbit) CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
}

// ========== Motion structure ==========
struct Motion {
  std::string file_name;
  std::vector<std::string> triggers;
  double fuzzy_threshold = 0.75;
};

std::vector<Motion> motionRegistry;

// Load motion registry from YAML (unchanged)
bool loadMotionRegistry(const std::string &yamlPath) {
  try {
    YAML::Node root = YAML::LoadFile(yamlPath);
    motionRegistry.clear();
    for (const auto &motionNode : root["motions"]) {
      Motion motion;
      motion.file_name = motionNode["file"].as<std::string>();
      for (const auto &t : motionNode["triggers"]) motion.triggers.push_back(t.as<std::string>());
      motion.fuzzy_threshold = motionNode["threshold"] ? motionNode["threshold"].as<double>() : 0.75;
      motionRegistry.push_back(motion);
    }
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Error loading motions YAML: " << e.what() << "\n";
    return false;
  }
}

// ================== Robot Example ===================
class G1Example {
 public:
  G1Example(std::string networkInterface)
      : time_(0.0), control_dt_(0.002), duration_(3.0),
        mode_(0), mode_machine_(0), running_motion_(false),
        blend_weight_(0.0f), blend_target_(1.0f), blend_ramp_up_s_(0.6f), blend_ramp_down_s_(0.5f),
        approach_duration_(1.0), return_duration_(1.0), hold_duration_(1.5), frame_duration_(0.25),
        phase_(Phase::STOPPED), frame_time_accum_(0.0), hold_time_accum_(0.0), cur_frame_(0)
  {
    // Allow field tuning via env (scalable, portable)
    if (const char* s = std::getenv("G1_ARM_BLEND"))        { float v = std::atof(s); if (v>=0.f && v<=1.f) blend_target_ = v; }
    if (const char* s = std::getenv("G1_ARM_RAMP_IN_S"))    { double v = std::atof(s); if (v>0.05) approach_duration_ = v; }
    if (const char* s = std::getenv("G1_ARM_RAMP_OUT_S"))   { double v = std::atof(s); if (v>0.05) return_duration_ = v; }
    if (const char* s = std::getenv("G1_ARM_HOLD_S"))       { double v = std::atof(s); if (v>=0.0) hold_duration_ = v; }
    if (const char* s = std::getenv("G1_FRAME_DT_S"))       { double v = std::atof(s); if (v>=0.02) frame_duration_ = v; }

    // DDS init
    ChannelFactory::Instance()->Init(0, networkInterface);

    // High-level Loco RPC (keeps onboard balance controller running)
    loco_client_ = std::make_shared<unitree::robot::g1::LocoClient>();
    loco_client_->SetTimeout(5.0F);
    loco_client_->Init();

    // NO LONGER DEACTIVATING built-in motion services. We leave balance on.
    // (Kept for compatibility but unused)
    msc = std::make_shared<MotionSwitcherClient>();
    msc->SetTimeout(5.0F);
    msc->Init();

    // Publishers/subscribers
    lowcmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(HG_CMD_TOPIC);
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(HG_STATE_TOPIC);
    lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);
  }

  void PlayMotion() {
    std::cout << "\n🤖 Performing motion (upper-limb blend, balance ON)...\n";
    // Ensure robot is in balanced standing, not actively walking
    enterBalancedStanding_();

    // Reset sequencing
    time_ = 0.0;
    running_motion_ = true;
    blend_weight_ = 0.0f;
    phase_ = Phase::RAMP_IN;
    frame_time_accum_ = 0.0;
    hold_time_accum_  = 0.0;
    cur_frame_ = 0;

    if (!command_writer_ptr_) command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1Example::LowCommandWriter, this);
    if (!control_thread_ptr_) control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1Example::Control, this);
  }

  void loadBehaviorLibrary(const std::string &motion_path) {
    YAML::Node motion = YAML::LoadFile(motion_path);
    frames_data_.clear();
    auto frames = motion["components"][1]["frames"];
    for (const auto &frame : frames) {
      std::vector<double> frame_data;
      for (const auto &element : frame) frame_data.push_back(element.as<double>());
      frames_data_.push_back(frame_data);
    }
  }

  bool IsRunning() const { return running_motion_; }

  void LowStateHandler(const void *message) {
    unitree_hg::msg::dds_::LowState_ low_state = *(const unitree_hg::msg::dds_::LowState_ *)message;
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(low_state) >> 2) - 1)) return;

    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q[i] = low_state.motor_state()[i].q();
      ms_tmp.dq[i] = low_state.motor_state()[i].dq();
    }
    motor_state_buffer_.SetData(ms_tmp);

    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_state_buffer_.SetData(imu_tmp);

    if (mode_machine_ != low_state.mode_machine()) mode_machine_ = low_state.mode_machine();
  }

  // Writer publishes to rt/arm_sdk with a blend weight on index 29
  void LowCommandWriter() {
    if (!running_motion_) return;

    unitree_hg::msg::dds_::LowCmd_ cmd;
    cmd.mode_pr() = mode_;
    cmd.mode_machine() = mode_machine_;

    const auto mc = motor_command_buffer_.GetData();
    if (mc) {
      // Send ONLY upper-limb indices (avoid legs/balance)
      auto apply_joint = [&](int i){
        cmd.motor_cmd()[i].mode() = 1;         // position mode
        cmd.motor_cmd()[i].tau()  = 0.0f;
        cmd.motor_cmd()[i].q()    = mc->q_target[i];
        cmd.motor_cmd()[i].dq()   = 0.0f;      // smooth path
        cmd.motor_cmd()[i].kp()   = mc->kp[i];
        cmd.motor_cmd()[i].kd()   = mc->kd[i];
      };

      for (int i = LeftShoulderPitch; i <= LeftWristYaw;  ++i) apply_joint(i);
      for (int i = RightShoulderPitch; i <= RightWristYaw; ++i) apply_joint(i);

      // --- Blend weight on special index 29 (not part of G1_NUM_MOTOR) ---
      float dyn_weight = std::clamp(blend_weight_, 0.0f, 1.0f);
      if (auto imu = imu_state_buffer_.GetData()) {
        float w2 = imu->omega[0]*imu->omega[0] + imu->omega[1]*imu->omega[1] + imu->omega[2]*imu->omega[2];
        // Light heuristic: if |ω| > ~2 rad/s, fade the arms to avoid destabilizing
        if (w2 > 4.0f) dyn_weight *= 0.4f;
      }
      cmd.motor_cmd()[29].q() = dyn_weight;

      cmd.crc() = Crc32Core((uint32_t *)&cmd, (sizeof(cmd) >> 2) - 1);
      lowcmd_publisher_->Write(cmd);
    }
  }

  // ================== Control loop with explicit FSM ==================
  void Control() {
    if (!running_motion_) return;
    const auto ms = motor_state_buffer_.GetData();
    if (!ms || frames_data_.empty()) return;

    MotorCommand cmd_tmp;
    time_ += control_dt_;

    // Neutral poses for return
    const std::array<double, 7> left_arm_down  = {0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0};
    const std::array<double, 7> right_arm_down = {0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0};

    static std::array<float, G1_NUM_MOTOR> start_pose;   // capture at phase transitions
    static std::array<float, G1_NUM_MOTOR> target_pose;  // current target frame pose (mapped into 29 motors)

    auto map_frame_to_pose = [&](size_t fidx, std::array<float,G1_NUM_MOTOR>& out){
      out.fill(0.0f);
      // frames store 14 values: L[0..6], R[0..6]
      const auto& fr = frames_data_[fidx];
      // Left arm
      for (int j=0;j<7;j++) {
        out[LeftShoulderPitch + j] = (j < (int)fr.size()? (float)fr[j] : 0.0f);
      }
      // Right arm
      for (int j=0;j<7;j++) {
        int src = 7 + j;
        out[RightShoulderPitch + j] = (src < (int)fr.size()? (float)fr[src] : 0.0f);
      }
    };

    // Helper: LERP two full-body arrays for output (we only care about arms)
    auto lerp_into_cmd = [&](const std::array<float,G1_NUM_MOTOR>& a,
                             const std::array<float,G1_NUM_MOTOR>& b,
                             double alpha){
      for (int i=0;i<G1_NUM_MOTOR;i++){
        double q = (1.0 - alpha)*a[i] + alpha*b[i];
        cmd_tmp.q_target[i] = (float)q;
        cmd_tmp.dq_target[i] = 0.0f;
        cmd_tmp.tau_ff[i] = 0.0f;
        cmd_tmp.kp[i] = GetMotorKp(G1MotorType[i]);
        cmd_tmp.kd[i] = GetMotorKd(G1MotorType[i]);
      }
    };

    // Capture current measured pose as an array<float> (arms + others passthrough)
    auto capture_measured_pose = [&](){
      std::array<float,G1_NUM_MOTOR> m{}; m.fill(0.0f);
      for (int i=0;i<G1_NUM_MOTOR;i++) m[i] = ms->q[i];
      return m;
    };

    // Blend weight evolution
    static double blend_t = 0.0;

    switch (phase_) {
      case Phase::RAMP_IN: {
        if (time_ == control_dt_) {
          // phase start
          start_pose = capture_measured_pose();
          map_frame_to_pose(0, target_pose);
          blend_t = 0.0;
        }
        blend_t += control_dt_;
        float up_alpha = float(std::min(blend_t / blend_ramp_up_s_, 1.0));
        blend_weight_ = up_alpha * blend_target_;

        double alpha = std::min(time_ / approach_duration_, 1.0);
        lerp_into_cmd(start_pose, target_pose, alpha);
        if (alpha >= 1.0) {
          // arrive at first frame → continue
          time_ = 0.0;
          frame_time_accum_ = 0.0;
          cur_frame_ = 0;
          phase_ = (frames_data_.size() > 1) ? Phase::FOLLOW_FRAMES : Phase::HOLD;
          hold_time_accum_ = 0.0;
        }
      } break;

      case Phase::FOLLOW_FRAMES: {
        // Play keyframes with fixed per-frame duration (simple linear interp)
        size_t next = std::min(cur_frame_ + 1, frames_data_.size()-1);
        std::array<float,G1_NUM_MOTOR> a{}, b{};
        map_frame_to_pose(cur_frame_, a);
        map_frame_to_pose(next,       b);

        frame_time_accum_ += control_dt_;
        double alpha = std::min(frame_time_accum_ / frame_duration_, 1.0);
        lerp_into_cmd(a, b, alpha);

        blend_weight_ = blend_target_; // hold max blend while playing frames

        if (alpha >= 1.0) {
          frame_time_accum_ = 0.0;
          if (next >= frames_data_.size()-1) {
            // reached last frame → HOLD
            phase_ = Phase::HOLD;
            hold_time_accum_ = 0.0;
            // Ensure we keep last frame as target
            map_frame_to_pose(frames_data_.size()-1, target_pose);
          } else {
            cur_frame_ = next;
          }
        }
      } break;

      case Phase::HOLD: {
        // Keep last (or single) frame pose for a dwell time
        hold_time_accum_ += control_dt_;
        lerp_into_cmd(target_pose, target_pose, 1.0); // just write target_pose
        blend_weight_ = blend_target_;
        if (hold_time_accum_ >= hold_duration_) {
          // Start returning to neutral
          time_ = 0.0;
          start_pose = capture_measured_pose(); // current arms position as start
          // Build neutral target
          std::array<float,G1_NUM_MOTOR> neutral{}; neutral.fill(0.0f);
          for (int j=0;j<7;j++) neutral[LeftShoulderPitch + j]  = (float)left_arm_down[j];
          for (int j=0;j<7;j++) neutral[RightShoulderPitch + j] = (float)right_arm_down[j];
          target_pose = neutral;
          blend_t = 0.0; // reuse for ramp-down
          phase_ = Phase::RETURN;
        }
      } break;

      case Phase::RETURN: {
        blend_t += control_dt_;
        float down_alpha = float(std::min(blend_t / blend_ramp_down_s_, 1.0));
        // Position interpolation back to neutral
        double alpha = std::min(time_ / return_duration_, 1.0);
        lerp_into_cmd(start_pose, target_pose, alpha);
        // Fade the blend weight down
        blend_weight_ = (1.0f - down_alpha) * blend_target_;

        if (alpha >= 1.0) {
          phase_ = Phase::STOPPED;
          running_motion_ = false;
          blend_weight_ = 0.0f;
          std::cout << "Motion finished smoothly. Stopping commands.\n";
        }
      } break;

      case Phase::STOPPED:
      default:
        running_motion_ = false;
        blend_weight_ = 0.0f;
        return;
    }

    // Push computed targets to the writer
    motor_command_buffer_.SetData(cmd_tmp);
  }

  // Exposed for old helper; still present but no longer used to disable modes
  int queryMotionStatus() {
    std::string form, name;
    int32_t ret = msc->CheckMode(form, name);
    return name.empty() ? 0 : 1;
  }

 private:
  enum class Phase { RAMP_IN, FOLLOW_FRAMES, HOLD, RETURN, STOPPED };

  // Enter balanced standing via RPC, with basic checks
  void enterBalancedStanding_() {
    if (!loco_client_) return;
    // Stop any residual motion
    loco_client_->StopMove();
    // Ask for balanced stand
    loco_client_->BalanceStand();
    // Request balance strategy = 0 (balanced standing) when available
    loco_client_->SetBalanceMode(0);

    // Wait briefly for standing state (best-effort)
    int fsm_mode = 1; // 0: standing; 1: moving
    for (int i=0; i<30; ++i) {
      if (loco_client_->GetFsmMode(fsm_mode) == 0 && fsm_mode == 0) break;
      usleep(50000); // 50 ms
    }
  }

 private:
  double time_, control_dt_, duration_;
  uint8_t mode_, mode_machine_;
  bool running_motion_;
  std::vector<std::vector<double>> frames_data_;

  // Blending control for arm_sdk
  float  blend_weight_;
  float  blend_target_;
  float  blend_ramp_up_s_;
  float  blend_ramp_down_s_;

  // Sequencer timing
  double approach_duration_;
  double return_duration_;
  double hold_duration_;
  double frame_duration_;

  // Sequencer state
  Phase  phase_;
  double frame_time_accum_;
  double hold_time_accum_;
  size_t cur_frame_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;

  // Keep but do not use to release modes
  std::shared_ptr<MotionSwitcherClient> msc;

  // High-level balance/stance client
  std::shared_ptr<unitree::robot::g1::LocoClient> loco_client_;
};

// === Helpers (unchanged) ===
std::string toLowerAndTrim(const std::string &s) {
  size_t start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  size_t end = s.find_last_not_of(" \t\r\n");
  std::string t = s.substr(start, end - start + 1);
  std::transform(t.begin(), t.end(), t.begin(), ::tolower);
  return t;
}

int levenshteinDistance(const std::string &s1, const std::string &s2) {
  const size_t len1 = s1.size(), len2 = s2.size();
  std::vector<std::vector<int>> d(len1 + 1, std::vector<int>(len2 + 1));
  for (size_t i = 0; i <= len1; ++i) d[i][0] = i;
  for (size_t j = 0; j <= len2; ++j) d[0][j] = j;
  for (size_t i = 1; i <= len1; ++i) {
    for (size_t j = 1; j <= len2; ++j) {
      int cost = (s1[i - 1] == s2[j - 1]) ? 0 : 1;
      d[i][j] = std::min({d[i - 1][j] + 1, d[i][j - 1] + 1, d[i - 1][j - 1] + cost});
    }
  }
  return d[len1][len2];
}

bool fuzzyMatch(const std::string &input, const std::vector<std::string> &triggers, double threshold, std::string *closest = nullptr) {
  double bestSimilarity = 0.0; std::string bestTrigger;
  for (const auto &trigger : triggers) {
    int dist = levenshteinDistance(input, trigger);
    double similarity = 1.0 - (double)dist / std::max(input.size(), trigger.size());
    if (similarity > bestSimilarity) { bestSimilarity = similarity; bestTrigger = trigger; }
  }
  if (closest) *closest = bestTrigger;
  return bestSimilarity >= threshold;
}

// ---------- Help (unchanged) ----------
void print_help() {
  std::cout <<
R"(Usage:
  AbdullahElnahrawy_g1_motions <net_if> [options]

Options:
  --help                     Show this help and exit
  --yaml <path>              Path to motions_registry.yaml (optional)
  --play <file.seq>          Play a motion file immediately (non-interactive)
  --say  "<phrase>"          Fuzzy-match phrase to a motion's triggers and play

Notes:
  - If --play is given a relative path, it is resolved under BLIB_DIR (motion_data/).
  - If neither --play nor --say is given, the app starts the classic interactive menu.
)" << std::endl;
}

// =================== MAIN (unchanged behavior) ===================
int main(int argc, char const *argv[]) {
  // Print project branding (commented originally)
  // std::cout << PROJECT_NAME << " — " << PROJECT_AUTHOR
  //           << " (" << PROJECT_SEMVER << ")\n";
  // std::cout << "Contact: " << PROJECT_EMAIL << " | " << PROJECT_GITHUB << "\n";

  if (argc < 2) {
    print_help();
    return 0;
  }

  // Required
  std::string interface = argv[1];

  // Defaults
  std::string registryPath = std::string(PROJECT_SOURCE_DIR) + "/config/motions_registry.yaml";
  std::string playFile;
  std::string sayPhrase;

  // Parse flags
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help" || a == "-h") { print_help(); return 0; }
    else if (a == "--yaml" && i + 1 < argc) { registryPath = argv[++i]; }
    else if (a == "--play" && i + 1 < argc) { playFile = argv[++i]; }
    else if (a == "--say"  && i + 1 < argc) { sayPhrase = argv[++i]; }
    else if (a.size() > 4 && a.substr(a.size()-4) == ".seq") { playFile = a; } // convenience
  }

  if (!loadMotionRegistry(registryPath)) return 1;

  // If --say provided, fuzzy-pick a file from registry triggers, RESPECTING per-motion thresholds.
  if (!sayPhrase.empty()) {
    std::string in = toLowerAndTrim(sayPhrase);
    std::string bestFile; double bestScore = -1.0; double bestThresh = 1.0;

    for (const auto &m : motionRegistry) {
      for (const auto &t : m.triggers) {
        int dist = levenshteinDistance(in, toLowerAndTrim(t));
        double sim = 1.0 - (double)dist / std::max(in.size(), t.size());
        if (sim > bestScore) {
          bestScore = sim;
          bestFile  = m.file_name;
          bestThresh = m.fuzzy_threshold;
        }
      }
    }

    if (!bestFile.empty() && bestScore >= bestThresh) {
      playFile = bestFile;
    } else {
      std::cout << "No motion matched (best similarity=" << bestScore << " < threshold=" << bestThresh << ").\n";
      return 2; // explicit non-match exit code for callers
    }
  }

  G1Example robot(interface);

  // --- Headless play mode ---
  if (!playFile.empty()) {
    std::string full = fs::path(playFile).is_absolute()
        ? playFile
        : (fs::path(BLIB_DIR) / playFile).string();
    std::cout << "Playing (headless): " << full << "\n";
    robot.loadBehaviorLibrary(full);
    robot.PlayMotion();

    // exit after motion completes so the caller can run the next one
    while (robot.IsRunning()) { usleep(10000); }

    return 0;
  }

  // --- Interactive menu (legacy) ---
  while (true) {
    std::cout << "\nAvailable motions:\n";
    for (size_t i = 0; i < motionRegistry.size(); ++i)
      std::cout << "  " << i + 1 << ". " << motionRegistry[i].file_name << "\n";
    std::cout << "  0. Exit program\n";

    int choice; std::cout << "Select motion number: "; std::cin >> choice;
    if (choice == 0) return 0;
    if (choice < 1 || (size_t)choice > motionRegistry.size()) { std::cout << "Invalid choice.\n"; continue; }

    Motion selectedMotion = motionRegistry[choice - 1];
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    robot.loadBehaviorLibrary(BLIB_DIR + selectedMotion.file_name);

    std::cout << "Default fuzzy threshold for this motion: " << selectedMotion.fuzzy_threshold << "\n";
    std::cout << "Enter new threshold (0.0 - 1.0) or press Enter to keep: ";
    std::string thresholdInput; std::getline(std::cin, thresholdInput);
    double sessionThreshold = selectedMotion.fuzzy_threshold;
    if (!thresholdInput.empty()) {
      try { sessionThreshold = std::stod(thresholdInput); }
      catch (...) { std::cout << "Invalid input, using default threshold.\n"; }
    }

    std::cout << "\nSay any trigger phrase to play motion:\n";
    for (auto &t : selectedMotion.triggers) std::cout << "  \"" << t << "\"\n";
    std::cout << "(Threshold = " << sessionThreshold << ")\n";
    std::cout << "Type \"back\" or \"exit\".\n";

    std::string input;
    while (true) {
      std::cout << ">> ";
      std::getline(std::cin, input);
      input = toLowerAndTrim(input);

      std::string closestTrigger;
      if (fuzzyMatch(input, selectedMotion.triggers, sessionThreshold, &closestTrigger)) {
        std::cout << "Trigger matched (fuzzy). Closest phrase: \"" << closestTrigger << "\"\n";
        robot.PlayMotion();
      } else if (input == "back") break;
      else if (input == "exit") return 0;
      else std::cout << "Unknown command.\n";
      sleep(1);
    }
  }
  return 0;
}
