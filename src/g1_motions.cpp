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

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

// Project Branding
#include "robot/branding/branding_fingerprint.hpp"
#include "project_branding.hpp"

#ifndef BLIB_DIR
#define BLIB_DIR std::string(PROJECT_SOURCE_DIR) + "/motion_data/"
#endif

using namespace unitree::robot::b2;
using namespace unitree::common;
using namespace unitree::robot;
namespace fs = std::filesystem;

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
const int G1_NUM_MOTOR = 29;

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

enum G1JointValidIndex {
  LeftShoulderPitch = 15, LeftShoulderRoll = 16, LeftShoulderYaw = 17,
  LeftElbow = 18, LeftWristRoll = 19, LeftWristPitch = 20, LeftWristYaw = 21,
  RightShoulderPitch = 22, RightShoulderRoll = 23, RightShoulderYaw = 24,
  RightElbow = 25, RightWristRoll = 26, RightWristPitch = 27, RightWristYaw = 28
};

float GetMotorKp(MotorType type) { return (type == GearboxL) ? 100 : 40; }
float GetMotorKd(MotorType type) { return 1; }

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

// Load motion registry from YAML
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
        mode_(0), mode_machine_(0), running_motion_(false) {
    ChannelFactory::Instance()->Init(0, networkInterface);
    msc = std::make_shared<MotionSwitcherClient>();
    msc->SetTimeout(5.0F);
    msc->Init();
    while (queryMotionStatus()) {
      std::cout << "Try to deactivate motion control service.\n";
      msc->ReleaseMode();
      sleep(5);
    }
    lowcmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(HG_CMD_TOPIC);
    lowcmd_publisher_->InitChannel();
    lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(HG_STATE_TOPIC);
    lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);
  }

  void PlayMotion() {
    std::cout << "\n🤖 Performing motion...\n";
    time_ = 0.0;
    running_motion_ = true;
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

  void LowCommandWriter() {
    if (!running_motion_) return;
    unitree_hg::msg::dds_::LowCmd_ cmd;
    cmd.mode_pr() = mode_;
    cmd.mode_machine() = mode_machine_;
    const auto mc = motor_command_buffer_.GetData();
    if (mc) {
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        cmd.motor_cmd()[i].mode() = 1;
        cmd.motor_cmd()[i].tau() = mc->tau_ff[i];
        cmd.motor_cmd()[i].q() = mc->q_target[i];
        cmd.motor_cmd()[i].dq() = mc->dq_target[i];
        cmd.motor_cmd()[i].kp() = mc->kp[i];
        cmd.motor_cmd()[i].kd() = mc->kd[i];
      }
      cmd.crc() = Crc32Core((uint32_t *)&cmd, (sizeof(cmd) >> 2) - 1);
      lowcmd_publisher_->Write(cmd);
    }
  }

  // ================== Control ==================
  void Control() {
    if (!running_motion_) return;
    MotorCommand cmd_tmp;
    const auto ms = motor_state_buffer_.GetData();
    if (!ms || frames_data_.empty()) return;

    double transition_duration = 1.5, frame_dt = control_dt_;
    time_ += control_dt_;

    std::array<double, 7> left_arm_down  = {0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0};
    std::array<double, 7> right_arm_down = {0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0};

    static bool returning = false;
    static double return_time = 0.0;
    static std::array<float, G1_NUM_MOTOR> start_return_pose;

    if (!returning) {
      if (time_ < transition_duration) {
        double alpha = time_ / transition_duration;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          double target = (i >= LeftShoulderPitch) ? frames_data_[0][i - LeftShoulderPitch] : 0.0;
          cmd_tmp.q_target[i] = (1.0 - alpha) * ms->q[i] + alpha * target;
          cmd_tmp.dq_target[i] = 0.0; cmd_tmp.tau_ff[i] = 0.0;
          cmd_tmp.kp[i] = GetMotorKp(G1MotorType[i]); cmd_tmp.kd[i] = GetMotorKd(G1MotorType[i]);
        }
      } else {
        double t = (time_ - transition_duration) / frame_dt;
        size_t frame_index = static_cast<size_t>(t);
        double alpha = t - frame_index;

        if (frame_index >= frames_data_.size() - 1) {
          returning = true; return_time = 0.0;
          for (int i = 0; i < G1_NUM_MOTOR; ++i) start_return_pose[i] = ms->q[i];
        } else {
          for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            double curr = (i >= LeftShoulderPitch) ? frames_data_[frame_index][i - LeftShoulderPitch] : 0.0;
            double nxt  = (frame_index + 1 < frames_data_.size() && i >= LeftShoulderPitch)
                            ? frames_data_[frame_index + 1][i - LeftShoulderPitch] : curr;
            cmd_tmp.q_target[i] = curr * (1.0 - alpha) + nxt * alpha;
            cmd_tmp.dq_target[i] = 0.0; cmd_tmp.tau_ff[i] = 0.0;
            cmd_tmp.kp[i] = GetMotorKp(G1MotorType[i]); cmd_tmp.kd[i] = GetMotorKd(G1MotorType[i]);
          }
        }
      }
    } else {
      return_time += control_dt_;
      double beta = std::min(return_time / 1.5, 1.0);
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        double target;
        if (i >= LeftShoulderPitch && i <= LeftWristYaw) target = left_arm_down[i - LeftShoulderPitch];
        else if (i >= RightShoulderPitch && i <= RightWristYaw) target = right_arm_down[i - RightShoulderPitch];
        else target = start_return_pose[i];
        cmd_tmp.q_target[i] = (1.0 - beta) * start_return_pose[i] + beta * target;
        cmd_tmp.dq_target[i] = 0.0; cmd_tmp.tau_ff[i] = 0.0;
        cmd_tmp.kp[i] = GetMotorKp(G1MotorType[i]); cmd_tmp.kd[i] = GetMotorKd(G1MotorType[i]);
      }
      if (beta >= 1.0) {
        returning = false; running_motion_ = false;
        std::cout << "Motion finished smoothly. Stopping commands.\n";
      }
    }
    motor_command_buffer_.SetData(cmd_tmp);
  }

  int queryMotionStatus() {
    std::string form, name;
    int32_t ret = msc->CheckMode(form, name);
    return name.empty() ? 0 : 1;
  }

 private:
  double time_, control_dt_, duration_;
  uint8_t mode_, mode_machine_;
  bool running_motion_;
  std::vector<std::vector<double>> frames_data_;
  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;
  ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;
  std::shared_ptr<MotionSwitcherClient> msc;
};

// === Helpers ===
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

// ---------- Help ----------
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

// =================== MAIN ===================
int main(int argc, char const *argv[]) {
// Print project branding
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

    // FIX: exit after motion completes so the caller (voice_interface) can run the next one
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
