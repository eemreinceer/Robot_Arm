// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "arm_hardware/activation_safety.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_hardware
{

class STM32SystemInterface final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STM32SystemInterface)

  ~STM32SystemInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  // ros2_control routes a write()/read() failure through the lifecycle ERROR
  // transition, NOT through deactivate(). Measured 2026-07-29: without this
  // override the default no-op ran, so an ESP32 ACK failure emitted zero stop
  // frames and left the interface internally armed. See test_error_transition.
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct ChannelCalibration
  {
    int channel{0};
    std::string joint;
    // ÖLÇEK ÇAPASI: min_rad/max_rad <-> min_us/max_us dogrusal eslemesini
    // tanimlar. Bunlar OLCULMUS fiziksel kalibrasyondur (tezgahta 500/2500 us
    // = 180 derece) ve guvenlik icin daraltilamaz -- daraltmak rad->us
    // olcegini bozar, joint_5 gibi asimetrik bir daraltmada notr bile kayar.
    double min_rad{0.0};
    double max_rad{0.0};
    int min_us{500};
    int max_us{2500};
    // GUVENLIK LIMITI: kolun gitmesine izin verilen aralik. Olcegi ETKILEMEZ.
    // YAML'da yoksa min_rad/max_rad'a duser (eski davranis).
    double limit_min_rad{0.0};
    double limit_max_rad{0.0};
    double zero_offset_rad{0.0};
    double max_velocity_rad_s{0.1};
    bool invert{false};
  };

  // Outcome of one framed read. A timeout is NOT an error: the link is alive
  // and the line may simply be incomplete, which is the field failure this
  // interface has to report precisely rather than collapse into `false`.
  enum class ReadResult
  {
    kLine,      // a complete '\n'-terminated line was received
    kTimeout,   // deadline expired; partial_rx() holds what did arrive
    kOverflow,  // line exceeded the frame limit -- framing is broken
    kError      // poll/read failed, or the peer hung up
  };

  bool load_calibration(const std::string & path);
  // Fingerprint of the calibration semantics this interface is driving with:
  // channel map, scale anchors, safety limits, zero offset, pulse range and
  // direction. Both sides of the link derive it from the same YAML -- the host
  // at load time, the firmware at build time via generate_robot_config.py --
  // so a YAML edited without reflashing shows up as a mismatch.
  //
  // Deliberately NOT the generator's own SHA256: that digest is taken over the
  // generator's normalised channel dicts, which carry pin/ledc/deadband fields
  // that exist only in its FIRMWARE_POLICY table and never reach this file.
  // Reproducing it here would mean copying that table into C++ and matching
  // Python's float-to-JSON formatting byte for byte. This fingerprint covers
  // what the two sides actually have to agree on instead of where it came from.
  static std::string calibration_canonical(
    const std::array<ChannelCalibration, 6> & calibration);
  static std::string calibration_fingerprint(
    const std::array<ChannelCalibration, 6> & calibration);
  // Asks the firmware for its fingerprint. Returns the reply, or an empty
  // string when the firmware does not know the request (older builds).
  bool request_calibration_fingerprint(std::string & reported, std::string & failure);
  // What the activation gate decides. Kept separate from on_activate so the
  // three outcomes can be tested without a serial device: "cannot verify" and
  // "disagrees" must not collapse into one answer.
  enum class CalibrationGate
  {
    kAgreed,
    kMismatch,     // firmware answered, with different numbers -> always refuse
    kUnverified    // firmware could not answer -> refuse only if required
  };
  CalibrationGate evaluate_calibration_gate(bool answered, const std::string & reported) const;
  int position_to_microseconds(std::size_t joint_index, double position) const;
  double microseconds_to_position(std::size_t joint_index, int pulse_us) const;
  bool open_serial();
  void close_serial();
  bool send_line(const std::string & line, std::chrono::milliseconds timeout);
  bool send_line_until(
    const std::string & line, std::chrono::steady_clock::time_point deadline);
  ReadResult read_line(std::string & line, std::chrono::milliseconds timeout);
  ReadResult read_line_until(
    std::string & line, std::chrono::steady_clock::time_point deadline);
  // Discards whatever is still inbound so a failed exchange cannot leave a
  // half-consumed reply that the NEXT exchange would mistake for its own ACK.
  std::size_t drain_serial(std::chrono::milliseconds grace);
  const std::string & partial_rx() const {return partial_rx_;}
  void reset_ack_telemetry();
  void record_ack_latency(std::chrono::microseconds latency);
  std::string ack_latency_distribution_summary() const;
  bool handshake();
  std::string position_command(const std::array<int, 6> & pulse_us) const;
  bool start_safety_node();
  void stop_safety_node();
  bool send_stop_frame();
  // Single teardown path shared by deactivate and the error transition, so the
  // two can never drift apart on what "stop the arm" means.
  void release_to_safe_state(const char * cause);
  rcl_interfaces::msg::SetParametersResult on_safety_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  static constexpr std::size_t kChannelCount = 6;
  std::array<ChannelCalibration, kChannelCount> calibration_{};
  std::array<double, kChannelCount> position_commands_{};
  std::array<double, kChannelCount> position_states_{};
  std::array<double, kChannelCount> velocity_states_{};
  std::array<int, kChannelCount> last_pulse_us_{};

  // COMMANDED, NOT MEASURED. This arm has no encoders, so `position_states_`
  // above is an echo of what write() believes it delivered -- and once it
  // reaches joint_state_broadcaster, nothing downstream can tell an echo from a
  // measurement. These two members give that distinction a place to live
  // BEFORE the encoders arrive, exported through the optional `commanded` GPIO:
  //
  //   commanded_position_states_  what the interface last delivered (rad)
  //   position_is_measured_       1.0 only when position_states_ comes from a
  //                               real sensor; 0.0 while it is an echo
  //
  // When the magnetic encoders land, read() starts filling position_states_
  // from the sensor and raises the flag. Nothing above ros2_control changes,
  // and any consumer that cares can already ask the question today.
  std::array<double, kChannelCount> commanded_position_states_{};
  double position_is_measured_{0.0};
  // Index into the arrays above per GPIO state interface the URDF declared, so
  // a description without the `commanded` block (the mock test systems) keeps
  // working unchanged.
  struct GpioBinding
  {
    std::string interface_name;
    double * value{nullptr};
  };
  std::vector<GpioBinding> gpio_bindings_;

  std::string serial_device_{"/dev/ttyUSB0"};
  std::string calibration_file_;
  int baud_rate_{115200};
  double command_rate_hz_{50.0};
  bool mock_serial_{false};
  int max_delta_us_per_step_{10};
  int step_period_ms_{20};
  // ACK penceresi soft-start ile AYNI DEGER olabilir ama AYNI SEY DEGILDIR.
  // `step_period_ms_` bir hareket parametresidir (PWM ne kadar hizli ramplanir);
  // asagidaki ise haberlesme parametresidir (MCU cevabi ne kadar gecikebilir).
  // Bunlari 2026-07-28'e kadar tek degisken bagliyordu, yani soft-start'i
  // hizlandirmak sessizce seri zaman asimini da kisaltiyordu. Varsayilan
  // bilerek 20 ms: eski efektif degerin AYNISI, boylece bu ayristirma tek
  // basina saha davranisini degistirmez. Gercek deger ACK gecikme
  // telemetrisinden secilecek.
  int serial_response_timeout_ms_{20};
  bool calibration_complete_{false};
  // Refuse activation when the firmware cannot report a fingerprint at all.
  // Defaults to false so a Jetson running firmware that predates the check
  // still activates -- it warns instead. Flip it to true once the ESP32 has
  // been flashed, and the unverifiable case becomes a refusal too. A
  // fingerprint that IS reported and disagrees always refuses, whatever this
  // is set to.
  bool require_calibration_match_{false};
  std::string calibration_fingerprint_;
  double commissioning_range_rad_{0.3};
  int serial_fd_{-1};
  bool active_{false};
  bool arm_initialization_pending_{false};
  ActivationSafety activation_safety_;
  std::chrono::steady_clock::time_point last_write_time_{};
  std::mutex safety_mutex_;
  std::mutex serial_mutex_;
  rclcpp::Node::SharedPtr safety_node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr safety_parameter_callback_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> safety_executor_;
  std::thread safety_executor_thread_;
  // Set by the executor thread when spin() returns. stop_safety_node() needs it
  // because cancel() only clears the executor's spinning flag: a cancel that
  // lands before spin() sets that flag is lost, and join() then waits forever.
  std::atomic<bool> safety_spin_returned_{true};
  std::size_t mock_position_frame_count_{0};
  std::size_t mock_stop_frame_count_{0};

  // Bytes of an unterminated reply left over from the last read. Kept so a
  // timeout can report exactly what arrived (the field symptom was a lone 'O'
  // from an `OK\n` that never completed inside the window).
  std::string partial_rx_;
  // True only when read_line() completed a frame from bytes that were already
  // readable after the nominal wait deadline. No grace wait is added: this
  // distinguishes host scheduling delay from a genuinely late/incomplete ACK.
  bool last_read_completed_after_deadline_{false};

  // Komut baslangicindan tam ACK'e kadar toplam islem telemetrisi. Ariza
  // aralikli oldugu icin tek bir basarisizlik
  // aninin log'u yetmiyor; zaman asimi degerini VERIDEN secebilmek icin normal
  // calismadaki dagilim da olculuyor.
  std::uint64_t ack_exchange_count_{0};
  std::uint64_t ack_timeout_count_{0};
  std::uint64_t ack_buffered_completion_count_{0};
  std::chrono::microseconds ack_latency_max_{0};
  std::chrono::microseconds ack_latency_total_{0};
  // 1 ms upper-bound buckets through 20 ms, then 25 ms, 50 ms and overflow.
  // The fixed array adds no allocation or locking to the 50 Hz write path.
  static constexpr std::size_t kAckLatencyHistogramBinCount{23};
  std::array<std::uint64_t, kAckLatencyHistogramBinCount> ack_latency_histogram_{};

  friend class STM32SystemInterfaceTestPeer;
  friend class STM32SystemInterfaceLifecycleTestPeer;
  friend class STM32SystemInterfaceCommandedStateTestPeer;
  friend class STM32SystemInterfaceMappingTestPeer;
};

}  // namespace arm_hardware
