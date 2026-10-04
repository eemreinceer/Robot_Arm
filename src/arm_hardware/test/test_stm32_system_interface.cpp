// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using namespace std::chrono_literals;

class PtyProtocolEmulator
{
public:
  explicit PtyProtocolEmulator(bool answer_handshake)
  : answer_handshake_(answer_handshake)
  {
    int descriptors[2] = {-1, -1};
    if (::openpty(&descriptors[0], &descriptors[1], nullptr, nullptr, nullptr) != 0) {
      throw std::runtime_error("openpty failed");
    }
    master_fd_ = descriptors[0];
    slave_fd_ = descriptors[1];
    const char * name = ::ttyname(slave_fd_);
    if (name == nullptr) {
      close_descriptors();
      throw std::runtime_error("ttyname failed");
    }
    slave_name_ = name;
    const int flags = ::fcntl(master_fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(master_fd_, F_SETFL, flags | O_NONBLOCK) != 0) {
      close_descriptors();
      throw std::runtime_error("failed to make PTY master nonblocking");
    }
    worker_ = std::thread([this]() {run();});
  }

  ~PtyProtocolEmulator()
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    close_descriptors();
  }

  PtyProtocolEmulator(const PtyProtocolEmulator &) = delete;
  PtyProtocolEmulator & operator=(const PtyProtocolEmulator &) = delete;

  const std::string & slave_name() const {return slave_name_;}

  bool wait_for_frame(const std::string & frame, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this, &frame]() {
               for (const auto & observed : frames_) {
                 if (observed == frame) {
                   return true;
                 }
               }
               return false;
    });
  }

  std::string error() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

private:
  void close_descriptors()
  {
    if (master_fd_ >= 0) {
      ::close(master_fd_);
      master_fd_ = -1;
    }
    if (slave_fd_ >= 0) {
      ::close(slave_fd_);
      slave_fd_ = -1;
    }
  }

  bool write_all(const std::string & data, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::size_t offset = 0;
    while (offset < data.size() && running_.load()) {
      const ssize_t count = ::write(master_fd_, data.data() + offset, data.size() - offset);
      if (count > 0) {
        offset += static_cast<std::size_t>(count);
        continue;
      }
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        return false;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      if (remaining <= 0ms) {
        return false;
      }
      pollfd descriptor{master_fd_, POLLOUT, 0};
      const int wait_ms = static_cast<int>(std::min(remaining, 20ms).count());
      (void)::poll(&descriptor, 1, wait_ms);
    }
    return offset == data.size();
  }

  void record_frame(const std::string & frame)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frames_.push_back(frame);
    }
    condition_.notify_all();
    if (frame == "V?" && answer_handshake_ && !write_all("V1,1.0\n", 200ms)) {
      set_error("failed to write handshake reply");
    }
  }

  void set_error(const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (error_.empty()) {
        error_ = message;
      }
    }
    condition_.notify_all();
  }

  void run()
  {
    std::string buffered;
    while (running_.load()) {
      pollfd descriptor{master_fd_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, 20);
      if (ready < 0) {
        if (errno != EINTR) {
          set_error("poll failed");
          return;
        }
        continue;
      }
      if (ready == 0 || (descriptor.revents & POLLIN) == 0) {
        continue;
      }
      char chunk[128] = {};
      const ssize_t count = ::read(master_fd_, chunk, sizeof(chunk));
      if (count > 0) {
        buffered.append(chunk, static_cast<std::size_t>(count));
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        set_error("read failed");
        return;
      }
      for (;; ) {
        const auto newline = buffered.find('\n');
        if (newline == std::string::npos) {
          break;
        }
        record_frame(buffered.substr(0, newline));
        buffered.erase(0, newline + 1);
      }
    }
  }

  bool answer_handshake_;
  int master_fd_{-1};
  int slave_fd_{-1};
  std::string slave_name_;
  std::atomic<bool> running_{true};
  std::thread worker_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<std::string> frames_;
  std::string error_;
};

std::string test_urdf(const std::string & serial_device)
{
  std::string joints;
  std::string links = "<link name=\"base_link\"/>";
  for (int index = 1; index <= 6; ++index) {
    links += "<link name=\"link_" + std::to_string(index) + "\"/>";
    joints +=
      "<joint name=\"joint_" + std::to_string(index) + "\" type=\"continuous\">"
      "<parent link=\"" + (index == 1 ? std::string("base_link") :
      "link_" + std::to_string(index - 1)) + "\"/>"
      "<child link=\"link_" + std::to_string(index) + "\"/></joint>";
  }

  std::string interfaces;
  for (int index = 1; index <= 6; ++index) {
    interfaces +=
      "<joint name=\"joint_" + std::to_string(index) + "\">"
      "<command_interface name=\"position\"/>"
      "<state_interface name=\"position\"/>"
      "<state_interface name=\"velocity\"/></joint>";
  }

  return
    "<?xml version=\"1.0\"?><robot name=\"pty_test\">" + links + joints +
    "<ros2_control name=\"test_system\" type=\"system\"><hardware>"
    "<plugin>arm_hardware/STM32SystemInterface</plugin>"
    "<param name=\"serial_device\">" + serial_device + "</param>"
    "<param name=\"baud_rate\">115200</param>"
    "<param name=\"mock_serial\">false</param>"
    "<param name=\"command_rate_hz\">50</param>"
    "<param name=\"serial_response_timeout_ms\">20</param>"
    "<param name=\"calibration_file\">" ARM_HARDWARE_TEST_CALIBRATION "</param>"
    "</hardware>" + interfaces + "</ros2_control></robot>";
}

std::unique_ptr<hardware_interface::ResourceManager> make_manager(const std::string & urdf)
{
  return std::make_unique<hardware_interface::ResourceManager>(
    urdf, std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("pty_system_test"), false, 0);
}

hardware_interface::return_type set_state(
  hardware_interface::ResourceManager & manager, uint8_t id, const std::string & label)
{
  rclcpp_lifecycle::State target(id, label);
  return manager.set_component_state("test_system", target);
}

TEST(STM32SystemInterfacePty, HandshakeArmAndDeactivateSendStopFrame)
{
  PtyProtocolEmulator emulator(true);
  auto manager = make_manager(test_urdf(emulator.slave_name()));
  ASSERT_TRUE(manager->get_components_status().count("test_system"));

  ASSERT_EQ(
    set_state(*manager, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"),
    hardware_interface::return_type::OK);
  ASSERT_TRUE(emulator.wait_for_frame("V?", 500ms));

  auto node = std::make_shared<rclcpp::Node>("pty_safety_client");
  auto client = node->create_client<rcl_interfaces::srv::SetParameters>(
    "/robot_arm_hardware_safety/set_parameters");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter reference;
  reference.name = "reference_positions";
  reference.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  reference.value.double_array_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rcl_interfaces::msg::Parameter armed;
  armed.name = "armed";
  armed.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  armed.value.bool_value = true;
  request->parameters = {reference, armed};

  auto future = client->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  ASSERT_EQ(response->results.size(), 2u);
  EXPECT_TRUE(response->results[0].successful) << response->results[0].reason;
  EXPECT_TRUE(response->results[1].successful) << response->results[1].reason;

  EXPECT_EQ(
    set_state(*manager, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"),
    hardware_interface::return_type::OK);
  EXPECT_TRUE(emulator.wait_for_frame("S", 500ms));
  EXPECT_TRUE(emulator.error().empty()) << emulator.error();
}

TEST(STM32SystemInterfacePty, MissingHandshakeAckFailsClosedWithinBound)
{
  PtyProtocolEmulator emulator(false);
  auto manager = make_manager(test_urdf(emulator.slave_name()));
  ASSERT_TRUE(manager->get_components_status().count("test_system"));

  const auto started = std::chrono::steady_clock::now();
  EXPECT_EQ(
    set_state(*manager, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"),
    hardware_interface::return_type::ERROR);
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_TRUE(emulator.wait_for_frame("V?", 200ms));
  EXPECT_LT(elapsed, 1500ms);
  EXPECT_FALSE(emulator.wait_for_frame("S", 50ms))
    << "an interface that never armed must not claim a delivered stop frame";
  EXPECT_TRUE(emulator.error().empty()) << emulator.error();
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
