// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0
//
// Regressions for the 2026-07-28 field fault: the ESP32 ACK exchange failed
// with `response='O'`, i.e. the `OK\n` reply was split across the read deadline
// and the leading byte arrived alone. Two defects made that possible:
//
//   1. the ACK deadline was the soft-start `step_period_ms` (a MOTION
//      parameter), so tuning PWM ramping silently retimed the serial link;
//   2. a timed-out partial line was dropped and the late tail stayed queued,
//      so a subsequent exchange would read the PREVIOUS reply as its own ACK.
//
// These tests drive the real send_line/read_line/drain_serial against a
// socketpair standing in for /dev/ttyTHS1, so the byte timing that produced
// the field symptom is reproduced rather than described.

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <thread>

#include "arm_hardware/stm32_system_interface.hpp"
#include "gtest/gtest.h"

namespace arm_hardware
{

// Granted access by the `friend` declaration in the interface header.
class STM32SystemInterfaceTestPeer
{
public:
  STM32SystemInterfaceTestPeer()
  {
    int pair[2] = {-1, -1};
    EXPECT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, pair), 0);
    // open_serial() opens the tty with O_NONBLOCK, and send_line's EAGAIN
    // handling only exists because of that. A blocking stand-in would let
    // ::write() block in the harness and hide the very behaviour under test.
    EXPECT_EQ(::fcntl(pair[0], F_SETFL, ::fcntl(pair[0], F_GETFL, 0) | O_NONBLOCK), 0);
    interface_.serial_fd_ = pair[0];
    peer_fd_ = pair[1];
  }

  ~STM32SystemInterfaceTestPeer()
  {
    if (peer_fd_ >= 0) {
      ::close(peer_fd_);
    }
    // interface_ closes its own fd in the destructor.
  }

  STM32SystemInterfaceTestPeer(const STM32SystemInterfaceTestPeer &) = delete;
  STM32SystemInterfaceTestPeer & operator=(const STM32SystemInterfaceTestPeer &) = delete;

  using ReadResult = STM32SystemInterface::ReadResult;

  ReadResult read_line(std::string & line, std::chrono::milliseconds timeout)
  {
    return interface_.read_line(line, timeout);
  }

  bool send_line(const std::string & line, std::chrono::milliseconds timeout)
  {
    return interface_.send_line(line, timeout);
  }

  bool send_line_until(
    const std::string & line, std::chrono::steady_clock::time_point deadline)
  {
    return interface_.send_line_until(line, deadline);
  }

  ReadResult read_line_until(
    std::string & line, std::chrono::steady_clock::time_point deadline)
  {
    return interface_.read_line_until(line, deadline);
  }

  std::size_t drain_serial(std::chrono::milliseconds grace)
  {
    return interface_.drain_serial(grace);
  }

  const std::string & partial_rx() const {return interface_.partial_rx();}
  bool completed_after_deadline() const
  {
    return interface_.last_read_completed_after_deadline_;
  }

  void set_step_period_ms(int value) {interface_.step_period_ms_ = value;}
  void set_ack_timeout_ms(int value) {interface_.serial_response_timeout_ms_ = value;}
  int ack_timeout_ms() const {return interface_.serial_response_timeout_ms_;}

  void reset_ack_telemetry() {interface_.reset_ack_telemetry();}

  void record_ack_latency(std::chrono::microseconds latency)
  {
    interface_.record_ack_latency(latency);
  }

  std::string ack_latency_distribution_summary() const
  {
    return interface_.ack_latency_distribution_summary();
  }

  // Writes bytes to the far end of the link, as the MCU would.
  void mcu_write(const std::string & bytes) const
  {
    ASSERT_GT(peer_fd_, 0);
    const ssize_t written = ::write(peer_fd_, bytes.data(), bytes.size());
    ASSERT_EQ(static_cast<std::size_t>(written), bytes.size());
  }

  std::string mcu_read_available() const
  {
    std::string collected;
    for (;; ) {
      pollfd descriptor{peer_fd_, POLLIN, 0};
      if (::poll(&descriptor, 1, 50) <= 0 || (descriptor.revents & POLLIN) == 0) {
        break;
      }
      char buffer[64] = {};
      const ssize_t count = ::read(peer_fd_, buffer, sizeof(buffer));
      if (count <= 0) {
        break;
      }
      collected.append(buffer, static_cast<std::size_t>(count));
    }
    return collected;
  }

  void close_peer()
  {
    if (peer_fd_ >= 0) {
      ::close(peer_fd_);
      peer_fd_ = -1;
    }
  }

  void shrink_send_buffer() const
  {
    int size = 1024;
    ::setsockopt(interface_.serial_fd_, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
  }

private:
  STM32SystemInterface interface_;
  int peer_fd_{-1};
};

namespace
{

using namespace std::chrono_literals;
using ReadResult = STM32SystemInterfaceTestPeer::ReadResult;

TEST(SerialAck, CompleteLineIsRead)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write("OK\n");

  std::string line;
  EXPECT_EQ(peer.read_line(line, 200ms), ReadResult::kLine);
  EXPECT_EQ(line, "OK");
  EXPECT_FALSE(peer.completed_after_deadline());
}

TEST(SerialAck, CarriageReturnIsStripped)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write("OK\r\n");

  std::string line;
  EXPECT_EQ(peer.read_line(line, 200ms), ReadResult::kLine);
  EXPECT_EQ(line, "OK");
}

// A deadline is a bound on WAITING, not permission to ignore bytes the kernel
// has already received.  The Nano field failure read the first 'O', resumed
// after the 20 ms deadline, then returned timeout before checking whether the
// queued "K\n" tail was already available.  Zero budget makes that scheduling
// state deterministic: the complete frame is buffered, but no further wait is
// allowed.
TEST(SerialAck, CompleteLineBufferedAtExpiredDeadlineIsStillConsumed)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write("OK\n");

  const auto started = std::chrono::steady_clock::now();
  std::string line;
  const ReadResult result = peer.read_line(line, 0ms);
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_EQ(result, ReadResult::kLine);
  EXPECT_EQ(line, "OK");
  EXPECT_TRUE(peer.completed_after_deadline());
  EXPECT_LT(elapsed, 50ms) << "expired-deadline recovery must never add a grace wait";
}

TEST(SerialAck, IncompleteBytesBufferedAtExpiredDeadlineStillTimeout)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write("O");

  std::string line;
  EXPECT_EQ(peer.read_line(line, 0ms), ReadResult::kTimeout);
  EXPECT_TRUE(line.empty());
  EXPECT_EQ(peer.partial_rx(), "O");
  EXPECT_FALSE(peer.completed_after_deadline());
}

// The field failure, reproduced exactly: 'O' arrives, the rest is late.
TEST(SerialAck, SplitAckThatCompletesInsideTheWindowSucceeds)
{
  STM32SystemInterfaceTestPeer peer;
  std::thread mcu([&peer]() {
      peer.mcu_write("O");
      std::this_thread::sleep_for(40ms);
      peer.mcu_write("K\n");
    });

  std::string line;
  const ReadResult result = peer.read_line(line, 400ms);
  mcu.join();

  EXPECT_EQ(result, ReadResult::kLine);
  EXPECT_EQ(line, "OK");
}

// The same split, but the window closes first. This must be reported as a
// TIMEOUT carrying the partial bytes -- not as an indistinguishable failure.
TEST(SerialAck, SplitAckThatOverrunsTheWindowReportsPartialBytes)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write("O");

  std::string line;
  EXPECT_EQ(peer.read_line(line, 20ms), ReadResult::kTimeout);
  EXPECT_TRUE(line.empty());
  // This is the byte the field log showed as response='O'.
  EXPECT_EQ(peer.partial_rx(), "O");
}

// The core regression for the assigned defect. Soft-start ramping and the ACK
// window are now independent: a tiny step period must not shrink the deadline.
TEST(SerialAck, AckWindowIsIndependentOfSoftStartStepPeriod)
{
  STM32SystemInterfaceTestPeer peer;
  peer.set_step_period_ms(1);       // aggressive PWM ramp
  peer.set_ack_timeout_ms(300);     // deliberate, measured comms budget
  ASSERT_EQ(peer.ack_timeout_ms(), 300);

  std::thread mcu([&peer]() {
      std::this_thread::sleep_for(60ms);   // longer than step_period_ms
      peer.mcu_write("OK\n");
    });

  std::string line;
  const ReadResult result =
    peer.read_line(line, std::chrono::milliseconds(peer.ack_timeout_ms()));
  mcu.join();

  EXPECT_EQ(result, ReadResult::kLine);
  EXPECT_EQ(line, "OK");
}

// After a timed-out exchange the late tail is still inbound. Without a drain
// the next read would return "K" -- or worse, a stale but well-formed "OK"
// belonging to the previous command, which stops looking like an error at all.
TEST(SerialAck, DrainRemovesTheLateTailSoTheNextExchangeIsNotOffByOne)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write("O");

  std::string line;
  ASSERT_EQ(peer.read_line(line, 20ms), ReadResult::kTimeout);
  ASSERT_EQ(peer.partial_rx(), "O");

  peer.mcu_write("K\n");             // the late tail finally lands
  EXPECT_EQ(peer.drain_serial(50ms), 2u);
  EXPECT_TRUE(peer.partial_rx().empty());

  // A fresh exchange now sees only its own reply.
  peer.mcu_write("OK\n");
  std::string next;
  EXPECT_EQ(peer.read_line(next, 200ms), ReadResult::kLine);
  EXPECT_EQ(next, "OK");
}

TEST(SerialAck, DrainOnQuietLinkDiscardsNothing)
{
  STM32SystemInterfaceTestPeer peer;
  EXPECT_EQ(peer.drain_serial(20ms), 0u);
}

// Demonstrates the desync the drain prevents: without it, the tail is read as
// the next reply. Pinning this keeps the failure mode described accurately.
TEST(SerialAck, WithoutDrainTheLateTailIsMisreadAsTheNextReply)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write("O");

  std::string line;
  ASSERT_EQ(peer.read_line(line, 20ms), ReadResult::kTimeout);

  peer.mcu_write("K\n");
  std::string next;
  EXPECT_EQ(peer.read_line(next, 200ms), ReadResult::kLine);
  EXPECT_EQ(next, "K");   // not "OK" -- the stream was left desynchronized
}

TEST(SerialAck, OverlongLineIsReportedAsFramingErrorNotTruncated)
{
  STM32SystemInterfaceTestPeer peer;
  peer.mcu_write(std::string(200, 'X'));

  std::string line;
  EXPECT_EQ(peer.read_line(line, 200ms), ReadResult::kOverflow);
  EXPECT_TRUE(line.empty());
  EXPECT_EQ(peer.partial_rx().size(), 127u);
}

TEST(SerialAck, HangupFailsFastInsteadOfWaitingOutTheWindow)
{
  STM32SystemInterfaceTestPeer peer;
  peer.close_peer();

  const auto started = std::chrono::steady_clock::now();
  std::string line;
  const ReadResult result = peer.read_line(line, 2000ms);
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_EQ(result, ReadResult::kError);
  EXPECT_LT(elapsed, 500ms);
}

TEST(SerialAck, SendLineDeliversTheFrame)
{
  STM32SystemInterfaceTestPeer peer;
  EXPECT_TRUE(peer.send_line("P1500,1914,2290,1373,1436,1500\n", 200ms));
  EXPECT_EQ(peer.mcu_read_available(), "P1500,1914,2290,1373,1436,1500\n");
}

// The control loop owns one total transaction budget, not one full timeout for
// TX followed by a second full timeout for RX. Once time consumed before the
// read has exhausted that shared deadline, read_line_until may inspect already
// buffered bytes but must not start a fresh wait window.
TEST(SerialAck, SendAndReadShareOneAbsoluteDeadline)
{
  STM32SystemInterfaceTestPeer peer;
  const auto shared_deadline = std::chrono::steady_clock::now() + 100ms;
  ASSERT_TRUE(peer.send_line_until("P1500,1914,2290,1373,1436,1500\n", shared_deadline));
  EXPECT_EQ(peer.mcu_read_available(), "P1500,1914,2290,1373,1436,1500\n");

  std::this_thread::sleep_until(shared_deadline + 5ms);
  const auto read_started = std::chrono::steady_clock::now();
  std::string response;
  EXPECT_EQ(peer.read_line_until(response, shared_deadline), ReadResult::kTimeout);
  const auto read_elapsed = std::chrono::steady_clock::now() - read_started;

  EXPECT_TRUE(response.empty());
  EXPECT_LT(read_elapsed, 50ms) << "an expired transaction must not receive a fresh RX timeout";
}

TEST(SerialAck, FixedHistogramReportsTailAndNearestRankPercentiles)
{
  STM32SystemInterfaceTestPeer peer;
  for (int index = 0; index < 50; ++index) {
    peer.record_ack_latency(500us);
  }
  for (int index = 0; index < 45; ++index) {
    peer.record_ack_latency(4500us);
  }
  for (int index = 0; index < 4; ++index) {
    peer.record_ack_latency(20500us);
  }
  peer.record_ack_latency(60000us);

  const std::string summary = peer.ack_latency_distribution_summary();
  EXPECT_NE(summary.find("p50<=1.00ms"), std::string::npos);
  EXPECT_NE(summary.find("p95<=5.00ms"), std::string::npos);
  EXPECT_NE(summary.find("p99<=25.00ms"), std::string::npos);
  EXPECT_NE(summary.find("1000:50"), std::string::npos);
  EXPECT_NE(summary.find("5000:45"), std::string::npos);
  EXPECT_NE(summary.find("25000:4"), std::string::npos);
  EXPECT_NE(summary.find("+inf:1"), std::string::npos);
}

TEST(SerialAck, EmptyHistogramIsExplicit)
{
  STM32SystemInterfaceTestPeer peer;
  EXPECT_EQ(
    peer.ack_latency_distribution_summary(),
    "ACK exchange distribution: no successful exchanges");
}

TEST(SerialAck, TelemetryResetSeparatesAcceptanceRuns)
{
  STM32SystemInterfaceTestPeer peer;
  peer.record_ack_latency(4500us);
  ASSERT_NE(
    peer.ack_latency_distribution_summary().find("p50<=5.00ms"),
    std::string::npos);

  peer.reset_ack_telemetry();
  EXPECT_EQ(
    peer.ack_latency_distribution_summary(),
    "ACK exchange distribution: no successful exchanges");
}

TEST(SerialAck, FixedHistogramAccountsForAcceptanceRunSize)
{
  STM32SystemInterfaceTestPeer peer;
  constexpr int kAcceptanceExchangeCount = 100000;
  for (int index = 0; index < kAcceptanceExchangeCount; ++index) {
    peer.record_ack_latency(3500us);
  }

  const std::string summary = peer.ack_latency_distribution_summary();
  EXPECT_NE(summary.find("p50<=4.00ms"), std::string::npos);
  EXPECT_NE(summary.find("p95<=4.00ms"), std::string::npos);
  EXPECT_NE(summary.find("p99<=4.00ms"), std::string::npos);
  EXPECT_NE(summary.find("4000:100000"), std::string::npos);
}

// The old send_line retried on EAGAIN with no poll and no deadline: a peer that
// stopped reading turned the real-time write() into an unbounded hot loop. It
// must now give up within its budget.
TEST(SerialAck, SendLineGivesUpInsteadOfSpinningWhenThePeerNeverReads)
{
  STM32SystemInterfaceTestPeer peer;
  peer.shrink_send_buffer();

  const std::string flood(1u << 20, 'P');
  const auto started = std::chrono::steady_clock::now();
  const bool sent = peer.send_line(flood, 100ms);
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_FALSE(sent);
  EXPECT_LT(elapsed, 2000ms);
}

}  // namespace
}  // namespace arm_hardware
