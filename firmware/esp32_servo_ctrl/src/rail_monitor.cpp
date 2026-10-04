#include "rail_monitor.hpp"

namespace robot_arm {
namespace {

constexpr uint32_t kRawUnset = 0xFFFFFFFFU;

}  // namespace

uint32_t railCountToMillivolts(
  const RailCalibration & calibration, uint32_t rawCount)
{
  if (calibration.adcMaxCount == 0U || calibration.dividerLowOhm == 0U) {
    return 0U;
  }
  if (rawCount > calibration.adcMaxCount) {
    rawCount = calibration.adcMaxCount;
  }

  // Node millivolts first, then undo the divider. 64-bit keeps the product
  // exact: 4095 * 3100 * (30000/10000) still has plenty of headroom.
  const uint64_t nodeMv =
    (static_cast<uint64_t>(rawCount) * calibration.adcFullScaleMv) /
    calibration.adcMaxCount;
  const uint64_t totalOhm =
    static_cast<uint64_t>(calibration.dividerHighOhm) + calibration.dividerLowOhm;
  const uint64_t railMv =
    (nodeMv * totalOhm) / calibration.dividerLowOhm;
  return static_cast<uint32_t>(railMv);
}

RailMonitor::RailMonitor(const RailCalibration & calibration)
: calibration_(calibration)
{
  reset(0U);
}

void RailMonitor::reset(uint32_t nowUs) {
  sampleCount_ = 0U;
  lastRaw_ = 0U;
  minRaw_ = kRawUnset;
  maxRaw_ = 0U;
  rawSum_ = 0U;

  for (std::size_t index = 0U; index < kRailBucketCount; ++index) {
    bucketMinRaw_[index] = kRawUnset;
  }
  bucketIndex_ = 0U;
  bucketStartedUs_ = nowUs;

  ringHead_ = 0U;
  ringFill_ = 0U;

  triggerState_ = RailTriggerState::Idle;
  triggerThresholdMv_ = 0U;
  triggerUs_ = 0U;
  postTriggerRemaining_ = 0U;
  frozenTriggerIndex_ = 0U;
}

void RailMonitor::rotateBuckets(uint32_t nowUs) {
  // Unsigned subtraction stays valid across wraparound. A long stall rotates
  // several buckets, so the window never reports a stale minimum as current.
  const uint32_t elapsedUs = nowUs - bucketStartedUs_;
  if (elapsedUs < kRailBucketIntervalUs) {
    return;
  }

  // A gap longer than the whole window means every bucket is stale. Clear them
  // in one pass and re-anchor: stepping one bucket at a time would spin tens of
  // thousands of iterations inside the sampler's critical section.
  constexpr uint32_t kWindowUs =
    kRailBucketIntervalUs * static_cast<uint32_t>(kRailBucketCount);
  if (elapsedUs >= kWindowUs) {
    for (std::size_t index = 0U; index < kRailBucketCount; ++index) {
      bucketMinRaw_[index] = kRawUnset;
    }
    bucketIndex_ = 0U;
    bucketStartedUs_ = nowUs;
    return;
  }

  const uint32_t steps = elapsedUs / kRailBucketIntervalUs;
  for (uint32_t step = 0U; step < steps; ++step) {
    bucketIndex_ = (bucketIndex_ + 1U) % kRailBucketCount;
    bucketMinRaw_[bucketIndex_] = kRawUnset;
  }
  bucketStartedUs_ += steps * kRailBucketIntervalUs;
}

std::size_t RailMonitor::ringIndex(std::size_t logicalIndex) const {
  // Logical 0 is the oldest retained sample.
  const std::size_t oldest =
    (ringFill_ < kRailCaptureSamples)
      ? 0U
      : ringHead_;
  return (oldest + logicalIndex) % kRailCaptureSamples;
}

void RailMonitor::pushRing(uint32_t rawCount, uint32_t nowUs) {
  ringRaw_[ringHead_] = rawCount;
  ringUs_[ringHead_] = nowUs;
  ringHead_ = (ringHead_ + 1U) % kRailCaptureSamples;
  if (ringFill_ < kRailCaptureSamples) {
    ++ringFill_;
  }
}

void RailMonitor::addSample(uint32_t rawCount, uint32_t nowUs) {
  if (rawCount > calibration_.adcMaxCount) {
    rawCount = calibration_.adcMaxCount;
  }

  // A frozen capture is evidence. Keep feeding statistics, but never let a
  // later sample overwrite the buffer the operator is about to read out.
  const bool captureFrozen = (triggerState_ == RailTriggerState::Complete);

  rotateBuckets(nowUs);

  lastRaw_ = rawCount;
  ++sampleCount_;
  rawSum_ += rawCount;
  if (rawCount < minRaw_) {
    minRaw_ = rawCount;
  }
  if (rawCount > maxRaw_) {
    maxRaw_ = rawCount;
  }
  if (rawCount < bucketMinRaw_[bucketIndex_]) {
    bucketMinRaw_[bucketIndex_] = rawCount;
  }

  if (captureFrozen) {
    return;
  }

  pushRing(rawCount, nowUs);

  if (triggerState_ == RailTriggerState::Armed) {
    const uint32_t mv = railCountToMillivolts(calibration_, rawCount);
    if (mv < triggerThresholdMv_) {
      triggerState_ = RailTriggerState::Capturing;
      triggerUs_ = nowUs;
      // Keep half the buffer as pre-trigger history and fill the rest after
      // the event, so the dump shows both the approach and the recovery.
      postTriggerRemaining_ = kRailCaptureSamples / 2U;
    }
  } else if (triggerState_ == RailTriggerState::Capturing) {
    if (postTriggerRemaining_ > 0U) {
      --postTriggerRemaining_;
    }
    if (postTriggerRemaining_ == 0U) {
      triggerState_ = RailTriggerState::Complete;
      // Resolve the trigger sample's logical position once, while the ring
      // still holds it. captureRelativeUsAt() would otherwise have to search.
      frozenTriggerIndex_ = 0U;
      for (std::size_t index = 0U; index < ringFill_; ++index) {
        if (ringUs_[ringIndex(index)] == triggerUs_) {
          frozenTriggerIndex_ = index;
          break;
        }
      }
    }
  }
}

bool RailMonitor::hasSample() const { return sampleCount_ > 0U; }
uint32_t RailMonitor::sampleCount() const { return sampleCount_; }

uint32_t RailMonitor::lastMv() const {
  return railCountToMillivolts(calibration_, lastRaw_);
}

uint32_t RailMonitor::minMv() const {
  if (minRaw_ == kRawUnset) {
    return 0U;
  }
  return railCountToMillivolts(calibration_, minRaw_);
}

uint32_t RailMonitor::maxMv() const {
  return railCountToMillivolts(calibration_, maxRaw_);
}

uint32_t RailMonitor::meanMv() const {
  if (sampleCount_ == 0U) {
    return 0U;
  }
  const uint32_t meanRaw =
    static_cast<uint32_t>(rawSum_ / static_cast<uint64_t>(sampleCount_));
  return railCountToMillivolts(calibration_, meanRaw);
}

uint32_t RailMonitor::minRecentMv() const {
  uint32_t lowest = kRawUnset;
  for (std::size_t index = 0U; index < kRailBucketCount; ++index) {
    if (bucketMinRaw_[index] < lowest) {
      lowest = bucketMinRaw_[index];
    }
  }
  if (lowest == kRawUnset) {
    return 0U;
  }
  return railCountToMillivolts(calibration_, lowest);
}

void RailMonitor::armTrigger(uint32_t thresholdMv) {
  triggerThresholdMv_ = thresholdMv;
  triggerState_ = RailTriggerState::Armed;
  triggerUs_ = 0U;
  postTriggerRemaining_ = 0U;
  frozenTriggerIndex_ = 0U;
}

void RailMonitor::disarmTrigger() {
  triggerState_ = RailTriggerState::Idle;
  postTriggerRemaining_ = 0U;
}

RailTriggerState RailMonitor::triggerState() const { return triggerState_; }
uint32_t RailMonitor::triggerThresholdMv() const { return triggerThresholdMv_; }

std::size_t RailMonitor::captureSize() const { return ringFill_; }
std::size_t RailMonitor::triggerIndex() const { return frozenTriggerIndex_; }

uint32_t RailMonitor::captureRawAt(std::size_t index) const {
  if (index >= ringFill_) {
    return 0U;
  }
  return ringRaw_[ringIndex(index)];
}

uint32_t RailMonitor::captureMvAt(std::size_t index) const {
  return railCountToMillivolts(calibration_, captureRawAt(index));
}

int32_t RailMonitor::captureRelativeUsAt(std::size_t index) const {
  if (index >= ringFill_) {
    return 0;
  }
  const uint32_t sampleUs = ringUs_[ringIndex(index)];
  // Wraparound-safe signed difference: the unsigned delta is reinterpreted,
  // so samples before the trigger come back negative.
  return static_cast<int32_t>(sampleUs - triggerUs_);
}

}  // namespace robot_arm
