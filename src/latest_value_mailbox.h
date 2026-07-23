#ifndef HESAI_LIDAR_LATEST_VALUE_MAILBOX_H_
#define HESAI_LIDAR_LATEST_VALUE_MAILBOX_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <utility>

namespace hesai_lidar {
namespace internal {

enum class LatestValuePushResult {
  kStored,
  kReplaced,
  kStopped,
};

// A single-slot handoff for producers that must never wait for a slow
// consumer. If the consumer is busy, a new value replaces the pending value so
// stale work cannot accumulate.
template <typename T>
class LatestValueMailbox {
 public:
  LatestValueMailbox() = default;

  LatestValueMailbox(const LatestValueMailbox &) = delete;
  LatestValueMailbox &operator=(const LatestValueMailbox &) = delete;

  LatestValuePushResult push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) {
      return LatestValuePushResult::kStopped;
    }

    const bool replaced = static_cast<bool>(pending_);
    pending_.reset(new T(std::move(value)));
    condition_.notify_one();
    return replaced ? LatestValuePushResult::kReplaced
                    : LatestValuePushResult::kStored;
  }

  bool wait_pop(T &value) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() { return stopped_ || pending_; });
    if (!pending_) {
      return false;
    }

    value = std::move(*pending_);
    pending_.reset();
    return true;
  }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    pending_.reset();
    condition_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::unique_ptr<T> pending_;
  bool stopped_{false};
};

}  // namespace internal
}  // namespace hesai_lidar

#endif  // HESAI_LIDAR_LATEST_VALUE_MAILBOX_H_
