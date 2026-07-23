/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef SRC_BOUNDED_SPSC_QUEUE_H_
#define SRC_BOUNDED_SPSC_QUEUE_H_

#include <array>
#include <atomic>
#include <cstddef>

// Fixed-storage, bounded single-producer/single-consumer queue. StorageCapacity
// retains the legacy driver's allocation while MaxDepth limits live backlog.
// One physical slot is always reserved so equal indices mean empty.
template <typename T, std::size_t StorageCapacity, std::size_t MaxDepth>
class BoundedSpscQueue {
 public:
  static_assert(StorageCapacity > 1, "SPSC queue needs at least two slots");
  static_assert(MaxDepth > 0, "SPSC queue backlog must be positive");
  static_assert(MaxDepth < StorageCapacity,
                "SPSC queue reserves one slot to distinguish full from empty");

  BoundedSpscQueue() : read_index_(0), write_index_(0) {}

  BoundedSpscQueue(const BoundedSpscQueue &) = delete;
  BoundedSpscQueue &operator=(const BoundedSpscQueue &) = delete;

  bool try_push(const T &value, std::size_t *size_after_push = nullptr) {
    const std::size_t write = write_index_.load(std::memory_order_relaxed);
    const std::size_t read = read_index_.load(std::memory_order_acquire);
    const std::size_t current_size = distance(read, write);
    if (current_size >= MaxDepth) {
      return false;
    }

    storage_[write] = value;
    write_index_.store(next(write), std::memory_order_release);
    if (size_after_push != nullptr) {
      *size_after_push = current_size + 1;
    }
    return true;
  }

  bool try_pop(T &value) {
    const std::size_t read = read_index_.load(std::memory_order_relaxed);
    const std::size_t write = write_index_.load(std::memory_order_acquire);
    if (read == write) {
      return false;
    }

    value = storage_[read];
    read_index_.store(next(read), std::memory_order_release);
    return true;
  }

  // Safe only while the producer and consumer are stopped.
  void reset() {
    read_index_.store(0, std::memory_order_relaxed);
    write_index_.store(0, std::memory_order_relaxed);
  }

  std::size_t approximate_size() const {
    const std::size_t read = read_index_.load(std::memory_order_acquire);
    const std::size_t write = write_index_.load(std::memory_order_acquire);
    return distance(read, write);
  }

  static constexpr std::size_t max_depth() { return MaxDepth; }
  static constexpr std::size_t storage_capacity() { return StorageCapacity; }

 private:
  static std::size_t next(std::size_t index) {
    ++index;
    return index == StorageCapacity ? 0 : index;
  }

  static std::size_t distance(std::size_t read, std::size_t write) {
    return write >= read ? write - read : StorageCapacity - read + write;
  }

  std::array<T, StorageCapacity> storage_{};
  alignas(64) std::atomic<std::size_t> read_index_;
  alignas(64) std::atomic<std::size_t> write_index_;
};

#endif  // SRC_BOUNDED_SPSC_QUEUE_H_
