#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>

#include "bounded_spsc_queue.h"

TEST(BoundedSpscQueue, EmptyAndFullBehavior) {
  BoundedSpscQueue<int, 5, 4> queue;
  int value = -1;

  EXPECT_FALSE(queue.try_pop(value));
  EXPECT_TRUE(queue.try_push(0));
  EXPECT_TRUE(queue.try_push(1));
  EXPECT_TRUE(queue.try_push(2));
  EXPECT_TRUE(queue.try_push(3));
  EXPECT_EQ(queue.approximate_size(), 4u);
  EXPECT_FALSE(queue.try_push(4));

  for (int expected = 0; expected < 4; ++expected) {
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, expected);
  }
  EXPECT_FALSE(queue.try_pop(value));
}

TEST(BoundedSpscQueue, PreservesFifoAcrossWrapAround) {
  BoundedSpscQueue<int, 5, 4> queue;
  int value = -1;

  ASSERT_TRUE(queue.try_push(0));
  ASSERT_TRUE(queue.try_push(1));
  ASSERT_TRUE(queue.try_push(2));
  ASSERT_TRUE(queue.try_pop(value));
  EXPECT_EQ(value, 0);
  ASSERT_TRUE(queue.try_pop(value));
  EXPECT_EQ(value, 1);

  ASSERT_TRUE(queue.try_push(3));
  ASSERT_TRUE(queue.try_push(4));
  ASSERT_TRUE(queue.try_push(5));
  for (int expected = 2; expected <= 5; ++expected) {
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, expected);
  }
}

TEST(BoundedSpscQueue, SurvivesMoreThanTwoFullWraps) {
  BoundedSpscQueue<std::uint32_t, 7, 6> queue;
  std::uint32_t next = 0;
  std::uint32_t value = 0;

  for (int wrap = 0; wrap < 10; ++wrap) {
    for (int i = 0; i < 6; ++i) {
      ASSERT_TRUE(queue.try_push(next++));
    }
    for (int i = 0; i < 6; ++i) {
      ASSERT_TRUE(queue.try_pop(value));
      EXPECT_EQ(value, static_cast<std::uint32_t>(wrap * 6 + i));
    }
  }
}

TEST(BoundedSpscQueue, EnforcesDriverMaximumBacklog) {
  BoundedSpscQueue<std::uint32_t, 36000, 400> queue;

  EXPECT_EQ(queue.storage_capacity(), 36000u);
  EXPECT_EQ(queue.max_depth(), 400u);
  for (std::uint32_t i = 0; i < 400; ++i) {
    ASSERT_TRUE(queue.try_push(i));
  }
  EXPECT_FALSE(queue.try_push(400));
}

TEST(BoundedSpscQueue, ResetDiscardsPendingValues) {
  BoundedSpscQueue<int, 8, 4> queue;
  int value = -1;

  ASSERT_TRUE(queue.try_push(10));
  ASSERT_TRUE(queue.try_push(11));
  queue.reset();
  EXPECT_EQ(queue.approximate_size(), 0u);
  EXPECT_FALSE(queue.try_pop(value));
  ASSERT_TRUE(queue.try_push(20));
  ASSERT_TRUE(queue.try_pop(value));
  EXPECT_EQ(value, 20);
}

TEST(BoundedSpscQueue, ConcurrentProducerConsumerPreservesUniqueOrdering) {
  constexpr std::uint64_t kPacketCount = 100000;
  BoundedSpscQueue<std::uint64_t, 1024, 400> queue;
  std::atomic<bool> valid{true};

  std::thread producer([&queue]() {
    for (std::uint64_t sequence = 0; sequence < kPacketCount; ++sequence) {
      while (!queue.try_push(sequence)) {
        std::this_thread::yield();
      }
    }
  });

  std::thread consumer([&queue, &valid]() {
    for (std::uint64_t expected = 0; expected < kPacketCount; ++expected) {
      std::uint64_t value = 0;
      while (!queue.try_pop(value)) {
        std::this_thread::yield();
      }
      if (value != expected) {
        valid.store(false);
      }
    }
  });

  producer.join();
  consumer.join();

  EXPECT_TRUE(valid.load());
  EXPECT_EQ(queue.approximate_size(), 0u);
}
